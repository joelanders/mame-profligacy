// license:BSD-3-Clause
//
// TMS57002 JIT implementation. See tms57002_jit.h.
#include "tms57002_jit.h"
#include "tms57002_ops.h"   // M2 pooled op-body emitter (op_poolable / emit_op_pooled)

// Host-architecture codegen backends. Both x86-64 and AArch64 are compiled and
// exercised by the Prophecy portability checks.
#if defined(__aarch64__)
#include <asmjit/a64.h>   // arm64 Compiler + emitter
#define KPROP_JIT_A64 1
#elif defined(__x86_64__)
#include <asmjit/x86.h>    // x86-64 Compiler + emitter
#define KPROP_JIT_X64 1
#endif
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace tms57002 {

namespace {
// C-callable wrappers so JIT-compiled code can call the device's per-op trampoline
// and idle check (asmjit invokes these by absolute address).
[[maybe_unused]] void jit_step(tms57002_device *d) { d->jit_step_one(); }                       // one program instruction (AArch64 trampoline)
[[maybe_unused]] unsigned jit_idle(tms57002_device *d) { return d->jit_is_idle() ? 1u : 0u; }   // program hit `idle` (AArch64 trampoline)

// Wrappers the native per-PC JIT calls (free functions -> the public seam methods).
void jit_w_op(tms57002_device *d, unsigned op, const void *i) { d->jit_op_exec(op, i); }
int jit_w_post(tms57002_device *d, int iipc, int ipc) { return d->jit_pc_post(iipc, ipc); }
void jit_w_xm(tms57002_device *d) { d->jit_xm_step(); }
[[maybe_unused]] void jit_w_xm_init(tms57002_device *d) { d->jit_xm_init(); }   // rde/wre arm the XRAM transaction (x86 pooled path only today)
void jit_w_run_rest(tms57002_device *d, int ipc) { d->jit_run_rest(ipc); }   // whole-frame fallback tail
[[maybe_unused]] u32 jit_w_get_cmem(tms57002_device *d, unsigned a) { return d->jit_get_cmem(u8(a)); }   // lacc deopt (x86 pooled path only today)
long s_forced_midframe_fallback_hits = 0;
void jit_w_forced_midframe_fallback_hit() { ++s_forced_midframe_fallback_hits; }
}  // namespace

// Compile: void run(tms57002_device* dev, int steps) — loop calling jit_step(dev)
// until the program idles or `steps` runs out. The interpreter trampoline today;
// native per-op codegen replaces the call site by op in later phases.
Jit::SampleFn Jit::compile_trampoline_loop()
{
#ifdef KPROP_JIT_A64
	using namespace asmjit;

	CodeHolder code;
	if (code.init(m_rt.environment()) != kErrorOk)
		return nullptr;
	a64::Compiler cc(&code);

	FuncNode *fn = cc.add_func(FuncSignature::build<void, tms57002_device *, int>());
	if (!fn)
		return nullptr;

	a64::Gp dev = cc.new_gpz("dev");
	a64::Gp steps = cc.new_gp32("steps");
	fn->set_arg(0, dev);
	fn->set_arg(1, steps);

	Label loop = cc.new_label();
	Label end = cc.new_label();

	cc.bind(loop);
	cc.cbz(steps, end);   // steps == 0 -> done

	InvokeNode *call_step = nullptr;
	cc.invoke(Out(call_step), uint64_t(&jit_step), FuncSignature::build<void, tms57002_device *>());
	call_step->set_arg(0, dev);

	a64::Gp idle = cc.new_gp32("idle");
	InvokeNode *call_idle = nullptr;
	cc.invoke(Out(call_idle), uint64_t(&jit_idle), FuncSignature::build<unsigned, tms57002_device *>());
	call_idle->set_arg(0, dev);
	call_idle->set_ret(0, idle);

	cc.cbnz(idle, end);   // idle -> done
	cc.sub(steps, steps, 1);
	cc.b(loop);

	cc.bind(end);
	cc.end_func();
	if (cc.finalize() != kErrorOk)
		return nullptr;

	SampleFn out = nullptr;
	if (m_rt.add(&out, &code) != kErrorOk)
		return nullptr;
	return out;
#else
	return nullptr;   // x86-64: trampoline loop not ported (run_sample_compiled falls back to run_sample)
#endif
}

std::array<u32, 4> Jit::run_sample_compiled(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps)
{
	if (!m_fn)
		m_fn = compile_trampoline_loop();
	if (!m_fn)
		return run_sample(dsp, in, max_steps);   // codegen unavailable -> C++ seam
	dsp.jit_begin_frame(in);
	m_fn(&dsp, max_steps > 0 ? max_steps : 1);
	return dsp.jit_end_frame();
}

std::array<u32, 4> Jit::run_sample(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps)
{
	dsp.jit_begin_frame(in);
	int guard = max_steps > 0 ? max_steps : 1;
	while (!dsp.jit_is_idle() && guard-- > 0)
		dsp.jit_step_one();
	return dsp.jit_end_frame();
}

#ifdef KPROP_JIT_A64
// Emit the per-PC jit_pc_post via the C++ helper (memory-based). Returns the next-ipc reg.
// compile_frame will replace this with a register-cached inline post; compile_pc keeps it.
static asmjit::a64::Gp emit_post_call(asmjit::a64::Compiler &cc, asmjit::a64::Gp dev, int iipc, asmjit::a64::Gp chain_ipc)
{
	using namespace asmjit;
	a64::Gp ret = cc.new_gp32("ret");
	InvokeNode *p = nullptr;
	cc.invoke(Out(p), uint64_t(&jit_w_post), FuncSignature::build<int, tms57002_device *, int, int>());
	p->set_arg(0, dev);
	p->set_arg(1, Imm(iipc));
	p->set_arg(2, chain_ipc);
	p->set_ret(0, ret);
	return ret;
}

// Emit ONE PC's chain body into an existing compiler (cc): inline jit_pc_pre (idle/MACC/
// xm-skip), the chain UNROLLED at compile time (jit_op_exec per micro-op + S_IDLE early-out
// to chain_end, terminator ++ca/++id native). Returns chain_ipc (the next-ipc) in a register;
// the CALLER emits the post. ok=false on a malformed chain. Reused by compile_pc (one PC = one
// function) and compile_frame (all PCs inlined into one frame fn). No runtime chain-walk/decode.
static asmjit::a64::Gp emit_pc_body(asmjit::a64::Compiler &cc, asmjit::a64::Gp dev, tms57002_device &dsp, int ipc, bool &ok)
{
	using namespace asmjit;

	a64::Gp chain_ipc = cc.new_gp32("chain_ipc");
	Label chain_end = cc.new_label();
	const int off_sti = int(tms57002_device::jit_off_sti());
	const unsigned idle_bit = __builtin_ctz(tms57002_device::jit_s_idle_mask());   // S_IDLE bit position
	const unsigned read_bit = __builtin_ctz(tms57002_device::jit_s_read_mask());
	const unsigned write_bit = __builtin_ctz(tms57002_device::jit_s_write_mask());
	const int off_macc = int(tms57002_device::jit_off_macc());
	const int off_macc_r = int(tms57002_device::jit_off_macc_read());
	const int off_macc_w = int(tms57002_device::jit_off_macc_write());
	const int off_ca = int(tms57002_device::jit_off_ca());
	const int off_id = int(tms57002_device::jit_off_id());

	// Inline jit_pc_pre: rare xm_step (skipped unless S_READ|S_WRITE) + native MACC pipeline.
	{
		a64::Gp sti = cc.new_gp32("sti");
		cc.ldr(sti, a64::ptr(dev, off_sti));
		Label call_xm = cc.new_label();
		Label after_xm = cc.new_label();
		cc.tbnz(sti, Imm(read_bit), call_xm);
		cc.tbnz(sti, Imm(write_bit), call_xm);
		cc.b(after_xm);
		cc.bind(call_xm);
		{
			InvokeNode *x = nullptr;
			cc.invoke(Out(x), uint64_t(&jit_w_xm), FuncSignature::build<void, tms57002_device *>());
			x->set_arg(0, dev);
		}
		cc.bind(after_xm);
		// macc_read = macc_write; macc_write = macc;   (s64, native 64-bit ldr/str)
		a64::Gp t = cc.new_gpz("t");
		cc.ldr(t, a64::ptr(dev, off_macc_w));
		cc.str(t, a64::ptr(dev, off_macc_r));
		cc.ldr(t, a64::ptr(dev, off_macc));
		cc.str(t, a64::ptr(dev, off_macc_w));
	}

	int cur = ipc;
	for (int steps = 0; ; ++steps)
	{
		if (steps > 64) { ok = false; return chain_ipc; }   // malformed chain (no terminator)
		const unsigned op = dsp.jit_inst_op(cur);
		const int next = dsp.jit_inst_next(cur);
		const uint64_t icd_addr = dsp.jit_inst_addr(cur);
		cc.mov(chain_ipc, Imm(next));   // ipc after this op (used if we break here)

		if (op < 4)
		{
			// chain terminator, inlined native: op0 nop, op1 ++ca, op2 ++id, op3 both (u8 wrap).
			if (op == 1 || op == 3)
			{
				a64::Gp r = cc.new_gp32("ca");
				cc.ldrb(r, a64::ptr(dev, off_ca));
				cc.add(r, r, Imm(1));
				cc.strb(r, a64::ptr(dev, off_ca));
			}
			if (op == 2 || op == 3)
			{
				a64::Gp r = cc.new_gp32("id");
				cc.ldrb(r, a64::ptr(dev, off_id));
				cc.add(r, r, Imm(1));
				cc.strb(r, a64::ptr(dev, off_id));
			}
			break;
		}

		{
			InvokeNode *m = nullptr;
			cc.invoke(Out(m), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>());
			m->set_arg(0, dev);
			m->set_arg(1, Imm(op));
			m->set_arg(2, Imm(icd_addr));
		}
		{
			// S_IDLE early-out, native (no helper call): if (sti & S_IDLE) goto chain_end.
			a64::Gp sti = cc.new_gp32("sti");
			cc.ldr(sti, a64::ptr(dev, off_sti));
			cc.tbnz(sti, Imm(idle_bit), chain_end);
		}
		cur = next;
	}

	cc.bind(chain_end);
	ok = true;
	return chain_ipc;   // the post is emitted by the caller (memory-based, or register-cached in compile_frame)
}

// Compile one PC's chain to int fn(device) returning the next ipc — emit_pc_body + ret.
Jit::PcFn Jit::compile_pc(tms57002_device &dsp, int ipc)
{
	using namespace asmjit;

	CodeHolder code;
	if (code.init(m_rt.environment()) != kErrorOk)
		return nullptr;
	a64::Compiler cc(&code);
	FuncNode *fn = cc.add_func(FuncSignature::build<int, tms57002_device *>());
	if (!fn)
		return nullptr;

	a64::Gp dev = cc.new_gpz("dev");
	fn->set_arg(0, dev);

	bool ok = true;
	a64::Gp chain_ipc = emit_pc_body(cc, dev, dsp, ipc, ok);
	if (!ok)
		return nullptr;
	a64::Gp ret = emit_post_call(cc, dev, ipc, chain_ipc);   // iipc == chain head
	cc.ret(ret);
	cc.end_func();
	if (cc.finalize() != kErrorOk)
		return nullptr;

	PcFn out = nullptr;
	if (m_rt.add(&out, &code) != kErrorOk)
		return nullptr;
	return out;
}
#elif defined(KPROP_JIT_X64)
// x86-64 port of the per-PC emitters (mirrors the a64 logic; memory-based machinery + helper calls
// for now — register-residency + op inlining come next, as proven by compile_frame_callfree's 24x).
static asmjit::x86::Gp emit_post_call(asmjit::x86::Compiler &cc, asmjit::x86::Gp dev, int iipc, asmjit::x86::Gp chain_ipc)
{
	using namespace asmjit;
	x86::Gp ret = cc.new_gp32("ret");
	InvokeNode *p = nullptr;
	cc.invoke(Out(p), uint64_t(&jit_w_post), FuncSignature::build<int, tms57002_device *, int, int>());
	p->set_arg(0, dev);
	p->set_arg(1, Imm(iipc));
	p->set_arg(2, chain_ipc);
	p->set_ret(0, ret);
	return ret;
}

static asmjit::x86::Gp emit_pc_body(asmjit::x86::Compiler &cc, asmjit::x86::Gp dev, tms57002_device &dsp, int ipc, bool &ok,
	asmjit::x86::Gp r_macc, asmjit::x86::Gp r_macc_r, asmjit::x86::Gp r_macc_w)
{
	using namespace asmjit;
	x86::Gp chain_ipc = cc.new_gp32("chain_ipc");
	Label chain_end = cc.new_label();
	const int off_sti = int(tms57002_device::jit_off_sti());
	const u32 idle_mask = tms57002_device::jit_s_idle_mask();
	const u32 rw_mask = tms57002_device::jit_s_read_mask() | tms57002_device::jit_s_write_mask();
	const int off_macc = int(tms57002_device::jit_off_macc());
	const int off_macc_r = int(tms57002_device::jit_off_macc_read());
	const int off_macc_w = int(tms57002_device::jit_off_macc_write());
	const int off_ca = int(tms57002_device::jit_off_ca());
	const int off_id = int(tms57002_device::jit_off_id());

	// macc/macc_read/macc_write are register-cached across the frame; sync to/from device memory
	// only around calls that may touch them (jit_w_xm, jit_w_op). Inlined ops (lacc/lira) don't.
	auto macc_out = [&]() { cc.mov(x86::qword_ptr(dev, off_macc), r_macc); cc.mov(x86::qword_ptr(dev, off_macc_r), r_macc_r); cc.mov(x86::qword_ptr(dev, off_macc_w), r_macc_w); };
	auto macc_in = [&]() { cc.mov(r_macc, x86::qword_ptr(dev, off_macc)); cc.mov(r_macc_r, x86::qword_ptr(dev, off_macc_r)); cc.mov(r_macc_w, x86::qword_ptr(dev, off_macc_w)); };

	// %wa(rr): aacc = saturate(s64 rr) — if out of int32 range set ST1_AOV, and if ST1_AOVM clamp
	// to [-2^31, 2^31-1]; then aacc = (u32)rr. Shared by add/sub/neg (the tmsinstr.lst %wa macro).
	const int off_aacc_wa = int(tms57002_device::jit_off_aacc());
	const int off_st1_wa = int(tms57002_device::jit_off_st1());
	auto emit_wa = [&](x86::Gp rr) {
		x86::Gp chk = cc.new_gpz();
		cc.movsxd(chk, rr.r32()); cc.cmp(chk, rr);
		Label no_ovf = cc.new_label();
		cc.je(no_ovf);
		cc.or_(x86::dword_ptr(dev, off_st1_wa), Imm(tms57002_device::jit_st1_aov()));
		Label no_clamp = cc.new_label();
		cc.test(x86::dword_ptr(dev, off_st1_wa), Imm(tms57002_device::jit_st1_aovm()));
		cc.jz(no_clamp);
		x86::Gp hi = cc.new_gpz(), lo = cc.new_gpz();
		cc.mov(hi, Imm(2147483647LL)); cc.cmp(rr, hi); cc.cmovg(rr, hi);
		cc.mov(lo, Imm(-2147483648LL)); cc.cmp(rr, lo); cc.cmovl(rr, lo);
		cc.bind(no_clamp);
		cc.bind(no_ovf);
		cc.mov(x86::dword_ptr(dev, off_aacc_wa), rr.r32());
	};
	(void)emit_wa;

	// Inline jit_pc_pre: rare xm_step (skipped unless S_READ|S_WRITE) + MACC pipeline (registers).
	{
		x86::Gp sti = cc.new_gp32("sti");
		cc.mov(sti, x86::dword_ptr(dev, off_sti));
		Label call_xm = cc.new_label(), after_xm = cc.new_label();
		cc.test(sti, Imm(rw_mask));
		cc.jnz(call_xm);
		cc.jmp(after_xm);
		cc.bind(call_xm);
		macc_out();
		{
			InvokeNode *x = nullptr;
			cc.invoke(Out(x), uint64_t(&jit_w_xm), FuncSignature::build<void, tms57002_device *>());
			x->set_arg(0, dev);
		}
		macc_in();
		cc.bind(after_xm);
		cc.mov(r_macc_r, r_macc_w);   // macc_read = macc_write; macc_write = macc;  (registers)
		cc.mov(r_macc_w, r_macc);
	}

	int cur = ipc;
	for (int steps = 0; ; ++steps)
	{
		if (steps > 64) { ok = false; return chain_ipc; }
		const unsigned op = dsp.jit_inst_op(cur);
		if (op >= 4 && !dsp.jit_op_mnemonic(op)) { ok = false; return chain_ipc; }   // out-of-range op (a
			// stale/churned decode-cache entry in the live machine) -> bail; the caller runs this frame via
			// the interpreter instead of baking a deopt that would hand jit_op_exec a fatal opcode.
		const int next = dsp.jit_inst_next(cur);
		const uint64_t icd_addr = dsp.jit_inst_addr(cur);
		cc.mov(chain_ipc, Imm(next));
		if (op < 4)
		{
			if (op == 1 || op == 3) { x86::Gp r = cc.new_gp32(); cc.movzx(r, x86::byte_ptr(dev, off_ca)); cc.add(r, Imm(1)); cc.mov(x86::byte_ptr(dev, off_ca), r.r8()); }
			if (op == 2 || op == 3) { x86::Gp r = cc.new_gp32(); cc.movzx(r, x86::byte_ptr(dev, off_id)); cc.add(r, Imm(1)); cc.mov(x86::byte_ptr(dev, off_id), r.r8()); }
			break;
		}
		const char *mn = dsp.jit_op_mnemonic(op);
		if (mn && __builtin_strcmp(mn, "lira") == 0)
		{
			// lira: id = aacc >> 24 — native, no call. Doesn't touch sti, so no idle check needed.
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			x86::Gp a = cc.new_gp32();
			cc.mov(a, x86::dword_ptr(dev, off_aacc));
			cc.shr(a, Imm(24));
			cc.mov(x86::byte_ptr(dev, off_id), a.r8());
		}
		else if (mn && __builtin_strcmp(mn, "lcak") == 0)
		{
			// lcak: ca = i->param (immediate). Native, no call; doesn't touch sti.
			cc.mov(x86::byte_ptr(dev, off_ca), Imm(u8(dsp.jit_inst_param(cur))));
		}
		else if (mn && __builtin_strcmp(mn, "lirk") == 0)
		{
			// lirk: id = i->param (immediate). Native, no call; doesn't touch sti.
			cc.mov(x86::byte_ptr(dev, off_id), Imm(u8(dsp.jit_inst_param(cur))));
		}
		else if (mn && __builtin_strcmp(mn, "lacc") == 0 && dsp.jit_op_variant(op) == 0)
		{
			// lacc: aacc = get_cmem(param). FAST PATH aacc = cmem[param] when no cmem update is
			// pending and no debug force; DEOPT to jit_w_get_cmem otherwise. Doesn't set S_IDLE.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			{ x86::Gp v = cc.new_gp32(); cc.mov(v, x86::dword_ptr(dev, off_cmem + int(param) * 4)); cc.mov(x86::dword_ptr(dev, off_aacc), v); }
			cc.jmp(doneop);
			cc.bind(deopt);
			{
				InvokeNode *g = nullptr;
				cc.invoke(Out(g), uint64_t(&jit_w_get_cmem), FuncSignature::build<u32, tms57002_device *, unsigned>());
				g->set_arg(0, dev);
				g->set_arg(1, Imm(param));
				x86::Gp v = cc.new_gp32();
				g->set_ret(0, v);
				cc.mov(x86::dword_ptr(dev, off_aacc), v);
			}
			cc.bind(doneop);
		}
		else if (mn && __builtin_strcmp(mn, "smhd") == 0 && !dsp.jit_smhd_raw_path()
			&& (dsp.jit_op_variant(op) == 0 || dsp.jit_op_variant(op) == 80))
		{
			// smhd sfmo=0 dmode=0 dbp=0 rnd=0: dmem0[(param+ba0)&0xff] = (macc_read >> 24) & 0xffffff (the
			// common, no-overflow case of macc_to_output_0[s]). variant 0 (movm=0): overflow just sets
			// ST1_MOV (native). variant 80 (movm=1): overflow ALSO clamps macc, which we DEOPT to the
			// interpreter (rare — normal audio doesn't overflow this, so the common path stays native and
			// the clamp stays oracle-correct rather than un-exercised JIT code). Reads register-cached
			// r_macc_r; writes dmem0; no cmem read / macc write / S_IDLE. Variant indices 0/80 pin
			// sfmo=dmode=dbp=rnd=0; sfmo>0 (v84/v88) and other variants deopt via the generic else.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const bool movm = dsp.jit_op_variant(op) == 80;   // v80 = movm=1 -> overflow clamps -> deopt it
			const int off_st1 = int(tms57002_device::jit_off_st1());
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			const u32 st1_mov = tms57002_device::jit_st1_mov();
			Label smhd_done = cc.new_label();
			{   // overflow: m1 = macc_read & 0xf800000000000; over if (m1 != 0 && m1 != mask)
				Label no_ovf = cc.new_label();
				x86::Gp m1 = cc.new_gpz(), mask = cc.new_gpz();
				cc.mov(mask, Imm(0xf800000000000ULL));
				cc.mov(m1, r_macc_r);
				cc.and_(m1, mask);
				cc.jz(no_ovf);
				cc.cmp(m1, mask);
				cc.je(no_ovf);
				if (movm) {   // v80 overflow needs the macc clamp -> let the interpreter run the whole op
					macc_out();
					InvokeNode *dn = nullptr;
					cc.invoke(Out(dn), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>());
					dn->set_arg(0, dev);
					dn->set_arg(1, Imm(op));
					dn->set_arg(2, Imm(icd_addr));
					macc_in();
					cc.jmp(smhd_done);
				} else {
					cc.or_(x86::dword_ptr(dev, off_st1), Imm(st1_mov));   // v0 overflow -> ST1_MOV (native)
				}
				cc.bind(no_ovf);
			}
			x86::Gp out = cc.new_gpz();                           // out = (macc_read >> 24) & 0xffffff
			cc.mov(out, r_macc_r);
			cc.sar(out, Imm(24));
			cc.and_(out, Imm(0xffffff));
			x86::Gp addr = cc.new_gpz();                          // dmem0[(param + ba0) & 0xff] = out
			cc.movzx(addr, x86::byte_ptr(dev, off_ba0));
			cc.add(addr, Imm(param));
			cc.and_(addr, Imm(0xff));
			cc.mov(x86::dword_ptr(dev, addr, 2, off_dmem0), out.r32());
			cc.bind(smhd_done);
		}
		else if (mn && __builtin_strcmp(mn, "sacd") == 0 && dsp.jit_op_variant(op) == 0)
		{
			// sacd: dmem0[(param+ba0)&0xff] = (aacc >> 8) & 0xffffff. No cmem, no macc, no flags,
			// no S_IDLE -> native inline (write_dmem dmem0 path is a plain masked store).
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			x86::Gp out = cc.new_gp32();
			cc.mov(out, x86::dword_ptr(dev, off_aacc));
			cc.shr(out, Imm(8));
			cc.and_(out, Imm(0xffffff));
			x86::Gp addr = cc.new_gpz();
			cc.movzx(addr, x86::byte_ptr(dev, off_ba0));
			cc.add(addr, Imm(param));
			cc.and_(addr, Imm(0xff));
			cc.mov(x86::dword_ptr(dev, addr, 2, off_dmem0), out);
		}
		else if (mn && __builtin_strcmp(mn, "srbd") == 0 && dsp.jit_op_variant(op) == 0)
		{
			// srbd (dmode=0): dmem0[(param+ba0)&0xff] = xrd & 0xffffff. No cmem/macc/S_IDLE.
			// NOTE: srbd has dmode/dbp variants; if DSP1 uses a non-dmem0 one this MISMATCHes
			// (the gate catches it) -> revert. Mirrors the smhd/sacd simple-store pattern.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_xrd = int(tms57002_device::jit_off_xrd());
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			x86::Gp out = cc.new_gp32();
			cc.mov(out, x86::dword_ptr(dev, off_xrd));
			cc.and_(out, Imm(0xffffff));
			x86::Gp addr = cc.new_gpz();
			cc.movzx(addr, x86::byte_ptr(dev, off_ba0));
			cc.add(addr, Imm(param));
			cc.and_(addr, Imm(0xff));
			cc.mov(x86::dword_ptr(dev, addr, 2, off_dmem0), out);
		}
		else if (mn && __builtin_strcmp(mn, "lacd") == 0 && dsp.jit_op_variant(op) == 0)
		{
			// lacd v0 (dmode=0 sfai=0 dbp=0): aacc = dmem0[(param+ba0)&0xff] << 8. No cmem/macc -> native,
			// validatable (writes aacc). Non-0 variants (sfai/dmode/dbp) deopt via the generic else.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			x86::Gp addr = cc.new_gpz();
			cc.movzx(addr, x86::byte_ptr(dev, off_ba0));
			cc.add(addr, Imm(param));
			cc.and_(addr, Imm(0xff));
			x86::Gp d = cc.new_gp32();
			cc.mov(d, x86::dword_ptr(dev, addr, 2, off_dmem0));   // dmem0[(param+ba0)&0xff]
			cc.shl(d, Imm(8));                                    // << 8
			cc.mov(x86::dword_ptr(dev, off_aacc), d);             // aacc = d
		}
		else if ((op == 954 || op == 938) && !dsp.jit_mpy_forward_on())
		{
			// mac (954) / mpy (938), cmode=1 dmode=0 dbp=0 sfma=0 — the hot cmem-read multiply-
			// accumulate, REGISTER-RESIDENT (operates on r_macc directly; no macc spill/call — the
			// residency lever 1(a) validated). Normal contract:
			//   d = sext24(dmem0[(param+ba0)&0xff]);  c = cmem[ca] (fast; DEOPT if update pending);
			//   creg = c;  r = (int64_t)(int32_t)c * (int64_t)(int32_t)d;  product = r >> 7;
			//   mac: macc += product;   mpy: macc = product.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const bool is_mac = (op == 954);
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			const int off_ca = int(tms57002_device::jit_off_ca());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_creg = int(tms57002_device::jit_off_creg());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			x86::Gp addr = cc.new_gpz();
			cc.movzx(addr, x86::byte_ptr(dev, off_ba0)); cc.add(addr, Imm(param)); cc.and_(addr, Imm(0xff));
			x86::Gp d2 = cc.new_gp32();
			cc.mov(d2, x86::dword_ptr(dev, addr, 2, off_dmem0));   // dmem0[addr]
			cc.shl(d2, Imm(8)); cc.sar(d2, Imm(8));                // sign-extend bit23 -> 32
			x86::Gp cai = cc.new_gpz();
			cc.movzx(cai, x86::byte_ptr(dev, off_ca));
			x86::Gp c = cc.new_gp32();
			cc.mov(c, x86::dword_ptr(dev, cai, 2, off_cmem));      // c = cmem[ca]
			cc.mov(x86::dword_ptr(dev, off_creg), c);             // creg = c
			x86::Gp rc = cc.new_gpz(), rd = cc.new_gpz();
			cc.movsxd(rc, c); cc.movsxd(rd, d2);
			cc.imul(rc, rd);                                      // r = (int64)c * (int64)d
			cc.sar(rc, Imm(7));                                   // product = r >> 7
			if (is_mac) cc.add(r_macc, rc); else cc.mov(r_macc, rc);   // macc += / = product (REGISTER)
			cc.jmp(doneop);
			cc.bind(deopt);
			macc_out();
			{
				InvokeNode *mm = nullptr;
				cc.invoke(Out(mm), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>());
				mm->set_arg(0, dev); mm->set_arg(1, Imm(op)); mm->set_arg(2, Imm(icd_addr));
			}
			macc_in();
			cc.bind(doneop);
			// mac/mpy don't set S_IDLE -> no idle check.
		}
		else if (op == 1121)
		{
			cc.xor_(r_macc, r_macc);   // zmac: macc = 0
		}
		else if (op == 985 || op == 949)
		{
			// mac 985 (c=aacc, macc +=, creg=aacc) / mpy 949 (c=creg, macc =), d=sext(dmem[param+ba0]),
			// dc shift (r>>7). No cmem read -> no deopt.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const bool is_mac = (op == 985);
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_creg = int(tms57002_device::jit_off_creg());
			x86::Gp addr = cc.new_gpz(); cc.movzx(addr, x86::byte_ptr(dev, off_ba0)); cc.add(addr, Imm(param)); cc.and_(addr, Imm(0xff));
			x86::Gp d = cc.new_gp32(); cc.mov(d, x86::dword_ptr(dev, addr, 2, off_dmem0)); cc.shl(d, Imm(8)); cc.sar(d, Imm(8));
			x86::Gp rc = cc.new_gpz(), rd = cc.new_gpz();
			if (is_mac) {
				// mac a,%d routes %a to the multiplier's full-width C input. The 24-bit
				// truncation applies only when %a is the second operand (the c,a forms).
				x86::Gp t = cc.new_gp32(); cc.mov(t, x86::dword_ptr(dev, off_aacc));
				cc.mov(x86::dword_ptr(dev, off_creg), t);
				cc.movsxd(rc, t);
			} else {
				cc.movsxd(rc, x86::dword_ptr(dev, off_creg));   // c = creg
			}
			cc.movsxd(rd, d);
			cc.imul(rc, rd); cc.sar(rc, Imm(7));
			if (is_mac) cc.add(r_macc, rc); else cc.mov(r_macc, rc);
		}
		else if (op == 1018 || op == 945 || op == 946)
		{
			// mac 1018 (c=cmem[ca],+=) / mpy 945 (c=cmem[param],=) / mpy 946 (c=cmem[ca],=),
			// d=aacc, ca shift (r>>15). creg = c. Fast cmem; DEOPT if pending.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const bool is_mac = (op == 1018);
			const bool by_ca = (op == 1018 || op == 946);
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_ca = int(tms57002_device::jit_off_ca());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_creg = int(tms57002_device::jit_off_creg());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			x86::Gp c = cc.new_gp32();
			if (by_ca) { x86::Gp cai = cc.new_gpz(); cc.movzx(cai, x86::byte_ptr(dev, off_ca)); cc.mov(c, x86::dword_ptr(dev, cai, 2, off_cmem)); }
			else       cc.mov(c, x86::dword_ptr(dev, off_cmem + int(param) * 4));
			cc.mov(x86::dword_ptr(dev, off_creg), c);
			x86::Gp rc = cc.new_gpz(), rd = cc.new_gpz();
			cc.movsxd(rc, c);
			x86::Gp av = cc.new_gp32(); cc.mov(av, x86::dword_ptr(dev, off_aacc));
			cc.and_(av, Imm(0xffffff00U));                  // d = 24-bit multiplier A-port view of aacc
			cc.movsxd(rd, av);
			cc.imul(rc, rd); cc.sar(rc, Imm(15));
			if (is_mac) cc.add(r_macc, rc); else cc.mov(r_macc, rc);
			cc.jmp(doneop);
			cc.bind(deopt);
			macc_out();
			{ InvokeNode *mv = nullptr; cc.invoke(Out(mv), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>()); mv->set_arg(0, dev); mv->set_arg(1, Imm(op)); mv->set_arg(2, Imm(icd_addr)); }
			macc_in();
			cc.bind(doneop);
		}
		else if (op == 1131 || op == 1130)
		{
			// lmhc: macc = macc_write = ((int64)(int32)get_cmem(ca|param)) << 16. Fast cmem; DEOPT if pending.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const bool by_ca = (op == 1131);
			const int off_ca = int(tms57002_device::jit_off_ca());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			x86::Gp v = cc.new_gp32();
			if (by_ca) { x86::Gp cai = cc.new_gpz(); cc.movzx(cai, x86::byte_ptr(dev, off_ca)); cc.mov(v, x86::dword_ptr(dev, cai, 2, off_cmem)); }
			else       cc.mov(v, x86::dword_ptr(dev, off_cmem + int(param) * 4));
			cc.movsxd(r_macc, v); cc.shl(r_macc, Imm(16)); cc.mov(r_macc_w, r_macc);
			cc.jmp(doneop);
			cc.bind(deopt);
			macc_out();
			{ InvokeNode *ml = nullptr; cc.invoke(Out(ml), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>()); ml->set_arg(0, dev); ml->set_arg(1, Imm(op)); ml->set_arg(2, Imm(icd_addr)); }
			macc_in();
			cc.bind(doneop);
		}
		else if ((op == 1122 || op == 1123) && !dsp.jit_lmhd_forward_on())
		{
			// lmhd: macc = macc_write = ((int64)(int32)(dmem0[((param|id)+ba0)&0xff]<<8)) << 16. No cmem.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const bool by_id = (op == 1123);
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			const int off_id = int(tms57002_device::jit_off_id());
			x86::Gp addr = cc.new_gpz(); cc.movzx(addr, x86::byte_ptr(dev, off_ba0));
			if (by_id) { x86::Gp idr = cc.new_gpz(); cc.movzx(idr, x86::byte_ptr(dev, off_id)); cc.add(addr, idr); }
			else       cc.add(addr, Imm(param));
			cc.and_(addr, Imm(0xff));
			x86::Gp v = cc.new_gp32(); cc.mov(v, x86::dword_ptr(dev, addr, 2, off_dmem0)); cc.shl(v, Imm(8));
			cc.movsxd(r_macc, v); cc.shl(r_macc, Imm(16)); cc.mov(r_macc_w, r_macc);
		}
		else if (op == 1843)
		{
			// sfmo (op 1843): st1 = st1 & ~ST1_SFMO (set sfmo mode 0). No macc/aacc/S_IDLE — a pure
			// st1 write. Inlining it stops the generic fallback from spilling macc around a call for a
			// non-macc op (the residency lever: keep macc register-resident across it). 28x/DSP1 frame.
			const int off_st1 = int(tms57002_device::jit_off_st1());
			cc.and_(x86::dword_ptr(dev, off_st1), Imm(~tms57002_device::jit_st1_sfmo()));
		}
		else if (op == 436)
		{
			// sub (op 436, cmode=0 sfao=0): aacc = %wa( (int32)get_cmem(param) - (int32)aacc ).
			// No macc. Tests the %wa write-aacc-with-saturation macro (overflow -> ST1_AOV; AOVM -> clamp).
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_st1 = int(tms57002_device::jit_off_st1());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			x86::Gp rr = cc.new_gpz();
			cc.movsxd(rr, x86::dword_ptr(dev, off_cmem + int(param) * 4));   // rr = (int64)(int32)cmem[param]
			x86::Gp ra = cc.new_gpz();
			cc.movsxd(ra, x86::dword_ptr(dev, off_aacc));                   // ra = (int64)(int32)aacc
			cc.sub(rr, ra);                                                // rr = c - aacc
			// %wa(rr) -> aacc : if (rr out of int32) { st1|=AOV; if(st1&AOVM) clamp }; aacc = (u32)rr
			{
				x86::Gp chk = cc.new_gpz();
				cc.movsxd(chk, rr.r32()); cc.cmp(chk, rr);
				Label no_ovf = cc.new_label();
				cc.je(no_ovf);
				cc.or_(x86::dword_ptr(dev, off_st1), Imm(tms57002_device::jit_st1_aov()));
				Label no_clamp = cc.new_label();
				cc.test(x86::dword_ptr(dev, off_st1), Imm(tms57002_device::jit_st1_aovm()));
				cc.jz(no_clamp);
				x86::Gp hi = cc.new_gpz(), lo = cc.new_gpz();
				cc.mov(hi, Imm(2147483647LL)); cc.cmp(rr, hi); cc.cmovg(rr, hi);
				cc.mov(lo, Imm(-2147483648LL)); cc.cmp(rr, lo); cc.cmovl(rr, lo);
				cc.bind(no_clamp);
				cc.bind(no_ovf);
			}
			cc.mov(x86::dword_ptr(dev, off_aacc), rr.r32());               // aacc = (u32)rr
			cc.jmp(doneop);
			cc.bind(deopt);
			macc_out();
			{
				InvokeNode *ms = nullptr;
				cc.invoke(Out(ms), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>());
				ms->set_arg(0, dev); ms->set_arg(1, Imm(op)); ms->set_arg(2, Imm(icd_addr));
			}
			macc_in();
			cc.bind(doneop);
		}
		else if (op == 1844 || op == 1845 || op == 1846)
		{
			// sfmo 1/2/3: st1 = (st1 & ~ST1_SFMO) | (val << shift). st1 write, no macc.
			const int off_st1 = int(tms57002_device::jit_off_st1());
			const u32 val = (op == 1844) ? 1u : (op == 1845) ? 2u : 3u;
			cc.and_(x86::dword_ptr(dev, off_st1), Imm(~tms57002_device::jit_st1_sfmo()));
			cc.or_(x86::dword_ptr(dev, off_st1), Imm(val << tms57002_device::jit_st1_sfmo_shift()));
		}
		else if (op == 1837 || op == 1838)
		{
			// sfai clear (1837) / set (1838).
			const int off_st1 = int(tms57002_device::jit_off_st1());
			if (op == 1837) cc.and_(x86::dword_ptr(dev, off_st1), Imm(~tms57002_device::jit_st1_sfai()));
			else            cc.or_(x86::dword_ptr(dev, off_st1), Imm(tms57002_device::jit_st1_sfai()));
		}
		else if (op == 1132)
		{
			// sfml: macc = (macc & 0x8000000000000) | ((macc << 1) & 0x7ffffffffffff). Register-resident.
			x86::Gp hi = cc.new_gpz(), m = cc.new_gpz();
			cc.mov(m, Imm(0x8000000000000LL)); cc.mov(hi, r_macc); cc.and_(hi, m);
			cc.shl(r_macc, Imm(1));
			cc.mov(m, Imm(0x7ffffffffffffLL)); cc.and_(r_macc, m);
			cc.or_(r_macc, hi);
		}
		else if (op == 868 || op == 920)
		{
			// and/xor cmode=1: aacc &= / ^= get_cmem(ca). No macc. Fast cmem[ca]; DEOPT if pending.
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_ca = int(tms57002_device::jit_off_ca());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			x86::Gp cai = cc.new_gpz(); cc.movzx(cai, x86::byte_ptr(dev, off_ca));
			x86::Gp c = cc.new_gp32(); cc.mov(c, x86::dword_ptr(dev, cai, 2, off_cmem));
			if (op == 868) cc.and_(x86::dword_ptr(dev, off_aacc), c);
			else           cc.xor_(x86::dword_ptr(dev, off_aacc), c);
			cc.jmp(doneop);
			cc.bind(deopt);
			macc_out();
			{ InvokeNode *mx = nullptr; cc.invoke(Out(mx), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>()); mx->set_arg(0, dev); mx->set_arg(1, Imm(op)); mx->set_arg(2, Imm(icd_addr)); }
			macc_in();
			cc.bind(doneop);
		}
		else if (op == 859)
		{
			// and dmode=0: aacc &= (dmem0[(param+ba0)&0xff] << 8). No cmem/macc.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			x86::Gp addr = cc.new_gpz(); cc.movzx(addr, x86::byte_ptr(dev, off_ba0)); cc.add(addr, Imm(param)); cc.and_(addr, Imm(0xff));
			x86::Gp d = cc.new_gp32(); cc.mov(d, x86::dword_ptr(dev, addr, 2, off_dmem0)); cc.shl(d, Imm(8));
			cc.and_(x86::dword_ptr(dev, off_aacc), d);
		}
		else if (op == 6)
		{
			// neg: aacc = %wa( -(int64)aacc ).  (int64)aacc is ZERO-extended (aacc is u32).
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			x86::Gp rr = cc.new_gpz();
			cc.mov(rr.r32(), x86::dword_ptr(dev, off_aacc));   // zero-extend u32 aacc into rr
			cc.neg(rr);
			emit_wa(rr);
		}
		else if (op == 8 || op == 428)
		{
			// add(8)/sub(428) dmode=0: aacc = %wa( (int32)(dmem0[(param+ba0)&0xff]<<8) +/- (int32)aacc ).
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			x86::Gp addr = cc.new_gpz(); cc.movzx(addr, x86::byte_ptr(dev, off_ba0)); cc.add(addr, Imm(param)); cc.and_(addr, Imm(0xff));
			x86::Gp d = cc.new_gp32(); cc.mov(d, x86::dword_ptr(dev, addr, 2, off_dmem0)); cc.shl(d, Imm(8));
			x86::Gp rr = cc.new_gpz(); cc.movsxd(rr, d);
			x86::Gp ra = cc.new_gpz(); cc.movsxd(ra, x86::dword_ptr(dev, off_aacc));
			if (op == 8) cc.add(rr, ra); else cc.sub(rr, ra);
			emit_wa(rr);
		}
		else if (op == 4)
		{
			// abs: if((int32)aacc < 0){ aacc = -aacc; if((int32)aacc < 0) st1 |= ST1_AOV; }
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_st1 = int(tms57002_device::jit_off_st1());
			x86::Gp a = cc.new_gp32(); cc.mov(a, x86::dword_ptr(dev, off_aacc));
			Label done = cc.new_label();
			cc.test(a, a); cc.jns(done);
			cc.neg(a); cc.mov(x86::dword_ptr(dev, off_aacc), a);
			cc.test(a, a); cc.jns(done);
			cc.or_(x86::dword_ptr(dev, off_st1), Imm(tms57002_device::jit_st1_aov()));
			cc.bind(done);
		}
		else if (op == 380)
		{
			// add 380 cmode=0 sfmo=0: aacc = %wa( (int32)get_cmem(param) + (macc_read >> 16) ).
			// macc_to_output_0s(0,~0,0) common path == macc_read; DEOPT on macc overflow (rare:
			// st1|=ST1_MOV + clip) or pending cmem update. Validates the %mo (macc-output) pattern.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			// macc overflow -> deopt: m1 = macc_read & 0xf800000000000; over if m1 != 0 && m1 != mask
			{
				x86::Gp m1 = cc.new_gpz(), mask = cc.new_gpz();
				cc.mov(mask, Imm(0xf800000000000LL)); cc.mov(m1, r_macc_r); cc.and_(m1, mask);
				Label no_ovf = cc.new_label();
				cc.jz(no_ovf); cc.cmp(m1, mask); cc.jne(deopt);
				cc.bind(no_ovf);
			}
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			x86::Gp mo = cc.new_gpz(); cc.mov(mo, r_macc_r); cc.sar(mo, Imm(16));   // macc_read >> 16
			x86::Gp rr = cc.new_gpz(); cc.movsxd(rr, x86::dword_ptr(dev, off_cmem + int(param) * 4));  // (int32)cmem[param]
			cc.add(rr, mo);
			emit_wa(rr);
			cc.jmp(doneop);
			cc.bind(deopt);
			macc_out();
			{ InvokeNode *ma = nullptr; cc.invoke(Out(ma), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>()); ma->set_arg(0, dev); ma->set_arg(1, Imm(op)); ma->set_arg(2, Imm(icd_addr)); }
			macc_in();
			cc.bind(doneop);
		}
		else if (op == 1144)
		{
			// sacc cmode=0: cmem[param] = aacc  (direct DSP cmem write; the queue is only for H8 uploads).
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_aacc = int(tms57002_device::jit_off_aacc());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			x86::Gp v = cc.new_gp32(); cc.mov(v, x86::dword_ptr(dev, off_aacc));
			cc.mov(x86::dword_ptr(dev, off_cmem + int(param) * 4), v);
		}
		else if (op == 1768)
		{
			// smhc 1768 cmode=0 sfmo=0: cmem[param] = (u32)(macc_read >> 16). DEOPT on macc overflow.
			const u8 param = u8(dsp.jit_inst_param(cur));
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			Label deopt = cc.new_label(), doneop = cc.new_label();
			{ x86::Gp m1 = cc.new_gpz(), mask = cc.new_gpz(); cc.mov(mask, Imm(0xf800000000000LL)); cc.mov(m1, r_macc_r); cc.and_(m1, mask); Label no_ovf = cc.new_label(); cc.jz(no_ovf); cc.cmp(m1, mask); cc.jne(deopt); cc.bind(no_ovf); }
			x86::Gp mo = cc.new_gpz(); cc.mov(mo, r_macc_r); cc.sar(mo, Imm(16));
			cc.mov(x86::dword_ptr(dev, off_cmem + int(param) * 4), mo.r32());   // cmem[param] = (u32)(macc_read>>16)
			cc.jmp(doneop);
			cc.bind(deopt);
			macc_out();
			{ InvokeNode *msc = nullptr; cc.invoke(Out(msc), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>()); msc->set_arg(0, dev); msc->set_arg(1, Imm(op)); msc->set_arg(2, Imm(icd_addr)); }
			macc_in();
			cc.bind(doneop);
		}
		else if (op == 1143 || op == 1142 || op == 1135)
		{
			// rde (1143 ca / 1142 param) / wre (1135 ca): arm the XRAM transaction.
			//   if (sti & (S_READ|S_WRITE)) return;   [wre: xwr = dmem0[(param+ba0)&0xff];]
			//   xoa = get_cmem(ca|param);  xm_init();  sti |= S_READ|S_WRITE;
			const u8 param = u8(dsp.jit_inst_param(cur));
			const bool is_wre = (op == 1135);
			const bool by_ca = (op == 1143 || op == 1135);
			const int off_xoa = int(tms57002_device::jit_off_xoa());
			const int off_xwr = int(tms57002_device::jit_off_xwr());
			const int off_ca = int(tms57002_device::jit_off_ca());
			const int off_cmem = int(tms57002_device::jit_off_cmem());
			const int off_dmem0 = int(tms57002_device::jit_off_dmem0());
			const int off_ba0 = int(tms57002_device::jit_off_ba0());
			const int off_ucc = int(tms57002_device::jit_off_uc_count());
			const int off_force = int(tms57002_device::jit_off_cmem_force());
			const u32 flag = is_wre ? tms57002_device::jit_s_write_mask() : tms57002_device::jit_s_read_mask();
			Label skip = cc.new_label(), deopt = cc.new_label(), done = cc.new_label();
			{ x86::Gp s = cc.new_gp32(); cc.mov(s, x86::dword_ptr(dev, off_sti)); cc.test(s, Imm(rw_mask)); cc.jnz(skip); }   // transaction pending -> nop
			{ x86::Gp c = cc.new_gp32(); cc.movzx(c, x86::byte_ptr(dev, off_ucc)); cc.test(c, c); cc.jnz(deopt); }
			{ x86::Gp f = cc.new_gp32(); cc.movzx(f, x86::byte_ptr(dev, off_force)); cc.test(f, f); cc.jnz(deopt); }
			if (is_wre) {
				x86::Gp addr = cc.new_gpz(); cc.movzx(addr, x86::byte_ptr(dev, off_ba0)); cc.add(addr, Imm(param)); cc.and_(addr, Imm(0xff));
				x86::Gp xw = cc.new_gp32(); cc.mov(xw, x86::dword_ptr(dev, addr, 2, off_dmem0)); cc.mov(x86::dword_ptr(dev, off_xwr), xw);   // xwr = dmem0[..]
			}
			x86::Gp v = cc.new_gp32();
			if (by_ca) { x86::Gp cai = cc.new_gpz(); cc.movzx(cai, x86::byte_ptr(dev, off_ca)); cc.mov(v, x86::dword_ptr(dev, cai, 2, off_cmem)); }
			else       cc.mov(v, x86::dword_ptr(dev, off_cmem + int(param) * 4));
			cc.mov(x86::dword_ptr(dev, off_xoa), v);   // xoa = get_cmem(ca|param)
			{ InvokeNode *xi = nullptr; cc.invoke(Out(xi), uint64_t(&jit_w_xm_init), FuncSignature::build<void, tms57002_device *>()); xi->set_arg(0, dev); }   // xm_init() (no macc touch)
			cc.or_(x86::dword_ptr(dev, off_sti), Imm(flag));   // sti |= S_READ / S_WRITE
			cc.jmp(done);
			cc.bind(deopt);
			macc_out();
			{ InvokeNode *mr = nullptr; cc.invoke(Out(mr), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>()); mr->set_arg(0, dev); mr->set_arg(1, Imm(op)); mr->set_arg(2, Imm(icd_addr)); }
			macc_in();
			cc.bind(skip);
			cc.bind(done);
		}
		else
		{
			macc_out();   // un-inlined op may read/write macc -> sync around the call
			InvokeNode *m = nullptr;
			cc.invoke(Out(m), uint64_t(&jit_w_op), FuncSignature::build<void, tms57002_device *, unsigned, const void *>());
			m->set_arg(0, dev);
			m->set_arg(1, Imm(op));
			m->set_arg(2, Imm(icd_addr));
			macc_in();
			x86::Gp sti = cc.new_gp32("sti");
			cc.mov(sti, x86::dword_ptr(dev, off_sti));
			cc.test(sti, Imm(idle_mask));
			cc.jnz(chain_end);
		}
		cur = next;
	}
	cc.bind(chain_end);
	ok = true;
	return chain_ipc;
}

// Register-cached post (x86): pc + icount live in regs across the frame. Fast path icount--/pc++ as
// register ops; deopt to jit_w_post (sync regs, call, reload) on pending-transfer/rptc/rptc_next/
// S_BRANCH/pc0-transition or when the serial model is on. Returns the next-ipc reg.
static asmjit::x86::Gp emit_post_regcached(asmjit::x86::Compiler &cc, asmjit::x86::Gp dev, tms57002_device &dsp,
	int iipc, asmjit::x86::Gp chain_ipc, asmjit::x86::Gp reg_pc, asmjit::x86::Gp reg_icount)
{
	using namespace asmjit;
	x86::Gp next = cc.new_gp32("next");
	const int off_pc = int(tms57002_device::jit_off_pc());
	const int off_ic = int(tms57002_device::jit_off_icount());

	auto sync_call_reload = [&]() {
		cc.mov(x86::byte_ptr(dev, off_pc), reg_pc.r8());     // sync cached pc/icount for the C++ post
		cc.mov(x86::dword_ptr(dev, off_ic), reg_icount);
		x86::Gp r = emit_post_call(cc, dev, iipc, chain_ipc);
		cc.mov(next, r);
		cc.movzx(reg_pc, x86::byte_ptr(dev, off_pc));         // reload what the C++ post changed
		cc.mov(reg_icount, x86::dword_ptr(dev, off_ic));
	};

	if (dsp.jit_serial_model()) { sync_call_reload(); return next; }

	const int off_pend = int(tms57002_device::jit_off_pending_pre_transfer());
	const int off_rptc = int(tms57002_device::jit_off_rptc());
	const int off_rptcn = int(tms57002_device::jit_off_rptc_next());
	const int off_sti = int(tms57002_device::jit_off_sti());
	const u32 branch_mask = tms57002_device::jit_s_branch_mask();
	Label slow = cc.new_label(), done2 = cc.new_label();

	{ x86::Gp t = cc.new_gp32(); cc.movzx(t, x86::byte_ptr(dev, off_pend)); cc.test(t, t); cc.jnz(slow); }
	{ x86::Gp t = cc.new_gp32(); cc.movzx(t, x86::byte_ptr(dev, off_rptc)); cc.test(t, t); cc.jnz(slow); }
	{ x86::Gp t = cc.new_gp32(); cc.movzx(t, x86::byte_ptr(dev, off_rptcn)); cc.test(t, t); cc.jnz(slow); }
	{ x86::Gp t = cc.new_gp32(); cc.mov(t, x86::dword_ptr(dev, off_sti)); cc.test(t, Imm(branch_mask)); cc.jnz(slow); }
	cc.test(reg_pc, reg_pc); cc.jz(slow);          // pc==0  -> pc0 transition
	cc.cmp(reg_pc, Imm(255)); cc.je(slow);         // pc==255 -> pc0 transition

	cc.sub(reg_icount, Imm(1));   // FAST PATH, registers only
	cc.add(reg_pc, Imm(1));
	cc.mov(next, chain_ipc);
	cc.jmp(done2);

	cc.bind(slow);
	sync_call_reload();
	cc.bind(done2);
	return next;
}

Jit::PcFn Jit::compile_pc(tms57002_device &, int) { return nullptr; }   // x86-64: per-PC path not used (compile_frame is)

#else   // neither arch
Jit::PcFn Jit::compile_pc(tms57002_device &, int) { return nullptr; }
#endif

Jit::PcFn Jit::get_pc_fn(tms57002_device &dsp, int ipc)
{
	if (ipc < 0)
		return nullptr;
	if (size_t(ipc) >= m_pc_fns.size())
		m_pc_fns.resize(size_t(ipc) + 1, nullptr);
	if (m_pc_fns[ipc])
		return m_pc_fns[ipc];   // O(1) flat lookup, no hash
	PcFn fn = compile_pc(dsp, ipc);
	if (fn)
		m_pc_fns[ipc] = fn;     // only cache success; a failed compile sets m_pc_native_failed in the caller
	return fn;
}

std::array<u32, 4> Jit::run_sample_native(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps)
{
	if (m_pc_native_failed)
		return run_sample(dsp, in, max_steps);
	dsp.jit_begin_frame(in);
	dsp.jit_set_icount(max_steps);
	int ipc = -1;
	int guard = (max_steps > 0 ? max_steps : 1) + 16;
	while (dsp.jit_running() && guard-- > 0)
	{
		if (ipc == -1)
			ipc = dsp.jit_decode_current();
		PcFn fn = get_pc_fn(dsp, ipc);
		if (!fn)
		{
			m_pc_native_failed = true;   // codegen unavailable -> interpreter from here on
			while (dsp.jit_running() && guard-- > 0)
			{
				if (ipc == -1)
					ipc = dsp.jit_decode_current();
				ipc = dsp.jit_run_chain(ipc);
			}
			break;
		}
		ipc = fn(&dsp);
	}
	dsp.jit_finalize_if_idle();
	return dsp.jit_end_frame();
}

// Whole-frame compile: ONE native void(device) function, every PC's body inlined (no per-PC
// calls). The linear PC sequence (pc++) is unrolled at compile time by decoding each pc; after
// each PC a safety check (runtime next-ipc == compile-time-expected linear next) bails to the
// interpreter for the REST of the frame on any branch/repeat. Idle ends the frame early. If the
// program is linear (DSP1), the whole frame runs with zero per-PC call overhead.
Jit::FrameFn Jit::compile_frame(tms57002_device &dsp, int max_steps, int start_pc)
{
#ifdef KPROP_JIT_A64
	using namespace asmjit;

	CodeHolder code;
	if (code.init(m_rt.environment()) != kErrorOk)
		return nullptr;
	a64::Compiler cc(&code);
	FuncNode *fn = cc.add_func(FuncSignature::build<void, tms57002_device *>());
	if (!fn)
		return nullptr;

	a64::Gp dev = cc.new_gpz("dev");
	fn->set_arg(0, dev);

	const int off_sti = int(tms57002_device::jit_off_sti());
	const unsigned idle_bit = __builtin_ctz(tms57002_device::jit_s_idle_mask());

	Label done = cc.new_label();
	Label fallback = cc.new_label();
	a64::Gp fb_ipc = cc.new_gp32("fb_ipc");

	const int nsteps = int(m_frame_trace.size());
	if (nsteps == 0)
		return nullptr;   // no trace recorded -> fall back to the per-PC interpreter
	(void)max_steps; (void)start_pc;
	for (int k = 0; k < nsteps; k++)
	{
		const int ipc_k = m_frame_trace[k];   // chain head recorded at this PC's RUNTIME st1 (not frame-start)

		bool ok = true;
		a64::Gp chain_ipc = emit_pc_body(cc, dev, dsp, ipc_k, ok);   // PC body INLINE (no call)
		if (!ok)
			return nullptr;
		a64::Gp next = emit_post_call(cc, dev, ipc_k, chain_ipc);   // post (memory-based for now)

		// Idle ends the frame (matches the interpreter's top-of-loop !(sti & S_IDLE) test).
		{
			a64::Gp sti = cc.new_gp32("sti");
			cc.ldr(sti, a64::ptr(dev, off_sti));
			cc.tbnz(sti, Imm(idle_bit), done);
		}

		// Safety: next ipc must be the compile-time-expected linear next; else the program
		// branched/repeated -> finish the rest of the frame via the interpreter from the real ipc.
		if (k < nsteps - 1)
		{   // chain-consistent next (walk ipc_k's chain to its terminator's next), matching the x86 path
			int expected = ipc_k;
			for (int s = 0; s <= 64; s++) {
				const unsigned cop = dsp.jit_inst_op(expected);
				const int nx = dsp.jit_inst_next(expected);
				expected = nx;
				if (cop < 4) break;
			}
			cc.mov(fb_ipc, next);
			a64::Gp tmp = cc.new_gp32("tmp");
			cc.sub(tmp, next, Imm(expected));
			cc.cbnz(tmp, fallback);
		}
	}
	cc.b(done);

	cc.bind(fallback);
	{
		InvokeNode *r = nullptr;
		cc.invoke(Out(r), uint64_t(&jit_w_run_rest), FuncSignature::build<void, tms57002_device *, int>());
		r->set_arg(0, dev);
		r->set_arg(1, fb_ipc);
	}
	// fall through to done

	cc.bind(done);
	cc.ret();
	cc.end_func();
	if (cc.finalize() != kErrorOk)
		return nullptr;

	FrameFn out = nullptr;
	if (m_rt.add(&out, &code) != kErrorOk)
		return nullptr;
	return out;

#elif defined(KPROP_JIT_X64)
	using namespace asmjit;
	CodeHolder code;
	if (code.init(m_rt.environment()) != kErrorOk)
		return nullptr;
	x86::Compiler cc(&code);
	FuncNode *fn = cc.add_func(FuncSignature::build<void, tms57002_device *>());
	if (!fn)
		return nullptr;
	x86::Gp dev = cc.new_gpz("dev");
	fn->set_arg(0, dev);
	const int off_sti = int(tms57002_device::jit_off_sti());
	const u32 idle_mask = tms57002_device::jit_s_idle_mask();
	const int off_pc = int(tms57002_device::jit_off_pc());
	const int off_ic = int(tms57002_device::jit_off_icount());
	Label done = cc.new_label();
	Label fallback = cc.new_label();
	x86::Gp fb_ipc = cc.new_gp32("fb_ipc");
	// Register-cache pc + icount across the frame (ops never touch them -> no sync around jit_w_op).
	x86::Gp reg_pc = cc.new_gp32("reg_pc");
	x86::Gp reg_icount = cc.new_gp32("reg_icount");
	cc.movzx(reg_pc, x86::byte_ptr(dev, off_pc));
	cc.mov(reg_icount, x86::dword_ptr(dev, off_ic));
	// Register-cache the MACC pipeline across the frame (emit_pc_body syncs around jit_w_op/jit_w_xm).
	const int off_macc = int(tms57002_device::jit_off_macc());
	const int off_macc_r = int(tms57002_device::jit_off_macc_read());
	const int off_macc_w = int(tms57002_device::jit_off_macc_write());
	x86::Gp reg_macc = cc.new_gpz("reg_macc"), reg_macc_r = cc.new_gpz("reg_macc_r"), reg_macc_w = cc.new_gpz("reg_macc_w");
	cc.mov(reg_macc, x86::qword_ptr(dev, off_macc));
	cc.mov(reg_macc_r, x86::qword_ptr(dev, off_macc_r));
	cc.mov(reg_macc_w, x86::qword_ptr(dev, off_macc_w));

	const int nsteps = int(m_frame_trace.size());
	if (nsteps == 0)
		return nullptr;   // no trace recorded -> fall back to the per-PC interpreter
	(void)max_steps; (void)start_pc;
	for (int k = 0; k < nsteps; k++)
	{
		const int ipc_k = m_frame_trace[k];   // chain head recorded at this PC's RUNTIME st1 (not frame-start)
		bool ok = true;
		x86::Gp chain_ipc = emit_pc_body(cc, dev, dsp, ipc_k, ok, reg_macc, reg_macc_r, reg_macc_w);
		if (!ok)
			return nullptr;
		x86::Gp next = emit_post_regcached(cc, dev, dsp, ipc_k, chain_ipc, reg_pc, reg_icount);

		{   // idle ends the frame
			x86::Gp sti = cc.new_gp32("sti");
			cc.mov(sti, x86::dword_ptr(dev, off_sti));
			cc.test(sti, Imm(idle_mask));
			cc.jnz(done);
		}
		if (k < nsteps - 1)
		{   // safety: the runtime next-ipc must match what pc_k's compiled chain LINKS to (its
			// chain-consistent next), NOT a fresh jit_decode(pc_next). The (pc,st1)-keyed decode
			// cache hands back inconsistent cache indices for diverse programs, which falsely
			// tripped this guard for real firmware (chain next=5 vs jit_decode=591) and forced the
			// whole frame to interpret. The compiled tail falls through to pc_{k+1}'s body either
			// way; the guard only needs to catch a runtime post (deopt) diverging from the linear
			// chain (a genuine branch/repeat), where next != the chain's terminator-next.
			int expected = ipc_k;
			for (int s = 0; s <= 64; s++) {
				const unsigned cop = dsp.jit_inst_op(expected);
				const int nx = dsp.jit_inst_next(expected);
				expected = nx;
				if (cop < 4) break;   // chain terminator -> expected = its next = the chain-consistent next
			}
			cc.mov(fb_ipc, next);
			cc.cmp(next, Imm(expected));
			cc.jne(fallback);
		}
	}
	cc.jmp(done);

	cc.bind(fallback);
	cc.mov(x86::byte_ptr(dev, off_pc), reg_pc.r8());     // sync cached state before the interpreter finishes
	cc.mov(x86::dword_ptr(dev, off_ic), reg_icount);
	cc.mov(x86::qword_ptr(dev, off_macc), reg_macc);
	cc.mov(x86::qword_ptr(dev, off_macc_r), reg_macc_r);
	cc.mov(x86::qword_ptr(dev, off_macc_w), reg_macc_w);
	{
		InvokeNode *r = nullptr;
		cc.invoke(Out(r), uint64_t(&jit_w_run_rest), FuncSignature::build<void, tms57002_device *, int>());
		r->set_arg(0, dev);
		r->set_arg(1, fb_ipc);
	}
	cc.movzx(reg_pc, x86::byte_ptr(dev, off_pc));         // reload after the interpreter advanced them
	cc.mov(reg_icount, x86::dword_ptr(dev, off_ic));
	cc.mov(reg_macc, x86::qword_ptr(dev, off_macc));
	cc.mov(reg_macc_r, x86::qword_ptr(dev, off_macc_r));
	cc.mov(reg_macc_w, x86::qword_ptr(dev, off_macc_w));

	cc.bind(done);
	cc.mov(x86::byte_ptr(dev, off_pc), reg_pc.r8());      // write cached state back for finalize/end_frame
	cc.mov(x86::dword_ptr(dev, off_ic), reg_icount);
	cc.mov(x86::qword_ptr(dev, off_macc), reg_macc);
	cc.mov(x86::qword_ptr(dev, off_macc_r), reg_macc_r);
	cc.mov(x86::qword_ptr(dev, off_macc_w), reg_macc_w);
	cc.ret();
	cc.end_func();
	if (cc.finalize() != kErrorOk)
		return nullptr;
	// M2 sizing: the UNROLLED resident frame's code size (x3 vs the 32KB L1I decides if fusion needs
	// a shared op-body pool). One-time per distinct trace length.
	{ static size_t last = 0; if (code.code_size() != last) { last = code.code_size(); std::fprintf(stderr, "[frame-size] unrolled resident frame: %zu bytes (%d trace steps) -> x3 fused = %zu bytes vs 32KB L1I\n", code.code_size(), nsteps, code.code_size() * 3); } }
	FrameFn out = nullptr;
	if (m_rt.add(&out, &code) != kErrorOk)
		return nullptr;
	return out;

#else
	return nullptr;
#endif
}

std::array<u32, 4> Jit::run_sample_frame_native(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps)
{
	if (m_frame_failed)
		return run_sample_native(dsp, in, max_steps);   // before begin_frame (no double-begin)

	dsp.jit_begin_frame(in);
	dsp.jit_set_icount(max_steps);
	const int ms = max_steps > 0 ? max_steps : 1;
	const int start_pc = dsp.jit_pc();
	// Key the compiled frame by the starting st1 CACHE bits too: they select op variants and evolve
	// across frames (frame 0 starts at the loaded st1, later frames at the program's steady state),
	// so a trace recorded under one starting-mode is invalid under another. Recompile when they change;
	// a stable program settles after ~1 frame, so steady-state frames reuse one compiled fn.
	const u32 start_st1 = dsp.dbg_st1() & tms57002_device::jit_st1_cache();

	if (!m_frame_fn || m_frame_max_steps != ms || m_frame_start_pc != start_pc || m_frame_start_st1 != start_st1)
	{
		// RECORD the per-PC decode trace by INTERPRETING this frame: re-decode each PC at its running
		// st1 (jit_decode_current), so compile_frame compiles the variant each PC actually runs. The
		// TMS57002 decode is st1-mode-dependent and st1 evolves mid-frame; the old code decoded every
		// PC at frame-start st1 and emitted/deopted the wrong variant. This frame's output is the
		// interpreter's (correct); the compiled fn (built from the trace) runs subsequent frames.
		// Branch-free program => the decode trace is identical every frame, so recording once is valid.
		m_frame_trace.clear();
		int guard = ms + 16;
		while (dsp.jit_running() && int(m_frame_trace.size()) < ms && guard-- > 0)
		{
			const int ipc = dsp.jit_decode_current();
			m_frame_trace.push_back(ipc);
			dsp.jit_run_chain(ipc);
		}
		dsp.jit_finalize_if_idle();
		const auto result = dsp.jit_end_frame();   // this frame ran (interpreted) while recording

		m_frame_fn = compile_frame(dsp, ms, start_pc);   // compiles from m_frame_trace
		m_frame_max_steps = ms;
		m_frame_start_pc = start_pc;
		m_frame_start_st1 = start_st1;
		if (!m_frame_fn)
			m_frame_failed = true;   // codegen unavailable -> per-PC interpreter from now on
		return result;
	}
	m_frame_fn(&dsp);
	dsp.jit_finalize_if_idle();
	return dsp.jit_end_frame();
}

// M2: the pooled+pinned whole-frame emit. Hand-written x86-64 (asmjit Assembler) with a private
// calling convention — macc/macc_read/macc_write/aacc PINNED in callee-saved regs (r12/r13/r14/r15)
// and dev in rbx — so C-ABI helper calls (jit_w_xm/jit_w_post) preserve them automatically and only
// a DEOPT (jit_w_op, which mutates device macc/aacc in memory) needs a spill/reload. THIS skeleton
// deopts EVERY op (proving the convention + machinery + post + fallback bit-exact); the next step
// replaces the deopt with CALLs into a shared op-body pool (the residency + fusion-viability win).
// M3: build the program-order (pc, chain-head ipc) list the pooled frame compiles from — EVERY PC from
// start to the program's idle, in program order, each decoded at its DETERMINISTIC st1 mode. The st1
// mode per PC is path-independent (the st1 de-risk proved no branch skips a mode-setter). See the body
// comment for the mechanism (a decode-only, chaining-disabled walk; the decoder itself evolves st1).
void Jit::build_pooled_order(tms57002_device &dsp, int start_pc, int max_steps)
{
	// Program-order DECODE-ONLY walk. Key fact: the TMS57002 decoder mutates st1's mode-select fields
	// (sfmo/sfai/sfao/sfma/scrm/rnd/rmom/smom/ldpk) AT DECODE TIME (the CDEC tables), and those mode
	// fields are the ONLY st1 bits that pick op variants (ST1_CACHE) — verified: no execution-only op
	// writes a decode-key bit. So decoding each PC in program order, with decode-CHAINING DISABLED (one
	// PC per decode_get_pc call), evolves st1 exactly as the interpreter's decoder does and yields the
	// correct variant for every PC at its deterministic runtime st1 — with ZERO op execution (no data /
	// XRAM / serial side effects, so nothing to corrupt). This replaces the old flag-setter replay, whose
	// bug was calling decode_get_pc with chaining ON: that decodes a whole RUN of PCs at once and
	// over-advances st1 past the current PC (e.g. decoding a downstream smhd at sfmo=2 instead of 0).
	// Branches are walked as fall-through (linear p++); the st1 de-risk proved no branch skips a
	// mode-setter, so program order visits every reachable PC at the correct (path-independent) st1.
	// Non-destructive: the snapshot restores st1/pc; the decode-cache entries built here are reused (as
	// cache hits) by the interpret that runs next, so the recorded (pc,ipc) stay valid for compile.
	m_pooled_order.clear();
	m_pooled_order_mode_safe = true;
	const auto snap = dsp.debug_capture_snapshot();
	const bool save_chain = dsp.jit_disable_chaining();
	dsp.jit_set_disable_chaining(true);
	dsp.jit_cache_flush();
	int p = start_pc & 0xff;
	const int lim = (max_steps > 0 ? max_steps : 1);
	// Mode-safety bookkeeping: the fall-through walk is only valid if no branch skips a mode-setter
	// (st1-per-PC path-independence). Record the st1 CACHE state on arrival at each order index and
	// every branch edge (order index -> target pc); verified below. Sufficient condition: for every
	// branch k -> target j, mode_after_decode(k) == mode_at_entry(j) — then any executed path (linear
	// segments joined by taken branches) evolves st1 exactly as this linear walk did, by induction
	// over segments. A violation means the compiled variants could be wrong for some path -> the
	// program is marked mode-unsafe and compile_frame_pooled refuses it (runs via interpreter).
	std::vector<u32> mode_at_entry;
	std::vector<std::pair<int, int>> branch_edges;   // (order index of the branch, target pc)
	const u32 st1_cache_mask = tms57002_device::jit_st1_cache();
	for (int steps = 0; steps < lim && steps < 256; steps++)
	{
		dsp.jit_set_pc(u8(p));
		mode_at_entry.push_back(dsp.dbg_st1() & st1_cache_mask);
		const int ipc = dsp.jit_decode_current();   // decodes ONLY p (chaining off); mutates st1 by p's mode-setter
		m_pooled_order.push_back({ p, ipc });
		bool idle = false;
		int cur = ipc;
		for (int s = 0; s <= 64; s++)
		{
			const unsigned op = dsp.jit_inst_op(cur);
			if (op >= 4)
			{
				const char *mn = dsp.jit_op_mnemonic(op);
				if (mn && __builtin_strcmp(mn, "idle") == 0) idle = true;
				if (mn && (__builtin_strcmp(mn, "b") == 0 || __builtin_strcmp(mn, "bgz") == 0
					|| __builtin_strcmp(mn, "blz") == 0 || __builtin_strcmp(mn, "bnz") == 0
					|| __builtin_strcmp(mn, "bv") == 0))
					branch_edges.emplace_back(int(m_pooled_order.size()) - 1, int(dsp.jit_inst_param(cur)) & 0xff);
			}
			if (op < 4) break;
			cur = dsp.jit_inst_next(cur);
		}
		if (idle) break;
		p = (p + 1) & 0xff;
	}
	const u32 mode_final = dsp.dbg_st1() & st1_cache_mask;   // state after the LAST pc's decode
	dsp.jit_set_disable_chaining(save_chain);
	dsp.debug_restore_snapshot(snap);

	const int n = int(m_pooled_order.size());
	// Branch-target order indices, for the F5-refinement window-overlap guard below.
	std::vector<bool> is_btarget(size_t(n ? n : 1), false);
	for (const auto &e : branch_edges)
	{
		const int tj = (e.second - (start_pc & 0xff)) & 0xff;
		if (tj < n) is_btarget[size_t(tj)] = true;
	}
	for (const auto &e : branch_edges)
	{
		const int k = e.first;
		const int tj = (e.second - (start_pc & 0xff)) & 0xff;   // target's order index (order is linear from start_pc)
		const u32 after_k = (k + 1 < n) ? mode_at_entry[k + 1] : mode_final;
		if (tj >= n)
		{
			m_pooled_order_mode_safe = false;   // target outside the order (compile would fail anyway)
			break;
		}
		const u32 diff0 = mode_at_entry[tj] ^ after_k;
		if (diff0 == 0)
			continue;   // arrival mode states match exactly — the common case
		// F5 REFINEMENT (2026-07-07; spec = 07-05 audit note §4). The branch path arrives at the
		// target with mode state differing from the fall-through walk's by diff0. That is harmful
		// ONLY if some op decoded between the join and the reconvergence point actually CONSUMED a
		// differing field — i.e. its (mnemonic, id) family enumerates that field, so the interpreter
		// would have decoded a DIFFERENT op-id on the branch path than the one this walk baked.
		// Walk forward from the target, shrinking `diff` at each mode-setter: both paths execute the
		// SAME setter op-id (its own consumption is checked first), writing the same immediate, so
		// the fields it rewrites become path-independent. Refuse (conservatively) on: a consumer of
		// a differing field, a branch originating or landing inside the unreconverged window, or a
		// diff that survives to the end of the program (the next frame's start_st1 cache key would
		// become path-dependent — the re-prime-storm class from the F1 post-mortem).
		// NOTE 2026-07-05: the korgprop boot-window program branches pc 18 -> 22 with diff=ST1_SFMO;
		// whether it is admitted now depends on whether any %mo/%mn/%mv consumer sits before the
		// next sfmo write — decided here per edge instead of refusing the whole program.
		const char *refuse = nullptr;
		int reconv_pc = -1;
		u32 diff = diff0;
		for (int i = tj; i < n && diff && !refuse; i++)
		{
			if (i != tj && is_btarget[size_t(i)])
			{
				refuse = "another branch lands in the window";
				break;
			}
			int cur = m_pooled_order[i].second;
			for (int s = 0; s <= 64 && diff; s++)
			{
				const unsigned op = dsp.jit_inst_op(cur);
				if (op < 4)
					break;
				const char *mn = dsp.jit_op_mnemonic(op);
				if (!mn) { refuse = "unknown op in the window"; break; }
				if (__builtin_strcmp(mn, "b") == 0 || __builtin_strcmp(mn, "bgz") == 0
					|| __builtin_strcmp(mn, "blz") == 0 || __builtin_strcmp(mn, "bnz") == 0
					|| __builtin_strcmp(mn, "bv") == 0)
				{ refuse = "a branch originates in the window"; break; }
				if (tms57002::op_mode_consumed_mask(op) & diff)
				{ refuse = "an op variant consumes a differing mode field"; break; }
				if (tms57002::op_is_mode_setter(op))
				{
					const void *icd = reinterpret_cast<const void *>(dsp.jit_inst_addr(cur));
					const u32 v0 = dsp.jit_probe_st1(op, icd, 0x00000000u);
					const u32 vF = dsp.jit_probe_st1(op, icd, 0xffffffffu);
					diff &= vF & ~v0;   // keep-mask: the setter's fields are now path-independent
				}
				cur = dsp.jit_inst_next(cur);
			}
			if (!diff) reconv_pc = m_pooled_order[i].first;
		}
		if (diff && !refuse)
			refuse = "the mode diff survives to the end of the program";
		// Log once per (program, edge, outcome): admitted programs get RE-PRIMED per program switch
		// and this analysis reruns each time — don't spam stderr.
		static std::set<u64> s_logged;
		const u64 log_key = m_cur_prog_hash ^ (u64(m_pooled_order[k].first) << 16)
			^ (u64(e.second) << 8) ^ (refuse ? 1u : 0u);
		if (!refuse)
		{
			if (s_logged.insert(log_key).second)
				std::fprintf(stderr, "[pooled] F5: branch pc %02x -> pc %02x mode diff %06x is BENIGN (no consumer; reconverges by pc %02x) -> pooled (prog %016llx)\n",
					m_pooled_order[k].first, e.second, diff0, reconv_pc, (unsigned long long)m_cur_prog_hash);
			continue;
		}
		m_pooled_order_mode_safe = false;
		if (s_logged.insert(log_key).second)
			std::fprintf(stderr, "[pooled] mode-setter/branch conflict (pc %02x -> pc %02x, prog %016llx, st1cache branch=%06x vs walk=%06x): %s; program runs via interpreter (KPROP_PF4_MODE_UNSAFE_COMPILE=1 overrides)\n",
				m_pooled_order[k].first, e.second, (unsigned long long)m_cur_prog_hash,
				after_k, mode_at_entry[tj], refuse);
		break;
	}
}

#if defined(KPROP_JIT_X64) || defined(KPROP_JIT_A64)
namespace {
using namespace asmjit;

// The pooled path's concrete assembler for this host (2026-07-07: dual-backend; the a64 twins of
// emit_thunks/emit_pooled_driver/emit_pool_bodies live below the x86 ones).
#ifdef KPROP_JIT_X64
using PooledAssembler = x86::Assembler;
#else
using PooledAssembler = a64::Assembler;
#endif

// One shared op-body pool entry: the body's label + a representative instruction descriptor and the
// device that owns it (emit_op_pooled probes st1 / reads the descriptor at emit time; the emitted code
// is DEVICE-INDEPENDENT — it operates on the pinned DEV register at runtime — so one body per op-id
// serves every program that uses that op-id).
struct PoolBody { Label label; uint64_t icd; tms57002_device *owner; };

// Records the first emit error. asmjit returns a per-call Error that this code historically ignored;
// without a handler a failed encoding SILENTLY DROPS the instruction (2026-07-07 a64 lesson: device
// offsets > imm12 range dropped the pc load, and the frame dispatched on a stale register). Attached
// to the CodeHolder in compile_frame_pooled/compile_fused; any error fails the WHOLE compile, which
// deopts that program to the interpreter — slow but never wrong.
struct PooledErrorSink : ErrorHandler {
	Error err = Error::kOk;
	void handle_error(Error e, const char *message, BaseEmitter *) override {
		if (err == Error::kOk) err = e;
		std::fprintf(stderr, "[pooled] asmjit emit error: %s (frame compile aborted -> interpreter)\n", message ? message : "?");
	}
};

// idea ③ (driver-size reduction): rel32 CALL THUNKS. Each C-ABI helper (jit_w_xm/op/post/run_rest) is
// reached once at a shared in-region thunk (`mov rax, imm64; jmp rax`) that the drivers `call` via a
// 5-byte rel32 — vs a 12-byte `mov rax, imm64; call rax` at every call site. Same behavior (the thunk
// tail-jumps, the helper returns straight to the driver), ~7 bytes saved per call site × ~2 calls/PC ×
// ~737 PCs ≈ 10KB smaller driver -> better L1I retention when fused. Thunks are shared across all 3
// fused drivers. Allocated by the caller (labels), emitted once by emit_thunks after the drivers.
struct Thunks { Label xm, op, post, rest, forced_fallback; };

#ifdef KPROP_JIT_X64
void emit_thunks(x86::Assembler &a, const Thunks &t)
{
	auto thunk = [&](Label lbl, void *fn) { a.bind(lbl); a.mov(x86::rax, Imm(uint64_t(fn))); a.jmp(x86::rax); };
	thunk(t.xm, reinterpret_cast<void *>(&jit_w_xm));
	thunk(t.op, reinterpret_cast<void *>(&jit_w_op));
	thunk(t.post, reinterpret_cast<void *>(&jit_w_post));
	thunk(t.rest, reinterpret_cast<void *>(&jit_w_run_rest));
	thunk(t.forced_fallback, reinterpret_cast<void *>(&jit_w_forced_midframe_fallback_hit));
}
#endif  // KPROP_JIT_X64

// Ensure `pool` has a body (label allocated via `a`) for every poolable op-id `order` uses on `dsp`.
// Shared across the fused drivers (called once per program) so each op-id body appears exactly once.
// (BaseEmitter: arch-independent — only allocates labels.)
void scan_pool(BaseEmitter &a, tms57002_device &dsp,
               const std::vector<std::pair<int, int>> &order, std::map<unsigned, PoolBody> &pool)
{
	for (const auto &e : order)
	{
		int cur = e.second;
		for (int s = 0; s <= 64; s++)
		{
			const unsigned op = dsp.jit_inst_op(cur);
			if (op < 4) break;
			if (tms57002::op_poolable(dsp, op) && !pool.count(op))
				pool.emplace(op, PoolBody{ a.new_label(), dsp.jit_inst_addr(cur), &dsp });
			cur = dsp.jit_inst_next(cur);
		}
	}
}

bool order_reads_cmem(tms57002_device &dsp, const std::vector<std::pair<int, int>> &order)
{
	for (const auto &entry : order)
	{
		int cur = entry.second;
		for (int steps = 0; steps <= 64; ++steps)
		{
			const unsigned op = dsp.jit_inst_op(cur);
			if (op < 4)
				break;
			if (tms57002::op_reads_cmem(dsp, op))
				return true;
			cur = dsp.jit_inst_next(cur);
		}
	}
	return false;
}

#ifdef KPROP_JIT_X64
// Emit ONE program's pooled driver (prologue..ret) into `a`, calling into the (already-scanned) shared
// `pool`. Bind the caller's entry label BEFORE calling this — the driver's first instruction is the
// prologue. Returns false on unsupported shape (caller bails). Pool bodies are emitted SEPARATELY (once).
bool emit_pooled_driver(x86::Assembler &a, tms57002_device &dsp,
                        const std::vector<std::pair<int, int>> &order, const std::map<unsigned, PoolBody> &pool,
                        const Thunks &thunks, bool cmem_per_op)
{
	const int off_macc = int(tms57002_device::jit_off_macc());
	const int off_macc_r = int(tms57002_device::jit_off_macc_read());
	const int off_macc_w = int(tms57002_device::jit_off_macc_write());
	const int off_aacc = int(tms57002_device::jit_off_aacc());
	const int off_sti = int(tms57002_device::jit_off_sti());
	const int off_ca = int(tms57002_device::jit_off_ca());
	const int off_id = int(tms57002_device::jit_off_id());
	const int off_pc = int(tms57002_device::jit_off_pc());
	const int off_ic = int(tms57002_device::jit_off_icount());
	const int off_pend = int(tms57002_device::jit_off_pending_pre_transfer());
	const int off_rptc = int(tms57002_device::jit_off_rptc());
	const int off_rptcn = int(tms57002_device::jit_off_rptc_next());
	const u32 idle_mask = tms57002_device::jit_s_idle_mask();
	const u32 rw_mask = tms57002_device::jit_s_read_mask() | tms57002_device::jit_s_write_mask();
	const x86::Gp DEV = x86::rbx, MACC = x86::r12, MACR = x86::r13, MACW = x86::r14, AACC = x86::r15, PARAM = x86::r11;
	// Keep the handwritten assembler's C++ calls ABI-correct on both x64 ABIs.
	// Windows uses RCX/RDX/R8 and requires 32 bytes of caller-provided shadow
	// space; System V uses RDI/RSI/RDX and has no shadow-space requirement.
#if defined(_WIN32)
	const x86::Gp ARG0 = x86::rcx, ARG1 = x86::rdx, ARG2 = x86::r8;
#else
	const x86::Gp ARG0 = x86::rdi, ARG1 = x86::rsi, ARG2 = x86::rdx;
#endif

	const int nsteps = int(order.size());
	if (nsteps == 0)
		return false;

	// [KPROP_PF4_FORCE_MIDFRAME_FALLBACK=<pc>] F3 test knob (2026-07-05): at that one PC, force the
	// mid-frame fallback path (spill -> jit_w_run_rest -> interpreter finishes the frame under
	// jit_running()'s icount/idle contract). Proves the in-frame fallback is byte-exact in the live
	// machine — it is otherwise ~never exercised (frame-level fallbacks dominate the stats).
	// KPSHIP-ENGINE: JIT mid-frame-fallback forcing knob (must stay byte-identical). Bake safe default, delete.
	static const int force_fb_pc = [] {
		const char *e = std::getenv("KPROP_PF4_FORCE_MIDFRAME_FALLBACK");
		return (e && *e) ? int(std::strtol(e, nullptr, 0)) : -1;
	}();

	// Fused slow-post trigger test: (m_pending_pre_transfer | rptc | rptc_next) are three adjacent u8s
	// (tms57002.h declares them together for exactly this), so the per-instruction test is ONE dword
	// load + mask instead of three byte loads. Adjacency is verified here at emit time; if the layout
	// ever changes the driver falls back to the three-load form (slower, never wrong).
	int off_flags_lo = off_rptc < off_rptcn ? off_rptc : off_rptcn;
	if (off_pend < off_flags_lo) off_flags_lo = off_pend;
	int off_flags_hi = off_rptc > off_rptcn ? off_rptc : off_rptcn;
	if (off_pend > off_flags_hi) off_flags_hi = off_pend;
	const bool fused_flags = (off_flags_hi - off_flags_lo == 2)
		&& off_rptc != off_rptcn && off_rptc != off_pend && off_rptcn != off_pend;

	// Does this program arm an XRAM transaction anywhere? rde/wre are the ONLY S_READ/S_WRITE setters
	// in the whole op set (verified in the generated .hxx; nothing in the core arms them either). If
	// the program has none, the per-PC pc_pre xm test is dead weight: hoist it to ONE check at frame
	// entry (armed there = a previous program's transaction still in flight across a PLOAD -> let the
	// interpreter run this frame; it xm_steps per instruction until the transaction drains).
	bool program_has_xm = false;
	bool program_reads_cmem = false;
	for (const auto &e : order)
	{
		int cur = e.second;
		for (int s = 0; s <= 64; s++)
		{
			const unsigned op = dsp.jit_inst_op(cur);
			if (op < 4) break;
			program_reads_cmem |= tms57002::op_reads_cmem(dsp, op);
			const char *mn = dsp.jit_op_mnemonic(op);
			if (mn && (__builtin_strcmp(mn, "rde") == 0 || __builtin_strcmp(mn, "wre") == 0))
			{
				program_has_xm = true;
				break;
			}
			cur = dsp.jit_inst_next(cur);
		}
	}
	const bool hoist_cmem_guard = dsp.jit_pf4_cmem_deopt() && program_reads_cmem
		&& !cmem_per_op;

	// A label per emitted PC (branch targets reference these). Per-driver (not shared).
	std::map<int, Label> pc_label;
	for (int k = 0; k < nsteps; k++)
		pc_label.emplace(order[k].first, a.new_label());
	Label done = a.new_label(), fallback = a.new_label();
	auto is_branch_mn = [](const char *mn) { return mn && (__builtin_strcmp(mn, "bgz") == 0 || __builtin_strcmp(mn, "blz") == 0 || __builtin_strcmp(mn, "bnz") == 0 || __builtin_strcmp(mn, "bv") == 0 || __builtin_strcmp(mn, "b") == 0); };

	auto spill = [&]() {
		a.mov(x86::qword_ptr(DEV, off_macc), MACC);
		a.mov(x86::qword_ptr(DEV, off_macc_r), MACR);
		a.mov(x86::qword_ptr(DEV, off_macc_w), MACW);
		a.mov(x86::dword_ptr(DEV, off_aacc), AACC.r32());
	};
	auto reload = [&]() {
		a.mov(MACC, x86::qword_ptr(DEV, off_macc));
		a.mov(MACR, x86::qword_ptr(DEV, off_macc_r));
		a.mov(MACW, x86::qword_ptr(DEV, off_macc_w));
		a.mov(AACC.r32(), x86::dword_ptr(DEV, off_aacc));
	};
	// Prologue: preserve the pinned callee-saved regs (five pushes leave RSP
	// 16-byte aligned at call sites), reserve Windows shadow space, and load state.
	a.push(x86::rbx); a.push(x86::r12); a.push(x86::r13); a.push(x86::r14); a.push(x86::r15);
#if defined(_WIN32)
	a.sub(x86::rsp, Imm(32));
#endif
	a.mov(DEV, ARG0);
	reload();

	if (hoist_cmem_guard)
	{
		// MAME runs devices serially, so the V55 cannot enqueue a coefficient update while this
		// DSP frame is executing. If the queue is clear here, direct CMEM bodies remain safe for
		// the whole frame; otherwise let the interpreter drain it with exact per-read timing.
		Label safe = a.new_label();
		a.movzx(x86::eax, x86::byte_ptr(DEV, int(tms57002_device::jit_off_uc_count())));
		a.test(x86::eax, x86::eax);
		a.jz(safe);
		a.movzx(x86::eax, x86::byte_ptr(DEV, int(tms57002_device::jit_off_pf4_force())));
		a.test(x86::eax, x86::eax);
		a.jnz(safe);
		a.mov(x86::eax, Imm(-1));
		a.jmp(fallback);
		a.bind(safe);
	}

	if (!program_has_xm)
	{
		// xm hoist: no rde/wre in this program -> nothing in-frame can arm S_READ/S_WRITE, so test it
		// ONCE here instead of per PC. Armed at entry -> the interpreter finishes this frame (fallback
		// with the re-decode sentinel); the pooled frame re-engages once the transaction drains.
		Label xm_clear = a.new_label();
		a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));
		a.test(x86::ecx, Imm(rw_mask));
		a.jz(xm_clear);
		a.mov(x86::eax, Imm(-1));
		a.jmp(fallback);
		a.bind(xm_clear);
	}

	// [continuation] mid-PC entry: jump to the label for the CURRENT device pc. A fresh frame enters at
	// pc==start_pc (-> order[0]); a partial-frame continuation (the previous timeslice exited at icount<=0)
	// re-enters at the mid-frame pc. The order is linear/contiguous (order[k].first = (start_pc+k)&0xff), so
	// the table index is (pc - start_pc)&0xff. run_begun_frame_pooled guarantees pc is within this frame.
	const Label entry_table = a.new_label();
	{
		const int base_pc = order.front().first;
		a.movzx(x86::eax, x86::byte_ptr(DEV, off_pc));
		if (base_pc) a.sub(x86::eax, Imm(base_pc));
		a.and_(x86::eax, Imm(0xff));
		a.lea(x86::rcx, x86::ptr(entry_table));                     // rcx = &table (runtime addr)
		a.movsxd(x86::rax, x86::dword_ptr(x86::rcx, x86::rax, 2));   // rax = signed 32-bit (label - table)
		a.add(x86::rax, x86::rcx);                                   // rax = target label addr
		a.jmp(x86::rax);
	}

	for (int k = 0; k < nsteps; k++)
	{
		const int pc_k = order[k].first, ipc_k = order[k].second;
		a.bind(pc_label[pc_k]);
		Label chain_end = a.new_label();

		// jit_pc_pre: xm_step only if S_READ|S_WRITE (C-ABI, preserves pinned regs; no macc/aacc touch).
		// Omitted entirely for programs with no rde/wre — see the entry-time xm hoist above.
		if (program_has_xm)
		{
			Label skip = a.new_label();
			a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));
			a.test(x86::ecx, Imm(rw_mask));
			a.jz(skip);
			a.mov(ARG0, DEV);
			a.call(thunks.xm);
			a.bind(skip);
		}
		// MACC pipeline (registers): macc_read = macc_write; macc_write = macc.
		a.mov(MACR, MACW);
		a.mov(MACW, MACC);

		int cur = ipc_k, chain_ipc = dsp.jit_inst_next(cur);
		bool is_branch = false; int branch_target = -1;
		bool chain_had_deopt = false;   // any deopted op in this chain (only a deopt can set S_IDLE)
		for (int steps = 0; ; steps++)
		{
			if (steps > 64)
				return false;
			const unsigned op = dsp.jit_inst_op(cur);
			const int next = dsp.jit_inst_next(cur);
			chain_ipc = next;
			if (op >= 4 && !dsp.jit_op_mnemonic(op))
				return false;   // stale/out-of-range op -> bail; caller runs this frame via interpreter
			if (op < 4)
			{
				if (op == 1 || op == 3) { a.movzx(x86::eax, x86::byte_ptr(DEV, off_ca)); a.add(x86::eax, Imm(1)); a.mov(x86::byte_ptr(DEV, off_ca), x86::al); }
				if (op == 2 || op == 3) { a.movzx(x86::eax, x86::byte_ptr(DEV, off_id)); a.add(x86::eax, Imm(1)); a.mov(x86::byte_ptr(DEV, off_id), x86::al); }
				break;
			}
			if (is_branch_mn(dsp.jit_op_mnemonic(op))) { is_branch = true; branch_target = int(dsp.jit_inst_param(cur)); }
			auto pit = pool.find(op);
			if (pit != pool.end())
			{
				// POOL: operand in r11 (=PARAM), then call the shared body under the private convention.
				if (!hoist_cmem_guard && dsp.jit_pf4_cmem_deopt()
					&& tms57002::op_reads_cmem(dsp, op))
				{
					// CMEM-read op under KPROP_PF4_CMEM_DEOPT: deopt to the interpreter when an
					// update is pending (count != 0) and not force-unsafe; else fast body.
					const uint64_t icd = dsp.jit_inst_addr(cur);
					Label do_fast = a.new_label(), guard_done = a.new_label();
					a.movzx(x86::eax, x86::byte_ptr(DEV, int(tms57002_device::jit_off_uc_count())));
					a.test(x86::eax, x86::eax);
					a.jz(do_fast);                                   // no pending -> fast
					a.movzx(x86::eax, x86::byte_ptr(DEV, int(tms57002_device::jit_off_pf4_force())));
					a.test(x86::eax, x86::eax);
					a.jnz(do_fast);                                  // force-unsafe baseline -> fast
					spill();
					a.mov(ARG0, DEV);
					a.mov(ARG1.r32(), Imm(op));
					a.mov(ARG2, Imm(icd));
					a.call(thunks.op);
					reload();
					a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));
					a.test(x86::ecx, Imm(idle_mask));
					a.jnz(chain_end);
					a.jmp(guard_done);
					a.bind(do_fast);
					a.mov(PARAM.r32(), Imm(u32(u8(dsp.jit_inst_param(cur)))));
					a.call(pit->second.label);
					a.bind(guard_done);
				}
			else
			{
				a.mov(PARAM.r32(), Imm(u32(u8(dsp.jit_inst_param(cur)))));
				a.call(pit->second.label);
			}
			}
			else
			{
				// DEOPT: spill pinned, run the op via the interpreter, reload. (Branches run here too —
				// they set device pc=target + S_BRANCH, which the post below applies; control is native.)
				chain_had_deopt = true;
				const uint64_t icd = dsp.jit_inst_addr(cur);
				spill();
				a.mov(ARG0, DEV);
				a.mov(ARG1.r32(), Imm(op));
				a.mov(ARG2, Imm(icd));
				a.call(thunks.op);
				reload();
				a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));   // op set S_IDLE -> end chain
				a.test(x86::ecx, Imm(idle_mask));
				a.jnz(chain_end);
			}
			cur = next;
		}
		a.bind(chain_end);

		// post — inline fast path. The common case (jit_pc_post with no pending pre-transfer, no
		// repeat, no branch, no pc0/255 transition) is just `icount--; pc++`. Compile-time specialize:
		//  - branch PCs and pc_k in {0,255} always take the slow post (jit_w_post applies S_BRANCH / the
		//    pc0 callback / rptc). Non-branch pc_k in [1,254] never set S_BRANCH (only branch ops do; the
		//    prior post cleared it) and never cause a pc0 transition, so those guards are dropped statically.
		//  - the remaining runtime conditions (pending pre-transfer / rptc / rptc_next, e.g. a deopted rptk)
		//    are one combined byte OR -> slow. Skips m_pc_hits[pc]++ (a diagnostic histogram not in the gate
		//    fingerprint), exactly like emit_post_regcached in the unrolled frame.
		// Fast-path epilogue fusions (2026-07-07; each runs on EVERY instruction of every pooled frame):
		//  - pc is stored as the static constant pc_k+1 (pure store, not an inc RMW), which also makes the
		//    linear "pc advanced" guard vacuous on this path — it is emitted after a SLOW post only.
		//  - `dec icount` sets SF/ZF/OF, so the timeslice exit is the fused `dec; jle done`; the separate
		//    `cmp icount,0` load survives only after a slow post (jit_w_post decrements in memory).
		//  - the frame-level S_IDLE test is emitted only when the chain contains a deopted op: the `idle`
		//    op is the ONLY S_IDLE setter in the op set (verified in the generated .hxx) and it is never
		//    poolable, and jit_pc_post cannot set S_IDLE either (apply_pending_pre_transfer only writes
		//    dmem; the serial-phase arm is excluded frame-level by !serial_cycle_model).
		auto post_slow = [&]() {
			a.mov(ARG0, DEV);
			a.mov(ARG1.r32(), Imm(ipc_k));
			a.mov(ARG2.r32(), Imm(chain_ipc));
			a.call(thunks.post);
		};
		// eax = -1 is the "re-decode at the device pc" resume sentinel (same as a branch post's return).
		auto force_fb = [&]() {
			if (force_fb_pc >= 0 && pc_k == force_fb_pc)
			{
				a.call(thunks.forced_fallback);
				a.mov(x86::eax, Imm(-1));
				a.jmp(fallback);
			}
		};
		// timeslice bound (on BOTH post paths below): the interpreter's outer loop halts at icount<=0 and
		// finishes the frame on the NEXT timeslice. Match it — exit the compiled frame at icount<=0 after
		// any PC's post (device pc/state left mid-frame; the interpreter resumes it). Running the whole
		// frame regardless of the timeslice would desync the DSP from the H8 (the DSP3-long-frame
		// extra-XM-frame divergence). Never remove these exits.
		if (is_branch || pc_k == 0 || pc_k == 255)
		{
			post_slow();
			a.cmp(x86::dword_ptr(DEV, off_ic), Imm(0));
			a.jle(done);
			force_fb();
		}
		else
		{
			Label pslow = a.new_label(), pnext = a.new_label();
			if (fused_flags)
			{
				a.mov(x86::ecx, x86::dword_ptr(DEV, off_flags_lo));
				a.test(x86::ecx, Imm(0x00ffffff));   // any of the three adjacent flag bytes nonzero -> slow
				a.jnz(pslow);
			}
			else
			{
				a.movzx(x86::ecx, x86::byte_ptr(DEV, off_pend));
				a.or_(x86::cl, x86::byte_ptr(DEV, off_rptc));
				a.or_(x86::cl, x86::byte_ptr(DEV, off_rptcn));
				a.jnz(pslow);
			}
			a.mov(x86::byte_ptr(DEV, off_pc), Imm(pc_k + 1));   // FAST: pc = pc_k+1 (static; no pc0 transition)
			a.dec(x86::dword_ptr(DEV, off_ic));                  //       icount--, flags set ...
			a.jle(done);                                         //       ... fused timeslice exit
			force_fb();
			if (chain_had_deopt)
			{
				// a deopted op (e.g. `idle`) may have set S_IDLE mid-chain -> end the frame.
				a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));
				a.test(x86::ecx, Imm(idle_mask));
				a.jnz(done);
			}
			a.jmp(pnext);   // pc == pc_k+1 by construction -> no linear guard on the fast path
			a.bind(pslow);
			post_slow();
			a.cmp(x86::dword_ptr(DEV, off_ic), Imm(0));
			a.jle(done);
			force_fb();
			if (chain_had_deopt)
			{
				a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));
				a.test(x86::ecx, Imm(idle_mask));
				a.jnz(done);
			}
			if (k < nsteps - 1)
			{   // safety: the slow post must have advanced pc to the next emitted PC (linear); a repeat
				// (rptc) or any other divergence lands elsewhere -> fallback (eax = the post's resume ipc).
				a.movzx(x86::ecx, x86::byte_ptr(DEV, off_pc));
				a.cmp(x86::ecx, Imm(order[k + 1].first));
				a.jne(fallback);
			}
			a.bind(pnext);
			continue;   // epilogue fully emitted for the non-branch [1,254] case
		}

		if (is_branch)
		{
			// M3 native branch: the branch op + post set device pc to target (taken) or pc+1 (not taken).
			// Dispatch on the (robust) device pc: taken -> jump to the target PC's label; else fall through
			// to the next emitted PC (= pc_k+1, the not-taken path).
			auto tl = pc_label.find(branch_target);
			if (tl == pc_label.end())
				return false;   // branch target not compiled (shouldn't happen for forward branches) -> bail
			// (2026-07-07) latent-hole hardening: a deopted op EARLIER in this chain (e.g. `idle`) may
			// have set S_IDLE and skipped the branch op via the deopt chain_end — the interpreter's outer
			// loop stops there, so the frame must end BEFORE dispatching the branch. No current program
			// pairs idle with a branch in one word, but the check costs 3 instructions on branch PCs only.
			a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));
			a.test(x86::ecx, Imm(idle_mask));
			a.jnz(done);
			// F2 (2026-07-05): fully guard the branch edge — it was the one unguarded control exit.
			// (a) repeat pending (rptc != 0 after the post): the interpreter re-runs the branch CHAIN
			//     rptc times (jit_pc_post returned ipc = iipc); the driver can't model that -> fallback.
			//     Catches taken AND not-taken repeats. eax = the slow post's resume ipc (branch PCs
			//     always take the slow post), which is exactly what `fallback` expects.
			a.movzx(x86::ecx, x86::byte_ptr(DEV, off_rptc));
			a.test(x86::ecx, x86::ecx);
			a.jnz(fallback);
			a.movzx(x86::ecx, x86::byte_ptr(DEV, off_pc));
			a.cmp(x86::ecx, Imm(branch_target));
			a.je(tl->second);
			if (k < nsteps - 1)
			{
				// (b) the not-taken path must have landed on pc_k+1; anything else would silently
				//     execute the wrong PC's body -> fallback (same guard every linear exit has).
				a.cmp(x86::ecx, Imm(order[k + 1].first));
				a.jne(fallback);
			}
		}
		else
		{
			// pc_k in {0,255} non-branch (always slow post): frame-level idle + linear guard
			// (uses ecx; eax = post's return preserved for the fallback).
			a.mov(x86::ecx, x86::dword_ptr(DEV, off_sti));
			a.test(x86::ecx, Imm(idle_mask));
			a.jnz(done);
			if (k < nsteps - 1)
			{   // safety: pc must have advanced to the next emitted PC (linear); else unexpected divergence.
				a.movzx(x86::ecx, x86::byte_ptr(DEV, off_pc));
				a.cmp(x86::ecx, Imm(order[k + 1].first));
				a.jne(fallback);   // eax = resume ipc at the jump
			}
		}
	}
	a.jmp(done);

	// fallback: eax = resume ipc. finish the frame via the interpreter (mutates state -> spill/reload).
	a.bind(fallback);
	a.mov(x86::r10d, x86::eax);   // stash resume ipc (r10 survives spill; spill uses only DEV + pinned)
	spill();
	a.mov(ARG0, DEV);
	a.mov(ARG1.r32(), x86::r10d);
	a.call(thunks.rest);
	reload();
	// fall through to done

	a.bind(done);
	spill();
#if defined(_WIN32)
	a.add(x86::rsp, Imm(32));
#endif
	a.pop(x86::r15); a.pop(x86::r14); a.pop(x86::r13); a.pop(x86::r12); a.pop(x86::rbx);
	a.ret();

	// entry jump table (data, never executed): one 8-byte absolute label address per emitted PC, indexed by
	// (device_pc - start_pc)&0xff. Emitted after ret so it doesn't fall into the code path.
	a.bind(entry_table);
	for (int k = 0; k < nsteps; k++)
		a.embed_label_delta(pc_label[order[k].first], entry_table, 4);   // signed 32-bit (label - table), relocation-free
	return true;
}

// Emit the SHARED op-body pool (one body per op-id in `pool`), each via emit_op_pooled + `ret`. Reached
// only by `call` from the drivers. Uses each body's owner device for the emit-time probe.
void emit_pool_bodies(x86::Assembler &a, const std::map<unsigned, PoolBody> &pool)
{
	const x86::Gp DEV = x86::rbx, MACC = x86::r12, MACR = x86::r13, MACW = x86::r14, AACC = x86::r15, PARAM = x86::r11;
	const tms57002::PoolRegs pregs{ DEV, MACC, MACR, MACW, AACC, PARAM };
	for (const auto &kv : pool)
	{
		a.bind(kv.second.label);
		tms57002::emit_op_pooled(a, pregs, *kv.second.owner, kv.first, kv.second.icd);
		a.ret();
	}
}

#elif defined(KPROP_JIT_A64)
// ===== a64 twins (2026-07-07 port) ==========================================
// Same structure and guard set as the x86 driver above, translated per the port rules (explicit
// flag ops, cmp+csel, materialized constants, ldr/orr/str RMW). Convention: DEV=x19, MACC=x20,
// MACR=x21, MACW=x22, AACC=x23(w23), PARAM=w15; driver scratch w9/w10/x11 (all caller-saved);
// x16 is the thunk veneer register; x30 saved in the prologue (the driver bl's into pool bodies).

// C-ABI helper thunks, one shared in-region veneer per helper (the a64 analog of the x86 rel32
// thunks: `bl thunk` is a 4-byte instruction vs a 4-insn mov+blr at every call site).
void emit_thunks(a64::Assembler &a, const Thunks &t)
{
	auto thunk = [&](Label lbl, void *fn) { a.bind(lbl); a.mov(a64::x16, Imm(uint64_t(fn))); a.br(a64::x16); };
	thunk(t.xm, reinterpret_cast<void *>(&jit_w_xm));
	thunk(t.op, reinterpret_cast<void *>(&jit_w_op));
	thunk(t.post, reinterpret_cast<void *>(&jit_w_post));
	thunk(t.rest, reinterpret_cast<void *>(&jit_w_run_rest));
	thunk(t.forced_fallback, reinterpret_cast<void *>(&jit_w_forced_midframe_fallback_hit));
}

// Emit ONE program's pooled driver (prologue..ret) into `a` — see the x86 twin for the design
// comments (entry table, F2/F5-era guards, timeslice contract, fallback); they apply verbatim.
bool emit_pooled_driver(a64::Assembler &a, tms57002_device &dsp,
                        const std::vector<std::pair<int, int>> &order, const std::map<unsigned, PoolBody> &pool,
                        const Thunks &thunks, bool cmem_per_op)
{
	const int off_macc = int(tms57002_device::jit_off_macc());
	const int off_macc_r = int(tms57002_device::jit_off_macc_read());
	const int off_macc_w = int(tms57002_device::jit_off_macc_write());
	const int off_aacc = int(tms57002_device::jit_off_aacc());
	const int off_sti = int(tms57002_device::jit_off_sti());
	const int off_ca = int(tms57002_device::jit_off_ca());
	const int off_id = int(tms57002_device::jit_off_id());
	const int off_pc = int(tms57002_device::jit_off_pc());
	const int off_ic = int(tms57002_device::jit_off_icount());
	const int off_pend = int(tms57002_device::jit_off_pending_pre_transfer());
	const int off_rptc = int(tms57002_device::jit_off_rptc());
	const int off_rptcn = int(tms57002_device::jit_off_rptc_next());
	const u32 idle_mask = tms57002_device::jit_s_idle_mask();
	const u32 rw_mask = tms57002_device::jit_s_read_mask() | tms57002_device::jit_s_write_mask();
	const a64::Gp DEV = a64::x19, MACC = a64::x20, MACR = a64::x21, MACW = a64::x22, AACC = a64::x23, PARAM = a64::x15;
	const a64::Gp S0 = a64::x9, S1 = a64::x10, S2 = a64::x11;   // driver scratch (caller-saved)
	const int off_uc_count = int(tms57002_device::jit_off_uc_count());   // CMEM-deopt guard (KPROP_PF4_CMEM_DEOPT)
	const int off_pf4_force = int(tms57002_device::jit_off_pf4_force());
	const bool cmem_deopt = dsp.jit_pf4_cmem_deopt();

	const int nsteps = int(order.size());
	if (nsteps == 0)
		return false;
	bool program_reads_cmem = false;
	for (const auto &entry : order)
	{
		int cur = entry.second;
		for (int steps = 0; steps <= 64; ++steps)
		{
			const unsigned op = dsp.jit_inst_op(cur);
			if (op < 4) break;
			program_reads_cmem |= tms57002::op_reads_cmem(dsp, op);
			cur = dsp.jit_inst_next(cur);
		}
	}
	const bool hoist_cmem_guard = cmem_deopt && program_reads_cmem
		&& !cmem_per_op;

	std::map<int, Label> pc_label;
	for (int k = 0; k < nsteps; k++)
		pc_label.emplace(order[k].first, a.new_label());
	Label done = a.new_label(), fallback = a.new_label();
	auto is_branch_mn = [](const char *mn) { return mn && (__builtin_strcmp(mn, "bgz") == 0 || __builtin_strcmp(mn, "blz") == 0 || __builtin_strcmp(mn, "bnz") == 0 || __builtin_strcmp(mn, "bv") == 0 || __builtin_strcmp(mn, "b") == 0); };

	// Offset-guarded device-field accessors. Device offsets run past 0x3a00, which exceeds the
	// scaled-imm12 range of the BYTE forms (4095) and can exceed the 32-bit forms (16380) — a raw
	// a64::ptr(DEV, off) there is an encoding error (and without the error sink it silently dropped
	// the instruction: the original port bug). x8 is the address temp, never a value carrier.
	auto lea_dev = [&](int off) {
		if (off >= 0 && off <= 4095) a.add(a64::x8, DEV, Imm(off));
		else { a.mov(a64::x8, Imm(off)); a.add(a64::x8, DEV, a64::x8); }
	};
	auto ld64 = [&](a64::Gp x, int off) {
		if (off >= 0 && off <= 32760 && !(off & 7)) a.ldr(x.x(), a64::ptr(DEV, off));
		else { lea_dev(off); a.ldr(x.x(), a64::ptr(a64::x8)); }
	};
	auto st64 = [&](a64::Gp x, int off) {
		if (off >= 0 && off <= 32760 && !(off & 7)) a.str(x.x(), a64::ptr(DEV, off));
		else { lea_dev(off); a.str(x.x(), a64::ptr(a64::x8)); }
	};
	auto ld32 = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 16380 && !(off & 3)) a.ldr(w.w(), a64::ptr(DEV, off));
		else { lea_dev(off); a.ldr(w.w(), a64::ptr(a64::x8)); }
	};
	auto st32 = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 16380 && !(off & 3)) a.str(w.w(), a64::ptr(DEV, off));
		else { lea_dev(off); a.str(w.w(), a64::ptr(a64::x8)); }
	};
	auto ldb = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 4095) a.ldrb(w.w(), a64::ptr(DEV, off));
		else { lea_dev(off); a.ldrb(w.w(), a64::ptr(a64::x8)); }
	};
	auto stb = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 4095) a.strb(w.w(), a64::ptr(DEV, off));
		else { lea_dev(off); a.strb(w.w(), a64::ptr(a64::x8)); }
	};

	auto spill = [&]() {
		st64(MACC, off_macc);
		st64(MACR, off_macc_r);
		st64(MACW, off_macc_w);
		st32(AACC, off_aacc);
	};
	auto reload = [&]() {
		ld64(MACC, off_macc);
		ld64(MACR, off_macc_r);
		ld64(MACW, off_macc_w);
		ld32(AACC, off_aacc);
	};
	// prologue: fp/lr + the pinned callee-saved regs (x30 is live across the pool-body `bl`s).
	a.stp(a64::x29, a64::x30, a64::ptr_pre(a64::sp, -64));
	a.mov(a64::x29, a64::sp);
	a.stp(a64::x19, a64::x20, a64::ptr(a64::sp, 16));
	a.stp(a64::x21, a64::x22, a64::ptr(a64::sp, 32));
	a.str(a64::x23, a64::ptr(a64::sp, 48));
	a.mov(DEV, a64::x0);
	reload();

	if (hoist_cmem_guard)
	{
		Label safe = a.new_label();
		ldb(S0, off_uc_count);
		a.cbz(S0.w(), safe);
		ldb(S0, off_pf4_force);
		a.cbnz(S0.w(), safe);
		a.mov(a64::w0, Imm(-1));
		a.b(fallback);
		a.bind(safe);
	}

	// [continuation] mid-PC entry: dispatch on the CURRENT device pc via the embedded delta table
	// (adr = pc-relative, relocation-free; the pooled region is far below adr's ±1MB range).
	const Label entry_table = a.new_label();
	{
		const int base_pc = order.front().first;
		ldb(S0, off_pc);
		if (base_pc) a.sub(S0.w(), S0.w(), Imm(base_pc));
		a.and_(S0.w(), S0.w(), Imm(0xff));
		{	// Defense-in-depth (F5 spirit): an out-of-frame index would read past the delta table and
			// branch wild — the C++ entry guards make that unreachable, so trap loudly (brk #1) rather
			// than jump to garbage if a guard is ever broken. (Caught the port's silent-drop bug live.)
			Label in_range = a.new_label();
			a.cmp(S0.w(), Imm(nsteps));
			a.b_lo(in_range);
			a.brk(Imm(1));
			a.bind(in_range);
		}
		a.adr(S1, entry_table);                                  // S1 = &table
		a.ldrsw(S2, a64::ptr(S1, S0, a64::lsl(2)));              // S2 = signed 32-bit (label - table)
		a.add(S1, S1, S2);
		a.br(S1);
	}

	for (int k = 0; k < nsteps; k++)
	{
		const int pc_k = order[k].first, ipc_k = order[k].second;
		a.bind(pc_label[pc_k]);
		Label chain_end = a.new_label();

		// jit_pc_pre: xm_step only if S_READ|S_WRITE (C-ABI, preserves pinned regs; no macc/aacc touch).
		{
			Label skip = a.new_label();
			ld32(S0, off_sti);
			a.mov(S1.w(), Imm(rw_mask));
			a.tst(S0.w(), S1.w());
			a.b_eq(skip);
			a.mov(a64::x0, DEV);
			a.bl(thunks.xm);
			a.bind(skip);
		}
		// MACC pipeline (registers): macc_read = macc_write; macc_write = macc.
		a.mov(MACR, MACW);
		a.mov(MACW, MACC);

		int cur = ipc_k, chain_ipc = dsp.jit_inst_next(cur);
		bool is_branch = false; int branch_target = -1;
		for (int steps = 0; ; steps++)
		{
			if (steps > 64)
				return false;
			const unsigned op = dsp.jit_inst_op(cur);
			const int next = dsp.jit_inst_next(cur);
			chain_ipc = next;
			if (op >= 4 && !dsp.jit_op_mnemonic(op))
				return false;   // stale/out-of-range op -> bail; caller runs this frame via interpreter
			if (op < 4)
			{
				if (op == 1 || op == 3) { ldb(S0, off_ca); a.add(S0.w(), S0.w(), Imm(1)); stb(S0, off_ca); }
				if (op == 2 || op == 3) { ldb(S0, off_id); a.add(S0.w(), S0.w(), Imm(1)); stb(S0, off_id); }
				break;
			}
			if (is_branch_mn(dsp.jit_op_mnemonic(op))) { is_branch = true; branch_target = int(dsp.jit_inst_param(cur)); }
			auto pit = pool.find(op);
			if (pit != pool.end())
			{
				// POOL: operand in w15 (=PARAM; write the W form so x15's upper bits stay zero),
				// then call the shared body under the private convention.
				if (!hoist_cmem_guard && cmem_deopt && tms57002::op_reads_cmem(dsp, op))
				{
					// CMEM-read op under KPROP_PF4_CMEM_DEOPT: if an update is pending
					// (count != 0) and not force-unsafe, DEOPT to the interpreter (get_cmem
					// drains the queue at the correct time); else run the fast compiled body.
					const uint64_t icd = dsp.jit_inst_addr(cur);
					Label do_fast = a.new_label(), guard_done = a.new_label();
					ldb(S0, off_uc_count);
					a.cbz(S0.w(), do_fast);          // no pending -> fast
					ldb(S0, off_pf4_force);
					a.cbnz(S0.w(), do_fast);         // force-unsafe baseline -> fast (divergent)
					spill();
					a.mov(a64::x0, DEV);
					a.mov(a64::w1, Imm(op));
					a.mov(a64::x2, Imm(icd));
					a.bl(thunks.op);
					reload();
					ld32(S0, off_sti);               // deopted op may set S_IDLE -> end chain
					a.mov(S1.w(), Imm(idle_mask));
					a.tst(S0.w(), S1.w());
					a.b_ne(chain_end);
					a.b(guard_done);
					a.bind(do_fast);
					a.mov(PARAM.w(), Imm(u32(u8(dsp.jit_inst_param(cur)))));
					a.bl(pit->second.label);
					a.bind(guard_done);
				}
			else
			{
				a.mov(PARAM.w(), Imm(u32(u8(dsp.jit_inst_param(cur)))));
				a.bl(pit->second.label);
			}
			}
			else
			{
				// DEOPT: spill pinned, run the op via the interpreter, reload.
				const uint64_t icd = dsp.jit_inst_addr(cur);
				spill();
				a.mov(a64::x0, DEV);
				a.mov(a64::w1, Imm(op));
				a.mov(a64::x2, Imm(icd));
				a.bl(thunks.op);
				reload();
				ld32(S0, off_sti);   // op set S_IDLE -> end chain
				a.mov(S1.w(), Imm(idle_mask));
				a.tst(S0.w(), S1.w());
				a.b_ne(chain_end);
			}
			cur = next;
		}
		a.bind(chain_end);

		// post — same compile-time specialization as the x86 twin (fast `icount--; pc++` for
		// non-branch pc in [1,254] with no pending/rptc byte set).
		auto post_slow = [&]() {
			a.mov(a64::x0, DEV);
			a.mov(a64::w1, Imm(ipc_k));
			a.mov(a64::w2, Imm(chain_ipc));
			a.bl(thunks.post);
		};
		if (is_branch || pc_k == 0 || pc_k == 255)
		{
			post_slow();
		}
		else
		{
			Label pslow = a.new_label(), pdone = a.new_label();
			ldb(S0, off_pend);
			ldb(S1, off_rptc);
			a.orr(S0.w(), S0.w(), S1.w());
			ldb(S1, off_rptcn);
			a.orr(S0.w(), S0.w(), S1.w());
			a.cbnz(S0.w(), pslow);
			ld32(S0, off_ic);    // FAST: icount--
			a.sub(S0.w(), S0.w(), Imm(1));
			st32(S0, off_ic);
			ldb(S0, off_pc);     //       pc++  (pc_k in [1,254] -> no pc0 transition)
			a.add(S0.w(), S0.w(), Imm(1));
			stb(S0, off_pc);
			a.b(pdone);
			a.bind(pslow);
			post_slow();
			a.bind(pdone);
		}

		// timeslice bound: exit the compiled frame at icount<=0 (interpreter contract; see x86 twin).
		ld32(S0, off_ic);
		a.cmp(S0.w(), Imm(0));
		a.b_le(done);

		// [KPROP_PF4_FORCE_MIDFRAME_FALLBACK=<pc>] F3 test knob — force the mid-frame fallback here.
		{
			static const int force_fb_pc = [] {
				const char *e = std::getenv("KPROP_PF4_FORCE_MIDFRAME_FALLBACK");
				return (e && *e) ? int(std::strtol(e, nullptr, 0)) : -1;
			}();
			if (force_fb_pc >= 0 && pc_k == force_fb_pc)
			{
				a.bl(thunks.forced_fallback);
				a.mov(a64::w0, Imm(-1));   // "re-decode at the device pc" resume sentinel
				a.b(fallback);
			}
		}

		if (is_branch)
		{
			auto tl = pc_label.find(branch_target);
			if (tl == pc_label.end())
				return false;   // branch target not compiled -> bail
			// F2: repeat pending -> fallback (w0 = the slow post's resume ipc; branch PCs always
			// take the slow post). Then dispatch on the device pc; not-taken must land on pc_k+1.
			ldb(S0, off_rptc);
			a.cbnz(S0.w(), fallback);
			ldb(S0, off_pc);
			a.cmp(S0.w(), Imm(branch_target));
			a.b_eq(tl->second);
			if (k < nsteps - 1)
			{
				a.cmp(S0.w(), Imm(order[k + 1].first));
				a.b_ne(fallback);
			}
		}
		else
		{
			// frame-level idle (S0/S1; w0 = post's return preserved for the fallback).
			ld32(S0, off_sti);
			a.mov(S1.w(), Imm(idle_mask));
			a.tst(S0.w(), S1.w());
			a.b_ne(done);
			if (k < nsteps - 1)
			{   // safety: pc must have advanced to the next emitted PC (linear); else divergence.
				ldb(S0, off_pc);
				a.cmp(S0.w(), Imm(order[k + 1].first));
				a.b_ne(fallback);   // w0 = resume ipc at the jump
			}
		}
	}
	a.b(done);

	// fallback: w0 = resume ipc. finish the frame via the interpreter (mutates state -> spill/reload).
	a.bind(fallback);
	a.mov(S0.w(), a64::w0);   // stash resume ipc (spill touches only DEV + pinned)
	spill();
	a.mov(a64::x0, DEV);
	a.mov(a64::w1, S0.w());
	a.bl(thunks.rest);
	reload();
	// fall through to done

	a.bind(done);
	spill();
	a.ldp(a64::x19, a64::x20, a64::ptr(a64::sp, 16));
	a.ldp(a64::x21, a64::x22, a64::ptr(a64::sp, 32));
	a.ldr(a64::x23, a64::ptr(a64::sp, 48));
	a.ldp(a64::x29, a64::x30, a64::ptr_post(a64::sp, 64));
	a.ret(a64::x30);

	// entry delta table (data, never executed): one signed 32-bit (label - table) per emitted PC.
	// 4-byte entries after a 4-byte-aligned ret keep the region aligned.
	a.bind(entry_table);
	for (int k = 0; k < nsteps; k++)
		a.embed_label_delta(pc_label[order[k].first], entry_table, 4);
	return true;
}

// Emit the SHARED op-body pool (one body per op-id), each via the a64 emit_op_pooled + `ret`.
// Reached only by `bl` from the drivers; bodies never touch x30.
void emit_pool_bodies(a64::Assembler &a, const std::map<unsigned, PoolBody> &pool)
{
	const tms57002::PoolRegs pregs{ a64::x19, a64::x20, a64::x21, a64::x22, a64::x23, a64::x15 };
	for (const auto &kv : pool)
	{
		a.bind(kv.second.label);
		tms57002::emit_op_pooled(a, pregs, *kv.second.owner, kv.first, kv.second.icd);
		a.ret(a64::x30);
	}
}
#endif  // KPROP_JIT_X64 / KPROP_JIT_A64
}  // namespace
#endif

Jit::FrameFn Jit::compile_frame_pooled(tms57002_device &dsp, int max_steps, int start_pc,
	bool cmem_per_op)
{
#if defined(KPROP_JIT_X64) || defined(KPROP_JIT_A64)
	using namespace asmjit;
	(void)max_steps; (void)start_pc;
	if (m_pooled_order.empty())
		return nullptr;
	if (!m_pooled_order_mode_safe)
	{
		// A branch skips a mode-setter (build_pooled_order's F5 check) -> the fall-through decode
		// variants aren't proven path-independent -> refuse; this program runs via the interpreter.
		// KPROP_PF4_MODE_UNSAFE_COMPILE=1 compiles it anyway (the pre-audit behavior — byte-identical
		// on the current corpus, where the exercised runtime path matches the baked variants, but
		// UNPROVEN for other patches). Diagnosis/perf escape hatch; the sound fix is refining the
		// check to compare only mode fields consumed downstream of the join (see the audit note).
		// KPSHIP-ENGINE: JIT unsafe-compile escape hatch (must stay byte-identical). Bake safe default, delete.
		static const bool s_mode_unsafe_ok = [] {
			const char *e = std::getenv("KPROP_PF4_MODE_UNSAFE_COMPILE");
			return e && *e && *e != '0';
		}();
		if (!s_mode_unsafe_ok)
			return nullptr;
	}
	CodeHolder code;
	if (code.init(m_rt.environment()) != kErrorOk)
		return nullptr;
	PooledErrorSink esink;
	code.set_error_handler(&esink);
	PooledAssembler a(&code);

	// Compile-time diagnostic: rank the operations baked into each live DSP program. This is
	// deliberately static rather than a counter in generated code, so enabling it cannot perturb
	// the hot path. It gives selective-inlining experiments a workload-derived target list.
	static const bool s_op_stats = [] {
		const char *e = std::getenv("KPROP_PF4_OP_STATS");
		return e && *e && *e != '0';
	}();
	if (s_op_stats)
	{
		std::map<std::string, int> counts;
		int total = 0;
		for (const auto &entry : m_pooled_order)
		{
			int cur = entry.second;
			for (int steps = 0; steps <= 64; ++steps)
			{
				const unsigned op = dsp.jit_inst_op(cur);
				if (op < 4)
					break;
				const char *mn = dsp.jit_op_mnemonic(op);
				++counts[mn ? mn : "?"];
				++total;
				cur = dsp.jit_inst_next(cur);
			}
		}
		std::vector<std::pair<std::string, int>> ranked(counts.begin(), counts.end());
		std::sort(ranked.begin(), ranked.end(), [](const auto &lhs, const auto &rhs) {
			return lhs.second != rhs.second ? lhs.second > rhs.second : lhs.first < rhs.first;
		});
		std::fprintf(stderr, "[pooled-ops] prog=%016llx pcs=%zu ops=%d",
			(unsigned long long)m_cur_prog_hash, m_pooled_order.size(), total);
		for (std::size_t i = 0; i < ranked.size() && i < 16; ++i)
			std::fprintf(stderr, " %s=%d", ranked[i].first.c_str(), ranked[i].second);
		std::fputc('\n', stderr);
	}

	// One driver + its own pool copy (per-program). scan_pool BEFORE the driver (labels referenced by it).
	std::map<unsigned, PoolBody> pool;
	scan_pool(a, dsp, m_pooled_order, pool);
	Thunks thunks{ a.new_label(), a.new_label(), a.new_label(), a.new_label(), a.new_label() };
	if (!emit_pooled_driver(a, dsp, m_pooled_order, pool, thunks, cmem_per_op))
		return nullptr;
	emit_pool_bodies(a, pool);
	emit_thunks(a, thunks);
	if (esink.err != Error::kOk)   // any dropped/failed encoding -> refuse the frame (interpreter runs it)
		return nullptr;

	{ static size_t last = 0; if (code.code_size() != last) { last = code.code_size(); std::fprintf(stderr, "[pooled-size] pooled frame: %zu bytes (%d steps, %zu shared bodies)\n", code.code_size(), int(m_pooled_order.size()), pool.size()); } }

	FrameFn out = nullptr;
	if (m_rt.add(&out, &code) != kErrorOk)
		return nullptr;
	return out;
#else
	(void)dsp; (void)max_steps; (void)start_pc; (void)cmem_per_op;
	return nullptr;
#endif
}

bool Jit::compile_fused(tms57002_device *const devs[3], const std::vector<std::pair<int, int>> *const orders[3],
                        FrameFn out[3], std::size_t *code_size)
{
	out[0] = out[1] = out[2] = nullptr;
#if defined(KPROP_JIT_X64) || defined(KPROP_JIT_A64)
	using namespace asmjit;
	CodeHolder code;
	if (code.init(m_rt.environment()) != kErrorOk)
		return false;
	PooledErrorSink esink;
	code.set_error_handler(&esink);
	PooledAssembler a(&code);

	// ONE shared pool = the union of all 3 programs' poolable ops (scanned first so every driver's
	// `call pool[op]` resolves to the single shared body). Then 3 drivers, each with an entry label,
	// then the pool bodies ONCE. Fused, only the ~18KB pool needs to stay L1I-resident; the 3 small
	// drivers stream from L2 — vs 3 separate frames each carrying their own pool copy (~3x30KB).
	std::map<unsigned, PoolBody> pool;
	for (int g = 0; g < 3; g++)
	{
		if (!orders[g] || orders[g]->empty())
			return false;
		scan_pool(a, *devs[g], *orders[g], pool);
	}
	Label entry[3] = { a.new_label(), a.new_label(), a.new_label() };
	Thunks thunks{ a.new_label(), a.new_label(), a.new_label(), a.new_label(), a.new_label() };
	for (int g = 0; g < 3; g++)
	{
		a.bind(entry[g]);
		if (!emit_pooled_driver(a, *devs[g], *orders[g], pool, thunks, false))
			return false;
	}
	const size_t drivers_bytes = code.code_size();   // 3 drivers only (before the shared pool bodies)
	emit_pool_bodies(a, pool);
	emit_thunks(a, thunks);
	if (esink.err != Error::kOk)   // any dropped/failed encoding -> refuse (interpreter runs these)
		return false;

	// The driver/pool split is the key finding: the per-PC DRIVER code dominates (~94%), the shared
	// op-body pool is tiny (~6%) — so sharing the pool barely shrinks the fused footprint. The retention
	// lever is driver-size reduction (post register-caching), NOT pool sharing. (One-time print.)
	{ static size_t last = 0; if (code.code_size() != last) { last = code.code_size();
		std::fprintf(stderr, "[fused-size] shared region %zu B = 3 drivers %zu (%.0f%%) + shared pool %zu (%.0f%%), %zu union op-ids\n",
			code.code_size(), drivers_bytes, 100.0 * double(drivers_bytes) / double(code.code_size()),
			code.code_size() - drivers_bytes, 100.0 * double(code.code_size() - drivers_bytes) / double(code.code_size()), pool.size()); } }
	if (code_size)
		*code_size = code.code_size();

	FrameFn base = nullptr;
	if (m_rt.add(&base, &code) != kErrorOk)
		return false;
	uint8_t *b = reinterpret_cast<uint8_t *>(base);
	for (int g = 0; g < 3; g++)
		out[g] = reinterpret_cast<FrameFn>(b + code.label_offset_from_base(entry[g]));
	return true;
#else
	(void)devs; (void)orders; (void)code_size;
	return false;
#endif
}

std::array<u32, 4> Jit::run_sample_frame_pooled(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps)
{
	// The pooled bodies assume render conditions (no pending cmem update, no debug forces, no serial
	// cycle model) so cmem reads are direct + C-call-free. If those don't hold, use the unconditionally
	// correct native frame. (Live/M5 must revisit mid-frame changes; static render always satisfies it.)
	if (!dsp.jit_pooled_safe())
	{
		m_pooled_fallbacks++;
		return run_sample_native(dsp, in, max_steps);
	}
	dsp.jit_begin_frame(in);
	dsp.jit_set_icount(max_steps);
	const int ms = max_steps > 0 ? max_steps : 1;
	const int start_pc = dsp.jit_pc();
	const u32 start_st1 = dsp.dbg_st1() & tms57002_device::jit_st1_cache();
	const u32 ver = dsp.jit_program_version();
	// Finish the ALREADY-BEGUN frame via the interpreter (used for the compile frame's result, for
	// programs whose compile failed, and when the optional guarded-CMEM variant is unavailable).
	// Mirrors the pre-M4 interpret loop and produces the bit-exact result.
	auto finish_via_interp = [&]() -> std::array<u32, 4> {
		m_frame_trace.clear();
		int guard = ms + 16;
		while (dsp.jit_running() && int(m_frame_trace.size()) < ms && guard-- > 0)
		{
			const int ipc = dsp.jit_decode_current();
			m_frame_trace.push_back(ipc);
			dsp.jit_run_chain(ipc);
		}
		dsp.jit_finalize_if_idle();
		return dsp.jit_end_frame();
	};
	auto active_fn = [&]() -> FrameFn {
		if (dsp.jit_cmem_pending())
		{
			if (m_active->cmem_fn)
			{
				m_pooled_cmem_runs++;
				return m_active->cmem_fn;
			}
			return nullptr;
		}
		return m_active->fn;
	};

	// STEADY STATE: program unchanged AND the active entry matches (st1/ms/pc) — run its compiled frame
	// directly. One int compare per frame; no hashing, no re-decode. This is the common case.
	if (ver == m_last_prog_version && m_active && m_active->fn && !m_active->failed
		&& m_active->max_steps == ms && m_active->start_pc == start_pc && m_active->start_st1 == start_st1)
	{
		FrameFn const fn = active_fn();
		if (!fn)
		{
			m_pooled_fallbacks++;
			return finish_via_interp();
		}
		m_pooled_runs++;
		fn(&dsp);
		dsp.jit_finalize_if_idle();
		return dsp.jit_end_frame();
	}

	// Program changed (or first frame / st1 / max_steps differ): re-hash only when the version moved,
	// then look up the compile cache by (program hash, start_st1, max_steps).
	if (ver != m_last_prog_version) { m_last_prog_version = ver; m_cur_prog_hash = dsp.jit_program_hash(); }
	const auto key = std::make_tuple(m_cur_prog_hash, start_st1, ms);
	auto it = m_pooled_cache.find(key);
	if (it != m_pooled_cache.end())
	{
		m_active = &it->second;
		if (m_active->failed)   // known un-poolable program -> interpret the begun frame (== native for it)
		{
			m_pooled_fallbacks++;
			return finish_via_interp();
		}
		// CACHE HIT: re-prime the decode cache to the layout the compiled frame's baked pointers expect
		// (build_pooled_order is deterministic: same program+st1 -> same (pc->ipc) indices + contents),
		// then run the cached frame. NO asmjit codegen — the M4 reuse win.
		build_pooled_order(dsp, start_pc, ms);
		FrameFn const fn = active_fn();
		if (!fn)
		{
			m_pooled_fallbacks++;
			return finish_via_interp();
		}
		m_pooled_runs++;
		fn(&dsp);
		dsp.jit_finalize_if_idle();
		return dsp.jit_end_frame();
	}

	// CACHE MISS: build the program order (primes the decode cache), interpret one frame for the result,
	// compile, and store the entry. M3 native branches: build_pooled_order lays every reachable PC out in
	// program order at its correct st1; compile_frame_pooled emits bgz/blz/bnz as native jcc. A compile
	// that can't handle the shape returns nullptr -> the entry is marked failed and runs via interp.
	build_pooled_order(dsp, start_pc, ms);
	const auto result = finish_via_interp();
	PooledEntry &e = m_pooled_cache[key];
	e.fn = compile_frame_pooled(dsp, ms, start_pc, false);
	if (e.fn && dsp.jit_pf4_cmem_deopt() && !std::getenv("KPROP_PF4_FORCE_CMEM_COMPILE_FAIL")
		&& order_reads_cmem(dsp, m_pooled_order))
		e.cmem_fn = compile_frame_pooled(dsp, ms, start_pc, true);
	e.order = m_pooled_order;
	e.max_steps = ms;
	e.start_pc = start_pc;
	e.start_st1 = start_st1;
	e.failed = (e.fn == nullptr);
	if (e.fn) m_pooled_compiles++;
	if (e.cmem_fn) m_pooled_compiles++;
	m_active = &e;
	return result;
}

void Jit::run_begun_frame(tms57002_device &dsp, int max_steps)
{
	// The caller already began the frame (debug_begin_sample_frame) and reads serial_output afterwards.
	// The interpreter paths here use debug_run_cycles. [Historical note, corrected 2026-07-05: this
	// comment used to claim jit_run_chain "corrupts machine state / derails the H8" in the live machine —
	// that derailment predates 71cdb19 and was the run-past-the-timeslice icount-override bug fixed
	// there, not jit_run_chain itself (jit_run_rest loops it under jit_running(), timeslice-correct).
	// This whole path (run_begun_frame) is a superseded pre-M5 milestone kept for the harness.]
	if (m_frame_failed) {   // codegen unavailable -> finish the begun frame via the device interpreter
		dsp.debug_run_cycles(max_steps);
		return;
	}
	const int ms = max_steps > 0 ? max_steps : 1;
	const int start_pc = dsp.jit_pc();
	const u32 start_st1 = dsp.dbg_st1() & tms57002_device::jit_st1_cache();
	// Live machine RELOADS DSP programs (PLOAD) and the (pc,st1)-keyed decode cache churns, so a cached
	// compiled frame can go stale (baked ipcs/ops no longer match the program). Re-decode the start PC;
	// if its chain head differs from the recorded trace, the program/cache changed -> recompile.
	const int start_ipc = dsp.jit_decode_current();
	if (!m_frame_fn || m_frame_max_steps != ms || m_frame_start_pc != start_pc || m_frame_start_st1 != start_st1
		|| m_frame_trace.empty() || m_frame_trace.front() != start_ipc) {
		m_frame_trace.clear();   // record the per-PC decode trace by running the begun frame PC-by-PC via the
		int guard = ms + 16;     // DEVICE interpreter (debug_run_cycles), NOT jit_run_chain (which derails).
		while (!dsp.jit_is_idle() && int(m_frame_trace.size()) < ms && guard-- > 0) {
			m_frame_trace.push_back(dsp.jit_decode_current());
			dsp.debug_run_cycles(1);
		}
		m_frame_fn = compile_frame(dsp, ms, start_pc);   // nullptr on a TRANSIENT failure (cache churn) -> retry
		m_frame_max_steps = ms;                          // next frame. Do NOT permanently disable: the live
		m_frame_start_pc = start_pc;                     // program stabilizes and compiles once the cache settles.
		m_frame_start_st1 = start_st1;
		return;   // the frame already ran (interpreted) while recording
	}
	dsp.jit_set_icount(max_steps);
	m_frame_fn(&dsp);   // run the compiled native frame
	dsp.jit_finalize_if_idle();
}

bool Jit::run_begun_frame_pooled(tms57002_device &dsp, int max_steps)
{
	// M5 live seam. The frame is ALREADY begun (route_dsp_serial). Return true if we ran it (the pooled
	// compiled frame, native — never jit_run_chain, which derails the live H8); return false to have the
	// CALLER run it via rt_run (the pf=2 path, whose cycle accounting the serial model depends on).
	const int ms = max_steps > 0 ? max_steps : 1;
	// stats gate hoisted to a static (2026-07-05): this function runs PER INSTRUCTION during
	// unsafe-window retries (~10M calls in a 15s note render) — a per-call getenv() environ scan
	// here is real time. The cadence and counters are per-Jit; include the device tag so
	// automated packaged-product gates can distinguish all three physical DSPs.
	// KPSHIP-TRACE: JIT pooled-frame stats to stderr. Strip.
	static const bool s_pf4_stats = std::getenv("KPROP_PF4_STATS") != nullptr;
	if (s_pf4_stats && (++m_stats_calls % 20000) == 0)
	{
		std::fprintf(stderr,
				"KPROP_JIT_RUNTIME tag=%s calls=%ld runs=%ld fallbacks=%ld compiles=%ld forced_midframe=%ld\n",
				dsp.tag(), m_stats_calls, m_pooled_runs, m_pooled_fallbacks,
				m_pooled_compiles, s_forced_midframe_fallback_hits);
		std::fprintf(stderr, "[pf4] calls=%ld runs=%ld fallbacks=%ld compiles=%ld forced_midframe=%ld\n",
				m_stats_calls, m_pooled_runs, m_pooled_fallbacks,
				m_pooled_compiles, s_forced_midframe_fallback_hits);
	}

	// Render conditions not met (serial cycle model / pending cmem / debug forces) -> caller rt_runs.
	// PLOAD still disengages pooled execution because the program space is in flux. CLOAD is safe under
	// the guarded-CMEM mode: its partial packet lives only in host[]; the completed value becomes visible
	// by advancing the update-queue head, which the guarded entry observes before a CMEM-reading op.
	if (!dsp.jit_pooled_safe() || dsp.jit_host_loading_unsafe())
	{
		// F1 (2026-07-05): keep m_pooled_partial PENDING here — deliberately. Unsafe conditions clear
		// MID-FRAME (the H8 streams cmem updates; get_cmem drains the queue), and the pending flag makes
		// the hook retry every instruction so the frame re-engages NATIVELY the moment it is safe again.
		// Clearing the flag here instead demoted every such frame's remainder to the interpreter
		// (measured: pf4 114% -> ~101% of realtime on the 12s gate).
		m_pooled_fallbacks++;
		return false;
	}

	const int cur_pc = dsp.jit_pc();
	const u32 ver = dsp.jit_program_version();
	auto active_fn = [&]() -> FrameFn {
		if (dsp.jit_cmem_pending())
		{
			if (m_active->cmem_fn)
			{
				m_pooled_cmem_runs++;
				return m_active->cmem_fn;
			}
			return nullptr;
		}
		return m_active->fn;
	};
	// NOTE: the compiled frame runs on the scheduler's CURRENT icount (the timeslice), NOT a fresh budget —
	// its per-PC posts decrement icount and it exits at icount<=0 (mid-frame), exactly like the
	// interpreter's `while(icount>0)`. Overriding icount (running the whole frame regardless of the
	// timeslice) desynced the DSP from the H8 on the long DSP3 fx frames (an extra XM frame).

	// CONTINUATION: the previous timeslice exited THIS frame partway (icount<=0). Resume the SAME compiled
	// frame at the current mid-frame pc — the fn's entry dispatch jumps to it (native, no interpreter). The
	// H8 ran between timeslices; the program must be unchanged and the pc must still lie within the active
	// frame (a PLOAD would bump ver; the pc bound catches an unexpected jump-out).
	if (m_pooled_partial)
	{
		m_pooled_partial = false;
		// F1 soundness note (2026-07-05, learned the hard way): resuming here is correct at ANY pc,
		// INCLUDING pc0, under these guards — the compiled frame is state-driven (entry dispatch on the
		// device pc; all data in device memory), F5 verifies the baked variants are valid for every
		// on-path arrival, host st1 writes only happen inside a PLOAD (= ver bump, caught here), and a
		// stale flag at a true frame boundary makes this resume equivalent to the steady-state run.
		// Do NOT "harden" pc0 by routing it to the fresh-frame path: never-idling programs cross pc0
		// MID-LOOP in whatever mode state their loop is in, and the fresh path's start_st1 key then
		// mints new cache entries and RE-PRIMES per lap (measured: pf4 collapsed to ~57% of realtime,
		// slower than the interpreter, with cache_flush/decode dominating the profile).
		if (ver == m_last_prog_version && m_active && m_active->fn && !m_active->failed
			&& m_active->max_steps == ms
			&& ((cur_pc - m_active->start_pc) & 0xff) < int(m_active->order.size()))
		{
			FrameFn const fn = active_fn();
			if (!fn)
			{
				m_pooled_fallbacks++;
				return false;
			}
			m_pooled_runs++;
			fn(&dsp);
			dsp.jit_finalize_if_idle();
			m_pooled_partial = !dsp.jit_is_idle();
			return true;
		}
		m_pooled_fallbacks++;   // can't resume -> the interpreter finishes this partial frame
		return false;
	}

	// FRESH FRAME: only at a program boundary (pc 0). A non-zero pc with no pending partial frame is not a
	// valid pooled entry -> the interpreter runs it.
	if (cur_pc != 0) { m_pooled_fallbacks++; return false; }
	const int start_pc = cur_pc;   // == 0
	const u32 start_st1 = dsp.dbg_st1() & tms57002_device::jit_st1_cache();

	// STEADY STATE: program unchanged since we primed AND the active entry matches -> run its compiled
	// frame directly (no hashing, no re-decode). Safe: in steady state this DSP runs only the compiled
	// frame (which does NOT decode), so cache.inst stays as the last re-prime left it. A PLOAD bumps the
	// version (below), forcing a re-prime; nothing else churns this device's decode cache mid-run.
	if (ver == m_last_prog_version && m_active && m_active->fn && !m_active->failed
		&& m_active->max_steps == ms && m_active->start_pc == start_pc && m_active->start_st1 == start_st1)
	{
		FrameFn const fn = active_fn();
		if (!fn)
		{
			m_pooled_fallbacks++;
			return false;
		}
		m_pooled_runs++;
		fn(&dsp);
		dsp.jit_finalize_if_idle();
		m_pooled_partial = !dsp.jit_is_idle();
		return true;
	}

	if (ver != m_last_prog_version) { m_last_prog_version = ver; m_cur_prog_hash = dsp.jit_program_hash(); }
	const auto key = std::make_tuple(m_cur_prog_hash, start_st1, ms);
	auto it = m_pooled_cache.find(key);
	if (it != m_pooled_cache.end())
	{
		m_active = &it->second;
		if (m_active->failed) { m_pooled_fallbacks++; return false; }   // caller rt_runs
		// HIT after a program/version change: RE-PRIME the decode cache (a PLOAD flushed it) so the
		// compiled frame's baked icd pointers reference the right cache.inst entries again. Deterministic +
		// decode-only + snapshot-safe; happens once per program switch, not per frame (steady state above).
		build_pooled_order(dsp, start_pc, ms);
		FrameFn const fn = active_fn();
		if (!fn)
		{
			m_pooled_fallbacks++;
			return false;
		}
		m_pooled_runs++;
		fn(&dsp);
		dsp.jit_finalize_if_idle();
		m_pooled_partial = !dsp.jit_is_idle();
		return true;
	}

	// MISS: build the order (decode-only, non-destructive) + compile. Do NOT run the frame here — return
	// false so the CALLER rt_runs THIS sample (byte-identical to pf=2); the compiled frame drives from the
	// next sample on. build_pooled_order snapshot-restores, so the device is untouched for the rt_run.
	build_pooled_order(dsp, start_pc, ms);
	PooledEntry &e = m_pooled_cache[key];
	e.fn = compile_frame_pooled(dsp, ms, start_pc, false);
	if (e.fn && dsp.jit_pf4_cmem_deopt() && !std::getenv("KPROP_PF4_FORCE_CMEM_COMPILE_FAIL")
		&& order_reads_cmem(dsp, m_pooled_order))
		e.cmem_fn = compile_frame_pooled(dsp, ms, start_pc, true);
	e.order = m_pooled_order;
	e.max_steps = ms;
	e.start_pc = start_pc;
	e.start_st1 = start_st1;
	e.failed = (e.fn == nullptr);
	if (e.fn) m_pooled_compiles++;
	if (e.cmem_fn) m_pooled_compiles++;
	m_active = &e;
	return false;
}

// VALIDATION: fully call-free, register-resident frame for an all-nop program. The MACC pipeline
// and pc/icount live in virtual registers for the WHOLE frame; the loop has ZERO calls and ZERO
// memory traffic (state synced to memory only at entry/exit). Tests the step-function hypothesis:
// does call-free register-resident machinery beat the clang-O2 interpreter? Bails (nullptr) if any
// real op appears — it only models nops (the machinery floor), which is all this validation needs.
Jit::FrameFn Jit::compile_frame_callfree(tms57002_device &dsp, int max_steps, int start_pc)
{
	using namespace asmjit;
	CodeHolder code;
	if (code.init(m_rt.environment()) != kErrorOk)
		return nullptr;
	const int off_macc = int(tms57002_device::jit_off_macc());
	const int off_macc_r = int(tms57002_device::jit_off_macc_read());
	const int off_macc_w = int(tms57002_device::jit_off_macc_write());
	const int off_pc = int(tms57002_device::jit_off_pc());
	const int off_ic = int(tms57002_device::jit_off_icount());

#if defined(KPROP_JIT_A64)
	a64::Compiler cc(&code);
	FuncNode *fn = cc.add_func(FuncSignature::build<void, tms57002_device *>());
	if (!fn) return nullptr;
	a64::Gp dev = cc.new_gpz("dev");
	fn->set_arg(0, dev);
	a64::Gp r_macc = cc.new_gpz(), r_macc_r = cc.new_gpz(), r_macc_w = cc.new_gpz();
	a64::Gp r_pc = cc.new_gp32(), r_ic = cc.new_gp32();
	cc.ldr(r_macc, a64::ptr(dev, off_macc));
	cc.ldr(r_macc_r, a64::ptr(dev, off_macc_r));
	cc.ldr(r_macc_w, a64::ptr(dev, off_macc_w));
	cc.ldrb(r_pc, a64::ptr(dev, off_pc));
	cc.ldr(r_ic, a64::ptr(dev, off_ic));
	for (int k = 0; k < max_steps; k++) {
		if (dsp.jit_inst_op(dsp.jit_decode((start_pc + k) & 0xff)) >= 4) return nullptr;
		cc.mov(r_macc_r, r_macc_w); cc.mov(r_macc_w, r_macc);
		cc.sub(r_ic, r_ic, Imm(1)); cc.add(r_pc, r_pc, Imm(1));
	}
	cc.str(r_macc, a64::ptr(dev, off_macc));
	cc.str(r_macc_r, a64::ptr(dev, off_macc_r));
	cc.str(r_macc_w, a64::ptr(dev, off_macc_w));
	cc.strb(r_pc, a64::ptr(dev, off_pc));
	cc.str(r_ic, a64::ptr(dev, off_ic));
	cc.ret(); cc.end_func();
	if (cc.finalize() != kErrorOk) return nullptr;
	FrameFn out = nullptr;
	if (m_rt.add(&out, &code) != kErrorOk) return nullptr;
	return out;

#elif defined(KPROP_JIT_X64)
	x86::Compiler cc(&code);
	FuncNode *fn = cc.add_func(FuncSignature::build<void, tms57002_device *>());
	if (!fn) return nullptr;
	x86::Gp dev = cc.new_gpz("dev");
	fn->set_arg(0, dev);
	x86::Gp r_macc = cc.new_gpz(), r_macc_r = cc.new_gpz(), r_macc_w = cc.new_gpz();
	x86::Gp r_pc = cc.new_gp32(), r_ic = cc.new_gp32();
	cc.mov(r_macc, x86::qword_ptr(dev, off_macc));     // load hot state ONCE
	cc.mov(r_macc_r, x86::qword_ptr(dev, off_macc_r));
	cc.mov(r_macc_w, x86::qword_ptr(dev, off_macc_w));
	cc.movzx(r_pc, x86::byte_ptr(dev, off_pc));
	cc.mov(r_ic, x86::dword_ptr(dev, off_ic));
	for (int k = 0; k < max_steps; k++) {
		if (dsp.jit_inst_op(dsp.jit_decode((start_pc + k) & 0xff)) >= 4) return nullptr;
		cc.mov(r_macc_r, r_macc_w); cc.mov(r_macc_w, r_macc);   // MACC pipeline, registers only
		cc.sub(r_ic, 1); cc.add(r_pc, 1);                       // post hot path, registers only
	}
	cc.mov(x86::qword_ptr(dev, off_macc), r_macc);     // write hot state back ONCE
	cc.mov(x86::qword_ptr(dev, off_macc_r), r_macc_r);
	cc.mov(x86::qword_ptr(dev, off_macc_w), r_macc_w);
	cc.mov(x86::byte_ptr(dev, off_pc), r_pc.r8());
	cc.mov(x86::dword_ptr(dev, off_ic), r_ic);
	cc.ret(); cc.end_func();
	if (cc.finalize() != kErrorOk) return nullptr;
	FrameFn out = nullptr;
	if (m_rt.add(&out, &code) != kErrorOk) return nullptr;
	{ static bool once = true; if (once) { once = false; std::fprintf(stderr, "[callfree] x86-64 JIT COMPILED + RUNNING (%d nops, %zu bytes)\n", max_steps, code.code_size()); } }
	return out;

#else
	(void)dsp; (void)max_steps; (void)start_pc;
	return nullptr;
#endif
}

std::array<u32, 4> Jit::run_sample_frame_callfree(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps)
{
	if (m_callfree_failed)
		return run_sample(dsp, in, max_steps);
	dsp.jit_begin_frame(in);
	dsp.jit_set_icount(max_steps);
	const int ms = max_steps > 0 ? max_steps : 1;
	const int start_pc = dsp.jit_pc();
	if (!m_callfree_fn || m_callfree_max != ms || m_callfree_start != start_pc)
	{
		m_callfree_fn = compile_frame_callfree(dsp, ms, start_pc);
		m_callfree_max = ms;
		m_callfree_start = start_pc;
		if (!m_callfree_fn)
		{
			m_callfree_failed = true;
			int ipc = -1, guard = ms + 16;
			while (dsp.jit_running() && guard-- > 0)
			{
				if (ipc == -1)
					ipc = dsp.jit_decode_current();
				ipc = dsp.jit_run_chain(ipc);
			}
			dsp.jit_finalize_if_idle();
			return dsp.jit_end_frame();
		}
	}
	m_callfree_fn(&dsp);
	dsp.jit_finalize_if_idle();
	return dsp.jit_end_frame();
}

int Jit::asmjit_self_check()
{
	asmjit::JitRuntime rt;   // ASMJIT_API ctor — constructing it proves asmjit links.
	(void)rt;
	return int(asmjit::Environment::host().arch());
}

}  // namespace tms57002
