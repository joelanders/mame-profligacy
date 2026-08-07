// license:BSD-3-Clause
//
// TMS57002 dynarec op-emitter (milestone 1). See tms57002_ops.h.
//
// Each op-id emits a standalone memory-based body reproducing the interpreter's ex_N bit-exact.
// Semantics are transcribed from tmsinstr.lst (the DSL the interpreter is generated from) and driven
// by the generated CINTRPDESC variant-descriptor table. The per-op gate proves each one against the
// oracle over randomized states. Correctness only here: state lives in the device struct (no register
// residency, no cmem fast-path — cmem reads call get_cmem directly, always bit-exact). Milestone 2
// re-emits these SAME lowerings under a pinned-register convention for residency.
//
// Deferred (return false -> gate skips, dynarec deopts): %mo/%mn/%mv macc-output ops (add/sub with m,
// smh*/sml*/slm*), sfma!=0 mac pre-shift, serial I/O (dis/domh/dos), branches (bgz/blz/bnz/bv, -> M3).
#include "tms57002_ops.h"

// ===== Arch-independent core: descriptor table + coverage predicate ==============================
// Shared by both backends so they can never diverge on WHICH ops are pooled (coverage parity by
// construction); only the lowerings below are per-arch, and the byte-identity gate checks those.
#if defined(__x86_64__) || defined(__aarch64__)
#include <cstring>
#include <cstdint>
#include <vector>

// Generated op-id -> (mnemonic, cat, id, decomposed variant bits, type) descriptor table.
namespace {
#define CINTRPDESC
#include "cpu/tms57002/tms57002.hxx"
#undef CINTRPDESC
}  // namespace

namespace tms57002 {

unsigned tms_op_desc_count() {
	return unsigned(sizeof(tms57002_op_desc) / sizeof(tms57002_op_desc[0]));
}

// Does this poolable op's body READ cmem (via cmem_into)? Such ops must deopt to the
// interpreter under pending CMEM updates (so get_cmem drains the queue with correct
// timing) when KPROP_PF4_CMEM_DEOPT is on. This is exact by descriptor form: treating a
// DMEM-only form as a reader needlessly deopts it on every pending update, while missing
// a real reader would DIVERGE in the PF4 CMEM oracle. CMEM-WRITE ops (sacc/smhc) write
// raw and are NOT queue-consuming -> excluded.
bool op_reads_cmem(tms57002_device &dsp, unsigned op) {
	const unsigned n = tms_op_desc_count();
	if (op < 4 || op >= n) return false;
	const tms57002_opdesc_t &d = tms57002_op_desc[op];
	if (!d.mn || !d.mn[0]) return false;
	const char *mn = d.mn;
	auto is = [&](const char *s) { return std::strcmp(mn, s) == 0; };
	(void)dsp;
	// Direct %c loads.
	if (is("lacc") || is("lmhc")) return true;
	// Logic forms are base+0=%d,a, base+1=%c,a, base+2=%d,%c.
	if (is("and")) return d.id != 0x14;
	if (is("or"))  return d.id != 0x17;
	if (is("xor")) return d.id != 0x1a;
	// add/sub low nibbles: %d,a and %d,m are DMEM-only; the remaining forms read %c.
	if (is("add") || is("sub")) {
		const unsigned lo = d.id & 0x0f;
		return lo == 0x04 || lo == 0x06 || lo == 0x07
			|| lo == 0x0a || lo == 0x0c || lo == 0x0d;
	}
	// Multiplier C-source is CMEM except for a,d / a,du and creg,d forms.
	if (is("mac"))  return d.id == 0x24 || d.id == 0x26;
	if (is("macs")) return d.id == 0x2e;
	if (is("macu")) return d.id == 0x29;
	if (is("mpy"))  return d.id == 0x21 || d.id == 0x22;
	if (is("mpyu")) return d.id == 0x28;
	return false;
}

bool op_poolable(tms57002_device &dsp, unsigned op) {
	const unsigned n = tms_op_desc_count();
	if (op < 4 || op >= n)
		return false;
	const tms57002_opdesc_t &d = tms57002_op_desc[op];
	if (!d.mn || !d.mn[0])
		return false;
	const char *mn = d.mn;
	auto is = [&](const char *s) { return std::strcmp(mn, s) == 0; };
	if (d.type == 'f') return true;   // flag-setters (affine st1)
	if (is("zmac") || is("zacc") || is("sfml") || is("sfmr")) return true;
	if (is("lcak") || is("lirk") || is("lira") || is("lcaa")) return true;   // NOT idle (sets S_IDLE -> deopt handles the check)
	if (is("lacc") || is("lacd")) return true;
	if (is("and") || is("or") || is("xor")) return true;
	if (is("neg") || is("abs")) return true;
	if (is("add") || is("sub")) return true;   // all forms (incl. %mo: %d,m / %c,m)
	if (is("sacc") || is("sacd") || is("srbd")) return true;
	if (is("smhc") || is("smhd") || is("smld") || is("slmh") || is("slml")) return true;   // %mo/%mn/%mv outputs
	if (is("lmhc") || is("lmhd") || is("lmld")) return true;   // macc loads
	if (is("dos") || is("domh")) return true;   // serial output (so[n]=v; serial cycle model excluded by the frame)
	// mac/mpy family (sfma=0 for the accumulating forms; jit_pooled_safe excludes the forward debug modes).
	if (is("mac")) return d.sfma == 0 && (d.id == 0x24 || d.id == 0x25 || d.id == 0x26);
	if (is("macs")) return d.sfma == 0 && d.id == 0x2e;
	if (is("macu")) return d.sfma == 0 && (d.id == 0x29 || d.id == 0x2a);
	if (is("mpy")) return d.id == 0x21 || d.id == 0x22 || d.id == 0x23;
	if (is("mpyu")) return d.id == 0x28;
	(void)dsp;
	return false;   // %mo (smh*/sml*/slm*)/lmh*/rde/wre/dis: later batch or deopt
}

u32 op_mode_consumed_mask(unsigned op) {
	// Built once, O(N^2) over the ~1.9k-entry descriptor table (a few ms, one-time): within a
	// (mnemonic, cat, .lst-id) family, any variant column that differs between two members was a
	// decode-selected dimension for the WHOLE family — union its ST1 mask into every member.
	// cmode/dmode are opcode-word addressing modes (not st1) and are deliberately unmapped. crm has
	// NO descriptor column — decode never enumerates it (the pooled bodies read the CRM view from
	// st1 at RUNTIME) — so CRM bits in a mode diff are consumed by nothing and reconverge at scrm.
	static const std::vector<u32> table = [] {
		const unsigned n = tms_op_desc_count();
		std::vector<u32> t(n, 0);
		for (unsigned i = 4; i < n; i++) {
			const tms57002_opdesc_t &a = tms57002_op_desc[i];
			if (!a.mn || !a.mn[0]) continue;
			for (unsigned j = i + 1; j < n; j++) {
				const tms57002_opdesc_t &b = tms57002_op_desc[j];
				if (!b.mn || !b.mn[0]) continue;
				if (a.cat != b.cat || a.id != b.id || std::strcmp(a.mn, b.mn) != 0) continue;
				u32 m = 0;
				if (a.sfai != b.sfai) m |= tms57002_device::jit_st1_sfai();
				if (a.sfao != b.sfao) m |= tms57002_device::jit_st1_sfao();
				if (a.sfma != b.sfma) m |= tms57002_device::jit_st1_sfma();
				if (a.sfmo != b.sfmo) m |= tms57002_device::jit_st1_sfmo();
				if (a.rnd  != b.rnd)  m |= tms57002_device::jit_st1_rnd_field();
				if (a.movm != b.movm) m |= tms57002_device::jit_st1_movm();
				if (a.dbp  != b.dbp)  m |= tms57002_device::jit_st1_dbp();
				t[i] |= m;
				t[j] |= m;
			}
		}
		return t;
	}();
	return op < table.size() ? table[op] : 0xffffffffu;
}

bool op_is_mode_setter(unsigned op) {
	const unsigned n = tms_op_desc_count();
	if (op < 4 || op >= n)
		return false;
	return tms57002_op_desc[op].type == 'f';
}

}  // namespace tms57002
#endif  // __x86_64__ || __aarch64__

#if defined(__x86_64__)

namespace tms57002 {

namespace {
using namespace asmjit;

// C-callable device helpers the emitted bodies invoke (mirror jit_w_* in tms57002_jit.cpp).
u32 ops_get_cmem(tms57002_device *d, unsigned a) { return d->jit_get_cmem(u8(a)); }
void ops_xm_init(tms57002_device *d) { d->jit_xm_init(); }

// Device-field offsets used across the lowerings.
struct Off {
	int st1, sti, aacc, macc, macc_r, macc_w, cmem, creg, ca, id;
	int dmem0, dmem1, ba0, ba1, xoa, xwr, xrd, so;
	Off()
		: st1(int(tms57002_device::jit_off_st1()))
		, sti(int(tms57002_device::jit_off_sti()))
		, aacc(int(tms57002_device::jit_off_aacc()))
		, macc(int(tms57002_device::jit_off_macc()))
		, macc_r(int(tms57002_device::jit_off_macc_read()))
		, macc_w(int(tms57002_device::jit_off_macc_write()))
		, cmem(int(tms57002_device::jit_off_cmem()))
		, creg(int(tms57002_device::jit_off_creg()))
		, ca(int(tms57002_device::jit_off_ca()))
		, id(int(tms57002_device::jit_off_id()))
		, dmem0(int(tms57002_device::jit_off_dmem0()))
		, dmem1(int(tms57002_device::jit_off_dmem1()))
		, ba0(int(tms57002_device::jit_off_ba0()))
		, ba1(int(tms57002_device::jit_off_ba1()))
		, xoa(int(tms57002_device::jit_off_xoa()))
		, xwr(int(tms57002_device::jit_off_xwr()))
		, xrd(int(tms57002_device::jit_off_xrd()))
		, so(int(tms57002_device::jit_off_so()))
	{}
};

// %c = get_cmem(cmode ? ca : i->param), as a fresh 32-bit reg (always the real helper -> bit-exact).
x86::Gp emit_get_cmem(x86::Compiler &cc, x86::Gp dev, const Off &o, unsigned cmode, unsigned param) {
	x86::Gp addr = cc.new_gp32();
	if (cmode)
		cc.movzx(addr, x86::byte_ptr(dev, o.ca));
	else
		cc.mov(addr, Imm(param));
	InvokeNode *g = nullptr;
	cc.invoke(Out(g), uint64_t(&ops_get_cmem), FuncSignature::build<u32, tms57002_device *, unsigned>());
	g->set_arg(0, dev);
	g->set_arg(1, addr);
	x86::Gp v = cc.new_gp32();
	g->set_ret(0, v);
	return v;
}

// dmem element index (param|id + ba{dbp}) & mask{dbp}, as a fresh 64-bit reg (for [dev+idx*4+disp]).
x86::Gp emit_d_index(x86::Compiler &cc, x86::Gp dev, const Off &o, unsigned dmode, unsigned dbp, unsigned param) {
	x86::Gp addr = cc.new_gpz();
	cc.movzx(addr, x86::byte_ptr(dev, dbp ? o.ba1 : o.ba0));
	if (dmode) {
		x86::Gp idr = cc.new_gpz();
		cc.movzx(idr, x86::byte_ptr(dev, o.id));
		cc.add(addr, idr);
	} else {
		cc.add(addr, Imm(param));
	}
	cc.and_(addr, Imm(dbp ? 0x1f : 0xff));
	return addr;
}

// %d24 = dmem{dbp}[index] (raw stored 24-bit), 32-bit reg.
x86::Gp emit_load_d24(x86::Compiler &cc, x86::Gp dev, const Off &o, unsigned dmode, unsigned dbp, unsigned param) {
	x86::Gp addr = emit_d_index(cc, dev, o, dmode, dbp, param);
	x86::Gp v = cc.new_gp32();
	cc.mov(v, x86::dword_ptr(dev, addr, 2, dbp ? o.dmem1 : o.dmem0));
	return v;
}

// %d = %d24 << 8, 32-bit reg.
x86::Gp emit_load_d(x86::Compiler &cc, x86::Gp dev, const Off &o, unsigned dmode, unsigned dbp, unsigned param) {
	x86::Gp v = emit_load_d24(cc, dev, o, dmode, dbp, param);
	cc.shl(v, Imm(8));
	return v;
}

// %sfai(dst): dst = sfai ? ((int32_t)dst) >> 1 : dst  (arithmetic >>1, in place on a 32-bit reg).
void emit_sfai(x86::Compiler &cc, x86::Gp v, unsigned sfai) {
	if (sfai)
		cc.sar(v, Imm(1));
}

// %a = aacc (sfao=0) or aacc<<7 (sfao=1), fresh 32-bit reg.
x86::Gp emit_load_a(x86::Compiler &cc, x86::Gp dev, const Off &o, unsigned sfao) {
	x86::Gp a = cc.new_gp32();
	cc.mov(a, x86::dword_ptr(dev, o.aacc));
	if (sfao)
		cc.shl(a, Imm(7));
	return a;
}

// %wa(rr): aacc = saturate(s64 rr). If rr out of int32 range: st1 |= ST1_AOV, and if ST1_AOVM clamp
// to [INT32_MIN, INT32_MAX]; then aacc = (u32)rr. (tmsinstr.lst %wa; from the reference emit_wa.)
void emit_wa(x86::Compiler &cc, x86::Gp dev, const Off &o, x86::Gp rr) {
	x86::Gp chk = cc.new_gpz();
	cc.movsxd(chk, rr.r32());
	cc.cmp(chk, rr);
	Label no_ovf = cc.new_label();
	cc.je(no_ovf);
	cc.or_(x86::dword_ptr(dev, o.st1), Imm(tms57002_device::jit_st1_aov()));
	Label no_clamp = cc.new_label();
	cc.test(x86::dword_ptr(dev, o.st1), Imm(tms57002_device::jit_st1_aovm()));
	cc.jz(no_clamp);
	x86::Gp hi = cc.new_gpz(), lo = cc.new_gpz();
	cc.mov(hi, Imm(2147483647LL));
	cc.cmp(rr, hi);
	cc.cmovg(rr, hi);
	cc.mov(lo, Imm(-2147483648LL));
	cc.cmp(rr, lo);
	cc.cmovl(rr, lo);
	cc.bind(no_clamp);
	cc.bind(no_ovf);
	cc.mov(x86::dword_ptr(dev, o.aacc), rr.r32());
}

// %wd(value): dmem{dbp}[(param|id + ba{dbp}) & mask] = value & 0xffffff  (write_dmem is a masked store).
void emit_write_dmem(x86::Compiler &cc, x86::Gp dev, const Off &o, unsigned dmode, unsigned dbp, unsigned param, x86::Gp value) {
	cc.and_(value, Imm(0xffffff));
	x86::Gp addr = emit_d_index(cc, dev, o, dmode, dbp, param);
	cc.mov(x86::dword_ptr(dev, addr, 2, dbp ? o.dmem1 : o.dmem0), value);
}

// Overflow test used by the macc-output stage: set `ov` = 1 if (m & mask) is neither 0 nor mask.
void emit_ovcheck(x86::Compiler &cc, x86::Gp m, u64 mask, x86::Gp ov) {
	x86::Gp m1 = cc.new_gpz(), mk = cc.new_gpz();
	cc.mov(mk, Imm(std::int64_t(mask)));
	cc.mov(m1, m);
	cc.and_(m1, mk);
	Label no = cc.new_label();
	cc.jz(no);            // m1 == 0 -> not over
	cc.cmp(m1, mk);
	cc.je(no);            // m1 == mask -> not over
	cc.mov(ov, Imm(1));
	cc.bind(no);
}

// %mo / %mn / %mv (macc_to_output_{sfmo}[s|n] / check_macc_overflow_{sfmo}[s]). Returns the s64
// output value in a fresh reg; sets ST1_MOV and (movm=1) clamps on overflow — bit-exact with the
// interpreter's helpers. kind: 'o'=%mo, 'n'=%mn, 'v'=%mv. All constants are compile-time from
// (sfmo, movm, rnd); the op then does its own >>shift & mask on the returned value.
x86::Gp emit_macc_output(x86::Compiler &cc, x86::Gp dev, const Off &o, char kind, unsigned sfmo, unsigned movm, unsigned rnd) {
	static const u64 PRE_MASK[4] = { 0xf800000000000ULL, 0xfe00000000000ULL, 0xff80000000000ULL, 0ULL };
	static const std::int64_t ROUNDING[5] = { 0, 0x8000, 0x800000, 0x8000000, 0x80000000LL };   // [3]=1<<27 (fork RND-mode3 fix, 48-20; was 0x20000=48-30)
	static const u64 RMASK[5] = { 0xffffffffffffffffULL, 0xffffffffffff0000ULL, 0xffffffffff000000ULL, 0xfffffffff0000000ULL, 0xffffffff00000000ULL };   // [3] low 28 zeroed (48-20 fix)
	static const std::int64_t CLIP_POS[5] = { 0x00007fffffffffffLL, 0x00007fffffff0000LL, 0x00007fffff000000LL, 0x00007ffff0000000LL, 0x00007fff00000000LL };
	static const std::int64_t CLIP_NEG[5] = { std::int64_t(0xffff800000000000ULL), std::int64_t(0xffff800000000000ULL), std::int64_t(0xffff800000000000ULL), std::int64_t(0xffff800000000000ULL), std::int64_t(0xffff800000000000ULL) };
	const std::int64_t FIX_POS = 0x00007fffffffffffLL, FIX_NEG = std::int64_t(0xffff800000000000ULL);
	const u64 POST_MASK = 0xf800000000000ULL;
	if (rnd > 4) rnd = 4;

	x86::Gp mr = cc.new_gpz();   // original macc_read (for the sign test)
	cc.mov(mr, x86::qword_ptr(dev, o.macc_r));
	x86::Gp m = cc.new_gpz();
	cc.mov(m, mr);
	x86::Gp ov = cc.new_gpz();
	cc.xor_(ov, ov);

	if (kind == 'v') {
		if (sfmo != 3) {   // check_macc_overflow_3 returns macc_read unchanged
			emit_ovcheck(cc, m, PRE_MASK[sfmo], ov);
			Label noov = cc.new_label();
			cc.test(ov, ov);
			cc.jz(noov);
			cc.or_(x86::dword_ptr(dev, o.st1), Imm(tms57002_device::jit_st1_mov()));
			if (movm) {   // 's' variant: fixed clamp
				x86::Gp neg = cc.new_gpz(), mk = cc.new_gpz(), nv = cc.new_gpz();
				cc.mov(mk, Imm(std::int64_t(0x8000000000000ULL)));
				cc.mov(neg, mr); cc.and_(neg, mk);
				cc.mov(m, Imm(FIX_POS)); cc.mov(nv, Imm(FIX_NEG));
				cc.test(neg, neg); cc.cmovnz(m, nv);
			}
			cc.bind(noov);
		}
		return m;
	}

	// 'o'/'n': shift, round+mask, overflow, clamp
	if (sfmo != 3)
		emit_ovcheck(cc, m, PRE_MASK[sfmo], ov);
	if (sfmo == 1) cc.shl(m, Imm(2));
	else if (sfmo == 2) cc.shl(m, Imm(4));
	else if (sfmo == 3) cc.sar(m, Imm(8));
	if (ROUNDING[rnd]) { x86::Gp rd = cc.new_gpz(); cc.mov(rd, Imm(ROUNDING[rnd])); cc.add(m, rd); }
	{ x86::Gp rm = cc.new_gpz(); cc.mov(rm, Imm(std::int64_t(RMASK[rnd]))); cc.and_(m, rm); }
	emit_ovcheck(cc, m, POST_MASK, ov);
	Label noov = cc.new_label();
	cc.test(ov, ov);
	cc.jz(noov);
	cc.or_(x86::dword_ptr(dev, o.st1), Imm(tms57002_device::jit_st1_mov()));
	if (movm) {
		x86::Gp neg = cc.new_gpz(), mk = cc.new_gpz(), nv = cc.new_gpz();
		cc.mov(mk, Imm(std::int64_t(0x8000000000000ULL)));
		cc.mov(neg, mr); cc.and_(neg, mk);
		cc.mov(m, Imm(kind == 'o' ? CLIP_POS[rnd] : FIX_POS));
		cc.mov(nv, Imm(kind == 'o' ? CLIP_NEG[rnd] : FIX_NEG));
		cc.test(neg, neg); cc.cmovnz(m, nv);
	}
	cc.bind(noov);
	return m;
}

}  // namespace

bool emit_op_mem(x86::Compiler &cc, x86::Gp dev, tms57002_device &dsp,
                 unsigned op, unsigned param, std::uint64_t icd_addr) {
	const unsigned n = tms_op_desc_count();
	if (op < 4 || op >= n)
		return false;
	const tms57002_opdesc_t &d = tms57002_op_desc[op];
	if (!d.mn || !d.mn[0])
		return false;
	const Off o;
	const char *mn = d.mn;
	auto is = [&](const char *s) { return std::strcmp(mn, s) == 0; };

	// ---- Flag-setter ops: st1' = (st1 & keep) | set, discovered by probing the oracle.
	if (d.type == 'f') {
		const void *icd = reinterpret_cast<const void *>(icd_addr);
		const u32 v0 = dsp.jit_probe_st1(op, icd, 0x00000000u);
		const u32 vF = dsp.jit_probe_st1(op, icd, 0xffffffffu);
		const u32 setb = v0;
		const u32 keep = vF & ~setb;
		const u32 pr[2] = { 0x5a5a5a5au, 0xa5a5a5a5u };
		for (u32 p : pr)
			if (((p & keep) | setb) != dsp.jit_probe_st1(op, icd, p))
				return false;
		if (keep != 0xffffffffu)
			cc.and_(x86::dword_ptr(dev, o.st1), Imm(keep));
		if (setb != 0u)
			cc.or_(x86::dword_ptr(dev, o.st1), Imm(setb));
		return true;
	}

	// ---- Trivial register / immediate ops -------------------------------------------------
	if (is("raov")) { cc.and_(x86::dword_ptr(dev, o.st1), Imm(~tms57002_device::jit_st1_aov())); return true; }
	if (is("rmov")) { cc.and_(x86::dword_ptr(dev, o.st1), Imm(~tms57002_device::jit_st1_mov())); return true; }
	if (is("zacc")) { cc.mov(x86::dword_ptr(dev, o.aacc), Imm(0)); return true; }
	if (is("zmac")) { cc.mov(x86::qword_ptr(dev, o.macc), Imm(0)); return true; }
	if (is("lcak")) { cc.mov(x86::byte_ptr(dev, o.ca), Imm(u8(param))); return true; }
	if (is("lirk")) { cc.mov(x86::byte_ptr(dev, o.id), Imm(u8(param))); return true; }
	if (is("idle")) { cc.or_(x86::dword_ptr(dev, o.sti), Imm(tms57002_device::jit_s_idle_mask())); return true; }
	if (is("lira")) { x86::Gp a = emit_load_a(cc, dev, o, d.sfao); cc.shr(a, Imm(24)); cc.mov(x86::byte_ptr(dev, o.id), a.r8()); return true; }
	if (is("lcaa")) { x86::Gp a = emit_load_a(cc, dev, o, d.sfao); cc.shr(a, Imm(24)); cc.mov(x86::byte_ptr(dev, o.ca), a.r8()); return true; }

	// ---- Loads -----------------------------------------------------------------------------
	if (is("lacc")) {   // aacc = %c
		x86::Gp c = emit_get_cmem(cc, dev, o, d.cmode, param);
		cc.mov(x86::dword_ptr(dev, o.aacc), c);
		return true;
	}
	if (is("lacd")) {   // %sfai(d,%d); aacc = d
		x86::Gp v = emit_load_d(cc, dev, o, d.dmode, d.dbp, param);
		emit_sfai(cc, v, d.sfai);
		cc.mov(x86::dword_ptr(dev, o.aacc), v);
		return true;
	}

	// ---- Logic: and / or / xor -------------------------------------------------------------
	if (is("and") || is("or") || is("xor")) {
		// forms are contiguous per mnemonic: base+0 = %d,a ; base+1 = %c,a ; base+2 = %d,%c
		const unsigned base = is("and") ? 0x14 : is("or") ? 0x17 : 0x1a;
		const unsigned form = d.id - base;   // 0=%d,a  1=%c,a  2=%d,%c
		x86::Gp res = cc.new_gp32();
		if (form == 1) {                     // %c,a : aacc OP= %c
			res = emit_get_cmem(cc, dev, o, d.cmode, param);
			if (is("and")) cc.and_(x86::dword_ptr(dev, o.aacc), res);
			else if (is("or")) cc.or_(x86::dword_ptr(dev, o.aacc), res);
			else cc.xor_(x86::dword_ptr(dev, o.aacc), res);
		} else {
			x86::Gp dv = emit_load_d(cc, dev, o, d.dmode, d.dbp, param);   // %d
			emit_sfai(cc, dv, d.sfai);
			if (form == 0) {                 // %d,a : aacc OP= d
				if (is("and")) cc.and_(x86::dword_ptr(dev, o.aacc), dv);
				else if (is("or")) cc.or_(x86::dword_ptr(dev, o.aacc), dv);
				else cc.xor_(x86::dword_ptr(dev, o.aacc), dv);
			} else {                         // %d,%c : aacc = %c OP d
				x86::Gp c = emit_get_cmem(cc, dev, o, d.cmode, param);
				if (is("and")) cc.and_(c, dv);
				else if (is("or")) cc.or_(c, dv);
				else cc.xor_(c, dv);
				cc.mov(x86::dword_ptr(dev, o.aacc), c);
			}
		}
		return true;
	}

	// ---- Arithmetic with %wa: add / sub / neg / abs (non-%mo forms only) --------------------
	if (is("neg")) {   // %wa(-(int64)(int32)%a)
		x86::Gp a = emit_load_a(cc, dev, o, d.sfao);
		x86::Gp rr = cc.new_gpz();
		cc.movsxd(rr, a);
		cc.neg(rr);
		emit_wa(cc, dev, o, rr);
		return true;
	}
	if (is("abs")) {   // .lst: aacc = %a; if((int32)aacc<0){ aacc=-aacc; if((int32)aacc<0){ st1|=AOV; [saturate] } }
		// 2026-07-05 audit: was reading aacc raw (missed the sfao<<7 %a view + the unconditional
		// `aacc = %a` write-back) and missed the fork abs-saturate silicon fix (parity with pooled).
		x86::Gp a = emit_load_a(cc, dev, o, d.sfao);
		cc.mov(x86::dword_ptr(dev, o.aacc), a);
		Label done = cc.new_label();
		cc.test(a, a);
		cc.jns(done);
		cc.neg(a);
		cc.mov(x86::dword_ptr(dev, o.aacc), a);
		cc.test(a, a);
		cc.jns(done);
		cc.or_(x86::dword_ptr(dev, o.st1), Imm(tms57002_device::jit_st1_aov()));
		if (dsp.jit_abs_saturate())
			cc.mov(x86::dword_ptr(dev, o.aacc), Imm(0x7fffffffu));
		else {
			Label no_clamp = cc.new_label();
			cc.test(x86::dword_ptr(dev, o.st1), Imm(tms57002_device::jit_st1_aovm()));
			cc.jz(no_clamp);
			cc.mov(x86::dword_ptr(dev, o.aacc), Imm(0x7fffffffu));
			cc.bind(no_clamp);
		}
		cc.bind(done);
		return true;
	}
	if (is("add") || is("sub")) {
		// forms: 0x03/0x09 %d,a ; 0x04/0x0a %c,a ; 0x07/0x0d %d,%c ; (0x05/0x06/0x0b/0x0c use %mo -> defer)
		const unsigned lo = d.id & 0xf;
		const bool is_add = is("add");
		x86::Gp lhs = cc.new_gpz(), rhs = cc.new_gpz();
		bool ok = true;
		if ((is_add && lo == 0x03) || (!is_add && lo == 0x09)) {         // %d,a : (int32)%d OP (int32)%a
			x86::Gp dv = emit_load_d(cc, dev, o, d.dmode, d.dbp, param);
			cc.movsxd(lhs, dv);
			x86::Gp a = emit_load_a(cc, dev, o, d.sfao);
			cc.movsxd(rhs, a);
		} else if ((is_add && lo == 0x04) || (!is_add && lo == 0x0a)) {  // %c,a : (int32)%c OP (int32)%a
			x86::Gp c = emit_get_cmem(cc, dev, o, d.cmode, param);
			cc.movsxd(lhs, c);
			x86::Gp a = emit_load_a(cc, dev, o, d.sfao);
			cc.movsxd(rhs, a);
		} else if ((is_add && lo == 0x07) || (!is_add && lo == 0x0d)) {  // %d,%c : (int32)%d OP (int32)%c
			x86::Gp dv = emit_load_d(cc, dev, o, d.dmode, d.dbp, param);
			cc.movsxd(lhs, dv);
			x86::Gp c = emit_get_cmem(cc, dev, o, d.cmode, param);
			cc.movsxd(rhs, c);
		} else if ((is_add && lo == 0x05) || (!is_add && lo == 0x0b)) {  // %d,m : %sfai(d,%d) OP (%mo>>16)
			x86::Gp dv = emit_load_d(cc, dev, o, d.dmode, d.dbp, param);
			emit_sfai(cc, dv, d.sfai);
			cc.movsxd(lhs, dv);
			rhs = emit_macc_output(cc, dev, o, 'o', d.sfmo, d.movm, d.rnd);
			cc.sar(rhs, Imm(16));
		} else if ((is_add && lo == 0x06) || (!is_add && lo == 0x0c)) {  // %c,m : (int32)%c OP (%mo>>16)
			x86::Gp c = emit_get_cmem(cc, dev, o, d.cmode, param);
			cc.movsxd(lhs, c);
			rhs = emit_macc_output(cc, dev, o, 'o', d.sfmo, d.movm, d.rnd);
			cc.sar(rhs, Imm(16));
		} else {
			ok = false;
		}
		if (!ok)
			return false;
		if (is_add) cc.add(lhs, rhs); else cc.sub(lhs, rhs);
		emit_wa(cc, dev, o, lhs);
		return true;
	}

	// ---- macc shift ops: sfml / sfmr -------------------------------------------------------
	if (is("sfml") || is("sfmr")) {
		// macc = (macc & 0x8000000000000) | ((macc <</>> 1) & 0x7ffffffffffff)
		x86::Gp m = cc.new_gpz(), hi = cc.new_gpz(), mask = cc.new_gpz();
		cc.mov(m, x86::qword_ptr(dev, o.macc));
		cc.mov(hi, m);
		cc.mov(mask, Imm(0x8000000000000LL));
		cc.and_(hi, mask);
		if (is("sfml")) cc.shl(m, Imm(1)); else cc.shr(m, Imm(1));
		cc.mov(mask, Imm(0x7ffffffffffffLL));
		cc.and_(m, mask);
		cc.or_(m, hi);
		cc.mov(x86::qword_ptr(dev, o.macc), m);
		return true;
	}

	// ---- macc loads: lmhc / lmhd / lmld ----------------------------------------------------
	if (is("lmhc")) {   // macc = macc_write = ((int64)(int32)%c) << 16
		x86::Gp c = emit_get_cmem(cc, dev, o, d.cmode, param);
		x86::Gp m = cc.new_gpz();
		cc.movsxd(m, c);
		cc.shl(m, Imm(16));
		cc.mov(x86::qword_ptr(dev, o.macc), m);
		cc.mov(x86::qword_ptr(dev, o.macc_w), m);
		return true;
	}
	if (is("lmhd")) {   // macc = macc_write = ((int64)(int32)(%d)) << 16  [lmhd_d_operand = %d, forward off]
		if (dsp.jit_lmhd_forward_on())
			return false;
		x86::Gp v = emit_load_d(cc, dev, o, d.dmode, d.dbp, param);
		x86::Gp m = cc.new_gpz();
		cc.movsxd(m, v);
		cc.shl(m, Imm(16));
		cc.mov(x86::qword_ptr(dev, o.macc), m);
		cc.mov(x86::qword_ptr(dev, o.macc_w), m);
		return true;
	}
	if (is("lmld")) {   // macc = macc_write = (macc & ~0xffffff) | %d24
		x86::Gp v = emit_load_d24(cc, dev, o, d.dmode, d.dbp, param);
		x86::Gp m = cc.new_gpz(), mask = cc.new_gpz();
		cc.mov(m, x86::qword_ptr(dev, o.macc));
		cc.mov(mask, Imm(~0xffffffLL));
		cc.and_(m, mask);
		x86::Gp v64 = cc.new_gpz();
		cc.mov(v64.r32(), v);     // zero-extend %d24
		cc.or_(m, v64);
		cc.mov(x86::qword_ptr(dev, o.macc), m);
		cc.mov(x86::qword_ptr(dev, o.macc_w), m);
		return true;
	}

	// ---- mac / macs / macu / mpy / mpyu (sfma=0, forward off) -------------------------------
	if (is("mac") || is("macs") || is("macu") || is("mpy") || is("mpyu")) {
		if (d.sfma != 0)
			return false;   // %ml pre-shift (macc<<2 / <<4 / >>16) not yet emitted
		if (dsp.jit_mpy_forward_on())
			return false;   // debug forward mode -> deopt
		// c-source (0=cmem(cmode) 1=aacc 2=creg), d-source (0=dmem signed 1=dmem unsigned 2=aacc),
		// shift, accumulate, creg-write — keyed by (mnemonic, .lst id).
		int csrc, dsrc, shift;
		bool acc, cwrite;
		const unsigned id = d.id;
		if (is("mac")) {
			acc = true; cwrite = true;
			if (id == 0x24) { csrc = 0; dsrc = 0; shift = 7; }
			else if (id == 0x25) { csrc = 1; dsrc = 0; shift = 7; }
			else if (id == 0x26) { csrc = 0; dsrc = 2; shift = 15; }
			else return false;
		} else if (is("macs")) {
			acc = true; cwrite = true; csrc = 0; dsrc = 2; shift = 14;
			if (id != 0x2e) return false;
		} else if (is("macu")) {
			acc = true; cwrite = true; shift = 7;
			if (id == 0x29) { csrc = 0; dsrc = 1; }
			else if (id == 0x2a) { csrc = 1; dsrc = 1; }
			else return false;
		} else if (is("mpy")) {
			acc = false;
			if (id == 0x21) { csrc = 0; dsrc = 0; shift = 7; cwrite = true; }
			else if (id == 0x22) { csrc = 0; dsrc = 2; shift = 15; cwrite = true; }
			else if (id == 0x23) { csrc = 2; dsrc = 0; shift = 7; cwrite = false; }
			else return false;
		} else {  // mpyu
			acc = false; cwrite = true; shift = 7;
			if (id == 0x28) { csrc = 0; dsrc = 1; }
			else return false;
		}

		// The multiplier's A input is 24 bits. It receives %a only in the c,a forms (dsrc==2);
		// a,d routes %a to the full-width C input (csrc==1).
		auto mask_a = [&](x86::Gp v32) { cc.and_(v32, Imm(0xffffff00U)); };
		// c (32-bit) + optional creg write. csrc==1 receives the full %a view, including sfao<<7.
		x86::Gp c = cc.new_gp32();
		if (csrc == 0) c = emit_get_cmem(cc, dev, o, d.cmode, param);
		else if (csrc == 1) c = emit_load_a(cc, dev, o, d.sfao);
		else cc.mov(c, x86::dword_ptr(dev, o.creg));
		if (cwrite) cc.mov(x86::dword_ptr(dev, o.creg), c);

		// r = (int64)(int32)c * (int64)d   (d: signed dmem / unsigned dmem / signed %a)
		x86::Gp rc = cc.new_gpz(), rd = cc.new_gpz();
		cc.movsxd(rc, c);
		if (dsrc == 2) {
			x86::Gp av = emit_load_a(cc, dev, o, d.sfao);   // %a: sfao<<7 before the (int32) cast
			mask_a(av);                                     // ...then the 24-bit A-port truncation
			cc.movsxd(rd, av);
		} else {
			x86::Gp dv = emit_load_d24(cc, dev, o, d.dmode, d.dbp, param);
			if (dsrc == 0) {                 // signed: sign-extend bit23
				cc.shl(dv, Imm(8));
				cc.sar(dv, Imm(8));
				cc.movsxd(rd, dv);
			} else {                         // unsigned 24-bit
				cc.mov(rd.r32(), dv);        // zero-extend
			}
		}
		cc.imul(rc, rd);
		cc.sar(rc, Imm(shift));
		if (acc) {
			x86::Gp m = cc.new_gpz();
			cc.mov(m, x86::qword_ptr(dev, o.macc));
			cc.add(m, rc);
			cc.mov(x86::qword_ptr(dev, o.macc), m);
		} else {
			cc.mov(x86::qword_ptr(dev, o.macc), rc);
		}
		return true;
	}

	// ---- dmem writes: sacd / srbd (non-%mo) ------------------------------------------------
	if (is("sacd")) {   // %wd(%a >> 8)
		x86::Gp a = emit_load_a(cc, dev, o, d.sfao);
		cc.shr(a, Imm(8));
		emit_write_dmem(cc, dev, o, d.dmode, d.dbp, param, a);
		return true;
	}
	if (is("srbd")) {   // %wd(xrd)
		x86::Gp v = cc.new_gp32();
		cc.mov(v, x86::dword_ptr(dev, o.xrd));
		emit_write_dmem(cc, dev, o, d.dmode, d.dbp, param, v);
		return true;
	}

	// ---- macc-output dmem writes: smhd / smld / slmh / slml (%mo/%mn/%mv) -------------------
	if (is("smhd")) {   // %wd(((raw?%mv:%mo) >> 24) & 0xffffff)
		x86::Gp mo = emit_macc_output(cc, dev, o, dsp.jit_smhd_raw_path() ? 'v' : 'o', d.sfmo, d.movm, d.rnd);
		cc.sar(mo, Imm(24));
		emit_write_dmem(cc, dev, o, d.dmode, d.dbp, param, mo.r32());
		return true;
	}
	if (is("smld")) {   // %wd(%mv & 0xffffff) — raw MACC[23:0], independent of output mode
		// Silicon writes the raw low 24 bits. No shift or SFMO/MOVM/RND transform applies.
		x86::Gp mo = emit_macc_output(cc, dev, o, 'v', d.sfmo, d.movm, d.rnd);
		emit_write_dmem(cc, dev, o, d.dmode, d.dbp, param, mo.r32());
		return true;
	}
	if (is("slmh")) {   // %wd((%mv >> 24) & 0xffff00)
		x86::Gp mo = emit_macc_output(cc, dev, o, 'v', d.sfmo, d.movm, d.rnd);
		cc.sar(mo, Imm(24));
		cc.and_(mo, Imm(0xffff00));
		emit_write_dmem(cc, dev, o, d.dmode, d.dbp, param, mo.r32());
		return true;
	}
	if (is("slml")) {   // %wd((%mv >> 8) & 0xffffff)
		x86::Gp mo = emit_macc_output(cc, dev, o, 'v', d.sfmo, d.movm, d.rnd);
		cc.sar(mo, Imm(8));
		emit_write_dmem(cc, dev, o, d.dmode, d.dbp, param, mo.r32());
		return true;
	}

	// ---- cmem writes: sacc (%a) / smhc (%mo) -----------------------------------------------
	if (is("sacc") || is("smhc")) {
		x86::Gp v;
		if (is("sacc")) {                                 // %wc(%a)
			v = emit_load_a(cc, dev, o, d.sfao);
		} else {                                          // smhc: %wc(smhc_store_shift(%mo)) = (%mo>>16) & ~((1<<tb)-1)
			x86::Gp mo = emit_macc_output(cc, dev, o, 'o', d.sfmo, d.movm, d.rnd);
			cc.sar(mo, Imm(16));
			v = cc.new_gp32();
			cc.mov(v, mo.r32());
			if (const int tb = dsp.jit_smhc_trunc_bits())   // fork silicon fix (parity with pooled): high-bits-only store
				cc.and_(v, Imm(u32(~((1u << tb) - 1))));
		}
		if (d.cmode) {
			x86::Gp caddr = cc.new_gpz();
			cc.movzx(caddr, x86::byte_ptr(dev, o.ca));
			cc.mov(x86::dword_ptr(dev, caddr, 2, o.cmem), v);
		} else {
			cc.mov(x86::dword_ptr(dev, o.cmem + int(param) * 4), v);
		}
		return true;
	}

	// ---- XRAM arm: rde / wre ---------------------------------------------------------------
	if (is("rde") || is("wre")) {
		// if (sti & (S_READ|S_WRITE)) return;  [wre: xwr = %d24;]  xoa = %c;  xm_init();  sti |= flag;
		const u32 rw = tms57002_device::jit_s_read_mask() | tms57002_device::jit_s_write_mask();
		const u32 flag = is("wre") ? tms57002_device::jit_s_write_mask() : tms57002_device::jit_s_read_mask();
		Label done = cc.new_label();
		{ x86::Gp s = cc.new_gp32(); cc.mov(s, x86::dword_ptr(dev, o.sti)); cc.test(s, Imm(rw)); cc.jnz(done); }
		if (is("wre")) {
			x86::Gp xw = emit_load_d24(cc, dev, o, d.dmode, d.dbp, param);
			cc.mov(x86::dword_ptr(dev, o.xwr), xw);
		}
		x86::Gp c = emit_get_cmem(cc, dev, o, d.cmode, param);
		cc.mov(x86::dword_ptr(dev, o.xoa), c);
		{ InvokeNode *xi = nullptr; cc.invoke(Out(xi), uint64_t(&ops_xm_init), FuncSignature::build<void, tms57002_device *>()); xi->set_arg(0, dev); }
		cc.or_(x86::dword_ptr(dev, o.sti), Imm(flag));
		cc.bind(done);
		return true;
	}

	// ---- serial output writes: dos / domh (render path only; deopt under the serial cycle model) --
	// serial_output_write(n, v) == so[n] = v & 0xffffff  when !serial_cycle_model_enabled(). The op's
	// output lane n is baked in the opcode id (dos 0x1c..0x1f, domh 0x20..0x23 -> n = id & 3).
	if (is("dos") || is("domh")) {
		if (dsp.jit_serial_model())
			return false;   // cycle model on -> intricate phase advance -> deopt to the interpreter
		const int n = int(d.id & 3);
		x86::Gp v;
		if (is("dos")) {                                  // serial_output_write(n, %d24)
			v = emit_load_d24(cc, dev, o, d.dmode, d.dbp, param);
		} else {                                          // domh: serial_output_write(n, (%mo>>24)&0xffffff)
			x86::Gp mo = emit_macc_output(cc, dev, o, 'o', d.sfmo, d.movm, d.rnd);
			cc.sar(mo, Imm(24));
			v = cc.new_gp32();
			cc.mov(v, mo.r32());
		}
		cc.and_(v, Imm(0xffffff));
		cc.mov(x86::dword_ptr(dev, o.so + n * 4), v);
		return true;
	}

	return false;   // not yet natively emitted — the gate skips it, the dynarec deopts it
}

// ===== Milestone 2: pooled (register-resident) op bodies =========================================
// Same semantics as emit_op_mem, but macc/aacc live in PINNED registers (r.macc/r.aacc) and the
// per-instruction operand comes from r.param — so ONE body serves every PC of an op-id. Preconditions
// (jit_pooled_safe): no pending cmem update etc., so a cmem read is a direct cmem[addr] (NO C-calls).

void emit_op_pooled(x86::Assembler &a, const PoolRegs &r, tms57002_device &dsp, unsigned op, std::uint64_t icd_addr) {
	const Off o;
	const tms57002_opdesc_t &d = tms57002_op_desc[op];
	const char *mn = d.mn;
	auto is = [&](const char *s) { return std::strcmp(mn, s) == 0; };
	// caller-saved scratch (no C-calls in these bodies, so all are freely clobberable):
	const x86::Gp T0 = x86::rax, T1 = x86::rcx, T2 = x86::rdx, T3 = x86::r8, T4 = x86::r9, T5 = x86::r10;

	// element index (param|id + ba{dbp}) & mask{dbp} -> idx (64); clobbers tmp (64).
	auto dmem_index = [&](x86::Gp idx, x86::Gp tmp) {
		a.movzx(idx, x86::byte_ptr(r.dev, d.dbp ? o.ba1 : o.ba0));
		if (d.dmode) { a.movzx(tmp, x86::byte_ptr(r.dev, o.id)); a.add(idx, tmp); }
		else a.add(idx, r.param);
		a.and_(idx, Imm(d.dbp ? 0x1f : 0xff));
	};
	// %d24 = dmem{dbp}[idx] -> dst32 ; clobbers idx(64)+tmp(64).
	auto load_d24 = [&](x86::Gp dst32, x86::Gp idx, x86::Gp tmp) {
		dmem_index(idx, tmp);
		a.mov(dst32, x86::dword_ptr(r.dev, idx, 2, d.dbp ? o.dmem1 : o.dmem0));
	};
	// %c = get_cmem(cmode) [direct] -> dst32 ; clobbers tmp(64) for the ca form.
	auto cmem_into = [&](x86::Gp dst32, x86::Gp tmp) {
		if (d.cmode) { a.movzx(tmp, x86::byte_ptr(r.dev, o.ca)); a.mov(dst32, x86::dword_ptr(r.dev, tmp, 2, o.cmem)); }
		else a.mov(dst32, x86::dword_ptr(r.dev, r.param, 2, o.cmem));
		// CRM (coefficient-RAM mode) view — match current_cmem_view(): crm==1 -> &0xffff0000, crm==2 -> <<16,
		// crm==0/3 -> raw. scrm can change the mode mid-frame so read st1 at runtime. crm==0 is the common
		// case and the `and` already set ZF for it -> one taken jz and done (the cmp/je pair only runs for
		// crm!=0). Reads (%c) go through here; cmem WRITES stay raw (crm is a read view).
		a.mov(tmp.r32(), x86::dword_ptr(r.dev, o.st1));
		a.and_(tmp.r32(), Imm(tms57002_device::jit_st1_crm()));
		Label crm_16h = a.new_label(), crm_16l = a.new_label(), crm_done = a.new_label();
		a.jz(crm_done);   // crm==0 -> raw view (common case, fused with the `and`'s ZF)
		a.cmp(tmp.r32(), Imm(tms57002_device::jit_st1_crm_16h())); a.je(crm_16h);
		a.cmp(tmp.r32(), Imm(tms57002_device::jit_st1_crm_16l())); a.je(crm_16l);
		a.jmp(crm_done);   // crm==3 -> raw
		a.bind(crm_16h); a.and_(dst32, Imm(0xffff0000u)); a.jmp(crm_done);
		a.bind(crm_16l); a.shl(dst32, Imm(16));
		a.bind(crm_done);
	};
	// %a = aacc (<<7 if sfao) -> dst32.
	auto load_a = [&](x86::Gp dst32) { a.mov(dst32, r.aacc.r32()); if (d.sfao) a.shl(dst32, Imm(7)); };
	// %wa(rr64) -> aacc(pinned): int32-overflow -> ST1_AOV; AOVM -> clamp. clobbers s1,s2 (64).
	auto wa = [&](x86::Gp rr, x86::Gp s1, x86::Gp s2) {
		a.movsxd(s1, rr.r32()); a.cmp(s1, rr);
		Label noovf = a.new_label();
		a.je(noovf);
		a.or_(x86::dword_ptr(r.dev, o.st1), Imm(tms57002_device::jit_st1_aov()));
		Label noclamp = a.new_label();
		a.test(x86::dword_ptr(r.dev, o.st1), Imm(tms57002_device::jit_st1_aovm()));
		a.jz(noclamp);
		a.mov(s1, Imm(2147483647LL)); a.cmp(rr, s1); a.cmovg(rr, s1);
		a.mov(s2, Imm(-2147483648LL)); a.cmp(rr, s2); a.cmovl(rr, s2);
		a.bind(noclamp); a.bind(noovf);
		a.mov(r.aacc.r32(), rr.r32());
	};

	// %mo/%mn/%mv (macc_to_output_{sfmo}[s|n] / check_macc_overflow_{sfmo}[s]) -> result in T0 (s64).
	// Reads macc_read (pinned r.macc_r). Sets ST1_MOV and (movm) clamps on overflow. kind: 'o'/'n'/'v'.
	// Uses T0=result, T1=mr, T2=ov, T3/T4 scratch. All constants compile-time from (sfmo,movm,rnd).
	auto macc_output = [&](char kind, unsigned sfmo, unsigned movm, unsigned rnd) {
		static const u64 PRE_MASK[4] = { 0xf800000000000ULL, 0xfe00000000000ULL, 0xff80000000000ULL, 0ULL };
		static const long long ROUNDING[5] = { 0, 0x8000, 0x800000, 0x8000000, 0x80000000LL };   // [3]=1<<27 (fork RND-mode3 fix, 48-20; was 0x20000=48-30)
		static const u64 RMASK[5] = { 0xffffffffffffffffULL, 0xffffffffffff0000ULL, 0xffffffffff000000ULL, 0xfffffffff0000000ULL, 0xffffffff00000000ULL };   // [3] low 28 zeroed (48-20 fix)
		static const long long CLIP_POS[5] = { 0x00007fffffffffffLL, 0x00007fffffff0000LL, 0x00007fffff000000LL, 0x00007ffff0000000LL, 0x00007fff00000000LL };
		static const long long CLIP_NEG[5] = { (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL };
		const long long FIX_POS = 0x00007fffffffffffLL, FIX_NEG = (long long)0xffff800000000000ULL;
		const u64 POST_MASK = 0xf800000000000ULL;
		if (rnd > 4) rnd = 4;
		a.mov(T1, r.macc_r);   // mr (for the sign test)
		a.mov(T0, T1);         // m = macc_read
		a.xor_(T2, T2);        // ov = 0
		auto ovcheck = [&](u64 mask) {
			a.mov(T3, T0); a.mov(T4, Imm((long long)mask)); a.and_(T3, T4);
			Label no = a.new_label(); a.jz(no); a.cmp(T3, T4); a.je(no); a.mov(T2, Imm(1)); a.bind(no);
		};
		auto clamp = [&](bool table) {
			a.or_(x86::dword_ptr(r.dev, o.st1), Imm(tms57002_device::jit_st1_mov()));
			if (movm) {
				a.mov(T3, T1); a.mov(T4, Imm((long long)0x8000000000000ULL)); a.and_(T3, T4);   // neg = mr & topbit
				a.mov(T0, Imm(table ? CLIP_POS[rnd] : FIX_POS));
				a.mov(T4, Imm(table ? CLIP_NEG[rnd] : FIX_NEG));
				a.test(T3, T3); a.cmovnz(T0, T4);
			}
		};
		if (kind == 'v') {
			if (sfmo != 3) { ovcheck(PRE_MASK[sfmo]); Label noov = a.new_label(); a.test(T2, T2); a.jz(noov); clamp(false); a.bind(noov); }
			return;   // result in T0
		}
		if (sfmo != 3) ovcheck(PRE_MASK[sfmo]);
		if (sfmo == 1) a.shl(T0, Imm(2)); else if (sfmo == 2) a.shl(T0, Imm(4)); else if (sfmo == 3) a.sar(T0, Imm(8));
		if (ROUNDING[rnd]) { a.mov(T3, Imm(ROUNDING[rnd])); a.add(T0, T3); }
		a.mov(T3, Imm((long long)RMASK[rnd])); a.and_(T0, T3);
		ovcheck(POST_MASK);
		Label noov = a.new_label(); a.test(T2, T2); a.jz(noov); clamp(kind == 'o'); a.bind(noov);
		// result in T0
	};

	// ---- flag-setters: st1 = (st1 & keep) | set (probe the oracle) -------------------------------
	if (d.type == 'f') {
		const void *icd = reinterpret_cast<const void *>(icd_addr);
		const u32 v0 = dsp.jit_probe_st1(op, icd, 0x00000000u);
		const u32 vF = dsp.jit_probe_st1(op, icd, 0xffffffffu);
		const u32 setb = v0, keep = vF & ~setb;
		if (keep != 0xffffffffu) a.and_(x86::dword_ptr(r.dev, o.st1), Imm(keep));
		if (setb != 0u) a.or_(x86::dword_ptr(r.dev, o.st1), Imm(setb));
		return;
	}
	// ---- trivial ----
	if (is("zmac")) { a.xor_(r.macc, r.macc); return; }
	if (is("zacc")) { a.xor_(r.aacc.r32(), r.aacc.r32()); return; }
	if (is("raov")) { a.and_(x86::dword_ptr(r.dev, o.st1), Imm(~tms57002_device::jit_st1_aov())); return; }
	if (is("rmov")) { a.and_(x86::dword_ptr(r.dev, o.st1), Imm(~tms57002_device::jit_st1_mov())); return; }
	if (is("lcak")) { a.mov(x86::byte_ptr(r.dev, o.ca), r.param.r8()); return; }
	if (is("lirk")) { a.mov(x86::byte_ptr(r.dev, o.id), r.param.r8()); return; }
	if (is("idle")) { a.or_(x86::dword_ptr(r.dev, o.sti), Imm(tms57002_device::jit_s_idle_mask())); return; }
	if (is("lira")) { load_a(T0.r32()); a.shr(T0.r32(), Imm(24)); a.mov(x86::byte_ptr(r.dev, o.id), T0.r8()); return; }
	if (is("lcaa")) { load_a(T0.r32()); a.shr(T0.r32(), Imm(24)); a.mov(x86::byte_ptr(r.dev, o.ca), T0.r8()); return; }
	// ---- macc shift: sfml/sfmr ----
	if (is("sfml") || is("sfmr")) {
		a.mov(T0, r.macc); a.mov(T1, Imm(0x8000000000000LL)); a.and_(T0, T1);   // hi = macc & topbit
		if (is("sfml")) a.shl(r.macc, Imm(1)); else a.shr(r.macc, Imm(1));
		a.mov(T1, Imm(0x7ffffffffffffLL)); a.and_(r.macc, T1);
		a.or_(r.macc, T0);
		return;
	}
	// ---- loads ----
	if (is("lacc")) { cmem_into(T0.r32(), T1); a.mov(r.aacc.r32(), T0.r32()); return; }
	if (is("lacd")) { load_d24(T0.r32(), T1, T2); a.shl(T0.r32(), Imm(8)); if (d.sfai) a.sar(T0.r32(), Imm(1)); a.mov(r.aacc.r32(), T0.r32()); return; }
	// ---- logic: and/or/xor (aacc is pinned -> operate on the register) ----
	if (is("and") || is("or") || is("xor")) {
		const unsigned base = is("and") ? 0x14 : is("or") ? 0x17 : 0x1a;
		const unsigned form = d.id - base;   // 0=%d,a 1=%c,a 2=%d,%c
		auto ap = [&](const x86::Gp &dst, const x86::Gp &src) { if (is("and")) a.and_(dst, src); else if (is("or")) a.or_(dst, src); else a.xor_(dst, src); };
		if (form == 1) {                                   // %c,a : aacc OP= %c
			cmem_into(T0.r32(), T1);
			ap(r.aacc.r32(), T0.r32());
		} else {
			load_d24(T0.r32(), T1, T2); a.shl(T0.r32(), Imm(8)); if (d.sfai) a.sar(T0.r32(), Imm(1));   // %sfai(d,%d)
			if (form == 0) {                               // %d,a : aacc OP= d
				ap(r.aacc.r32(), T0.r32());
			} else {                                       // %d,%c : aacc = %c OP d
				cmem_into(T1.r32(), T3);
				ap(T1.r32(), T0.r32());
				a.mov(r.aacc.r32(), T1.r32());
			}
		}
		return;
	}
	// ---- %wa arith: neg/abs/add/sub ----
	if (is("neg")) { load_a(T0.r32()); a.movsxd(T1, T0.r32()); a.neg(T1); wa(T1, T2, T3); return; }
	if (is("abs")) {
		// .lst: `aacc = %a;` FIRST (unconditionally) — under sfao=1 aacc becomes aacc<<7 even when
		// positive. The 2026-07-05 audit fixed this body reading aacc raw (missed the sfao view).
		load_a(T0.r32());
		a.mov(r.aacc.r32(), T0.r32());
		Label done = a.new_label();
		a.test(T0.r32(), T0.r32()); a.jns(done);
		a.neg(T0.r32()); a.mov(r.aacc.r32(), T0.r32());
		a.test(T0.r32(), T0.r32()); a.jns(done);
		a.or_(x86::dword_ptr(r.dev, o.st1), Imm(tms57002_device::jit_st1_aov()));
		if (dsp.jit_abs_saturate())
			a.mov(r.aacc.r32(), Imm(0x7fffffffu));
		else {
			Label no_clamp = a.new_label();
			a.test(x86::dword_ptr(r.dev, o.st1), Imm(tms57002_device::jit_st1_aovm()));
			a.jz(no_clamp);
			a.mov(r.aacc.r32(), Imm(0x7fffffffu));
			a.bind(no_clamp);
		}
		a.bind(done);
		return;
	}
	if (is("add") || is("sub")) {
		const unsigned lo = d.id & 0xf;
		const bool is_add = is("add");
		const bool mo_form = (lo == 0x05 || lo == 0x06 || lo == 0x0b || lo == 0x0c);
		if (!mo_form) {   // (int32)X OP (int32)Y -> %wa
			if ((is_add && lo == 0x03) || (!is_add && lo == 0x09)) { load_d24(T0.r32(), T1, T2); a.shl(T0.r32(), Imm(8)); a.movsxd(T1, T0.r32()); load_a(T0.r32()); a.movsxd(T2, T0.r32()); }
			else if ((is_add && lo == 0x04) || (!is_add && lo == 0x0a)) { cmem_into(T0.r32(), T3); a.movsxd(T1, T0.r32()); load_a(T0.r32()); a.movsxd(T2, T0.r32()); }
			else /* 0x07 / 0x0d */ { load_d24(T0.r32(), T1, T2); a.shl(T0.r32(), Imm(8)); a.movsxd(T1, T0.r32()); cmem_into(T0.r32(), T3); a.movsxd(T2, T0.r32()); }
			if (is_add) a.add(T1, T2); else a.sub(T1, T2);
			wa(T1, T3, T4);
		} else {          // %d/%c , m : lhs(int32) OP (%mo>>16) -> %wa   (lhs kept in T5 across macc_output)
			const bool d_form = (lo == 0x05 || lo == 0x0b);
			if (d_form) { load_d24(T0.r32(), T1, T2); a.shl(T0.r32(), Imm(8)); if (d.sfai) a.sar(T0.r32(), Imm(1)); a.movsxd(T5, T0.r32()); }
			else { cmem_into(T0.r32(), T1); a.movsxd(T5, T0.r32()); }
			macc_output('o', d.sfmo, d.movm, d.rnd);   // -> T0
			a.sar(T0, Imm(16));
			if (is_add) a.add(T5, T0); else a.sub(T5, T0);
			wa(T5, T1, T2);
		}
		return;
	}
	// ---- mac / macs / macu / mpy / mpyu (sfma=0; cmem direct; forward modes excluded by the frame) ----
	if (is("mac") || is("macs") || is("macu") || is("mpy") || is("mpyu")) {
		int csrc, dsrc, shift; bool acc, cwrite;
		const unsigned id = d.id;
		if (is("mac")) { acc = true; cwrite = true;
			if (id == 0x24) { csrc = 0; dsrc = 0; shift = 7; } else if (id == 0x25) { csrc = 1; dsrc = 0; shift = 7; } else { csrc = 0; dsrc = 2; shift = 15; } }
		else if (is("macs")) { acc = true; cwrite = true; csrc = 0; dsrc = 2; shift = 14; }
		else if (is("macu")) { acc = true; cwrite = true; shift = 7; if (id == 0x29) { csrc = 0; dsrc = 1; } else { csrc = 1; dsrc = 1; } }
		else if (is("mpy")) { acc = false;
			if (id == 0x21) { csrc = 0; dsrc = 0; shift = 7; cwrite = true; } else if (id == 0x22) { csrc = 0; dsrc = 2; shift = 15; cwrite = true; } else { csrc = 2; dsrc = 0; shift = 7; cwrite = false; } }
		else { acc = false; cwrite = true; csrc = 0; dsrc = 1; shift = 7; }   // mpyu 0x28
		// The multiplier's A input is 24 bits. It receives %a only in the c,a forms (dsrc==2);
		// a,d routes %a to the full-width C input (csrc==1).
		auto mask_a = [&](x86::Gp v32) { a.and_(v32, Imm(0xffffff00U)); };
		// c (T0.r32) + optional creg write. csrc==1 receives the full %a view, including sfao<<7.
		if (csrc == 0) cmem_into(T0.r32(), T1);
		else if (csrc == 1) load_a(T0.r32());
		else a.mov(T0.r32(), x86::dword_ptr(r.dev, o.creg));
		if (cwrite) a.mov(x86::dword_ptr(r.dev, o.creg), T0.r32());
		// r = (int64)(int32)c * (int64)d  -> T2 * T3
		a.movsxd(T2, T0.r32());
		if (dsrc == 2) {
			load_a(T4.r32());          // d-side %a: sfao<<7 applies before the (int32) cast
			mask_a(T4.r32());          // ...then the 24-bit A-port truncation, before the cast
			a.movsxd(T3, T4.r32());
		} else {
			load_d24(T4.r32(), T1, T5);
			if (dsrc == 0) { a.shl(T4.r32(), Imm(8)); a.sar(T4.r32(), Imm(8)); a.movsxd(T3, T4.r32()); }   // signed 24
			else a.mov(T3.r32(), T4.r32());                                                                 // unsigned 24 (zero-ext)
		}
		a.imul(T2, T3);
		a.sar(T2, Imm(shift));
		if (acc) a.add(r.macc, T2); else a.mov(r.macc, T2);
		return;
	}
	// ---- macc loads: lmhc / lmhd / lmld (write macc AND macc_write pinned) ----
	if (is("lmhc")) { cmem_into(T0.r32(), T1); a.movsxd(T2, T0.r32()); a.shl(T2, Imm(16)); a.mov(r.macc, T2); a.mov(r.macc_w, T2); return; }
	if (is("lmhd")) { load_d24(T0.r32(), T1, T2); a.shl(T0.r32(), Imm(8)); a.movsxd(T2, T0.r32()); a.shl(T2, Imm(16)); a.mov(r.macc, T2); a.mov(r.macc_w, T2); return; }
	if (is("lmld")) { load_d24(T0.r32(), T1, T2); a.mov(T1, r.macc); a.mov(T2, Imm((long long)~0xffffffLL)); a.and_(T1, T2); a.mov(T3.r32(), T0.r32()); a.or_(T1, T3); a.mov(r.macc, T1); a.mov(r.macc_w, T1); return; }
	// ---- macc-output writes: smhc (%wc) / smhd,smld,slmh,slml (%wd), reading macc_read via %mo/%mn/%mv ----
	if (is("smhc")) {
		macc_output('o', d.sfmo, d.movm, d.rnd);
		a.sar(T0, Imm(16));
		if (const int tb = dsp.jit_smhc_trunc_bits())   // fork silicon fix: smhc stores only the high (32-tb) bits
			a.and_(T0.r32(), Imm(u32(~((1u << tb) - 1))));
		if (d.cmode) { a.movzx(T1, x86::byte_ptr(r.dev, o.ca)); a.mov(x86::dword_ptr(r.dev, T1, 2, o.cmem), T0.r32()); }
		else a.mov(x86::dword_ptr(r.dev, r.param, 2, o.cmem), T0.r32());
		return;
	}
	if (is("smhd") || is("smld") || is("slmh") || is("slml")) {
		char kind; int sh; unsigned mask;
		if (is("smhd")) { kind = dsp.jit_smhd_raw_path() ? 'v' : 'o'; sh = 24; mask = 0xffffff; }
		// SMLD stores raw MACC[23:0]; it is independent of SFMO/MOVM/RND.
		else if (is("smld")) { kind = 'v'; sh = 0; mask = 0xffffff; }
		else if (is("slmh")) { kind = 'v'; sh = 24; mask = 0xffff00; }
		else { kind = 'v'; sh = 8; mask = 0xffffff; }   // slml
		macc_output(kind, d.sfmo, d.movm, d.rnd);
		if (sh)
			a.sar(T0, Imm(sh));
		a.and_(T0.r32(), Imm(mask));
		dmem_index(T1, T2);
		a.mov(x86::dword_ptr(r.dev, T1, 2, d.dbp ? o.dmem1 : o.dmem0), T0.r32());
		return;
	}
	// ---- writes ----
	if (is("sacc")) {
		load_a(T0.r32());
		if (d.cmode) { a.movzx(T1, x86::byte_ptr(r.dev, o.ca)); a.mov(x86::dword_ptr(r.dev, T1, 2, o.cmem), T0.r32()); }
		else a.mov(x86::dword_ptr(r.dev, r.param, 2, o.cmem), T0.r32());
		return;
	}
	if (is("sacd")) { load_a(T0.r32()); a.shr(T0.r32(), Imm(8)); a.and_(T0.r32(), Imm(0xffffff)); dmem_index(T1, T2); a.mov(x86::dword_ptr(r.dev, T1, 2, d.dbp ? o.dmem1 : o.dmem0), T0.r32()); return; }
	if (is("srbd")) { a.mov(T0.r32(), x86::dword_ptr(r.dev, o.xrd)); a.and_(T0.r32(), Imm(0xffffff)); dmem_index(T1, T2); a.mov(x86::dword_ptr(r.dev, T1, 2, d.dbp ? o.dmem1 : o.dmem0), T0.r32()); return; }
	// ---- XRAM arm: rde / wre (.lst: early-out if a transaction is in flight; wre stores %d24 to xwr;
	// xoa = %c; xm_init(); sti |= S_READ/S_WRITE). Same order as emit_op_mem (audited). xm_init is the
	// ONE pool-body C call: a body is entered via `call` from the driver (rsp ≡ 8 mod 16 here), so the
	// call site is realigned with sub/add rsp,8. xm_init touches only xm_adr/txrd/xm_fetches/xm_cycles
	// (no macc/aacc/pc), caller-saved scratch (incl. r11) is dead at that point, pinned regs survive.
	if (is("rde") || is("wre")) {
		const u32 rw = tms57002_device::jit_s_read_mask() | tms57002_device::jit_s_write_mask();
		const u32 flag = is("wre") ? tms57002_device::jit_s_write_mask() : tms57002_device::jit_s_read_mask();
		Label armed = a.new_label();
		a.mov(T0.r32(), x86::dword_ptr(r.dev, o.sti));
		a.test(T0.r32(), Imm(rw));
		a.jnz(armed);   // transaction in flight -> the op is a no-op
		if (is("wre")) { load_d24(T0.r32(), T1, T2); a.mov(x86::dword_ptr(r.dev, o.xwr), T0.r32()); }
		cmem_into(T0.r32(), T1);
		a.mov(x86::dword_ptr(r.dev, o.xoa), T0.r32());
		a.mov(x86::rdi, r.dev);
		a.sub(x86::rsp, Imm(8));
		a.mov(x86::rax, Imm(uint64_t(&ops_xm_init)));
		a.call(x86::rax);
		a.add(x86::rsp, Imm(8));
		a.or_(x86::dword_ptr(r.dev, o.sti), Imm(flag));
		a.bind(armed);
		return;
	}
	// ---- serial output: dos (so[n]=%d24) / domh (so[n]=(%mo>>24)&0xffffff). n = opcode id & 3. ----
	if (is("dos")) { const int n = int(d.id & 3); load_d24(T0.r32(), T1, T2); a.and_(T0.r32(), Imm(0xffffff)); a.mov(x86::dword_ptr(r.dev, o.so + n * 4), T0.r32()); return; }
	if (is("domh")) { const int n = int(d.id & 3); macc_output('o', d.sfmo, d.movm, d.rnd); a.sar(T0, Imm(24)); a.and_(T0.r32(), Imm(0xffffff)); a.mov(x86::dword_ptr(r.dev, o.so + n * 4), T0.r32()); return; }
}

}  // namespace tms57002

#endif  // __x86_64__

// ===== aarch64 backend: pooled op bodies (2026-07-07 port) =======================================
// Same semantics as the x86 emit_op_pooled above, translated to a64 under the PoolRegs convention in
// tms57002_ops.h (DEV=x19, MACC=x20, MACR=x21, MACW=x22, AACC=x23/w23, PARAM=x15/w15; scratch
// x9-x14 = T0-T5, x8 = helper-internal). Translation rules applied throughout:
//  - x86 implicit-flag idioms (`and_`+`jz`, `test`+`jns/jnz`) become explicit `ands`/`cbz`/`tbz`.
//  - `cmov*` becomes `cmp`+`csel`.
//  - Computed masks/constants are materialized with `a.mov(reg, Imm)` (asmjit synthesizes up to 4
//    insns) + register-form ALU, never trusted to logical-immediate encodability.
//  - RMW on device fields (x86 `or_ [mem], imm`) becomes ldr/orr/str through scratch.
// Coverage is op_poolable above (shared). Bit-exactness arbiter: scripts/korgprophecy_pf4_gate.sh.
#if defined(__aarch64__)

namespace tms57002 {

namespace {
using namespace asmjit;

// Device-field offsets used across the lowerings (same accessor set as the x86 backend).
struct OffA64 {
	int st1, sti, aacc, macc, macc_r, macc_w, cmem, creg, ca, id;
	int dmem0, dmem1, ba0, ba1, xoa, xwr, xrd, so;
	OffA64()
		: st1(int(tms57002_device::jit_off_st1()))
		, sti(int(tms57002_device::jit_off_sti()))
		, aacc(int(tms57002_device::jit_off_aacc()))
		, macc(int(tms57002_device::jit_off_macc()))
		, macc_r(int(tms57002_device::jit_off_macc_read()))
		, macc_w(int(tms57002_device::jit_off_macc_write()))
		, cmem(int(tms57002_device::jit_off_cmem()))
		, creg(int(tms57002_device::jit_off_creg()))
		, ca(int(tms57002_device::jit_off_ca()))
		, id(int(tms57002_device::jit_off_id()))
		, dmem0(int(tms57002_device::jit_off_dmem0()))
		, dmem1(int(tms57002_device::jit_off_dmem1()))
		, ba0(int(tms57002_device::jit_off_ba0()))
		, ba1(int(tms57002_device::jit_off_ba1()))
		, xoa(int(tms57002_device::jit_off_xoa()))
		, xwr(int(tms57002_device::jit_off_xwr()))
		, xrd(int(tms57002_device::jit_off_xrd()))
		, so(int(tms57002_device::jit_off_so()))
	{}
};

}  // namespace

void emit_op_pooled(a64::Assembler &a, const PoolRegs &r, tms57002_device &dsp, unsigned op, std::uint64_t icd_addr) {
	const OffA64 o;
	const tms57002_opdesc_t &d = tms57002_op_desc[op];
	const char *mn = d.mn;
	auto is = [&](const char *s) { return std::strcmp(mn, s) == 0; };
	// caller-saved scratch (no C-calls in these bodies, so all are freely clobberable):
	const a64::Gp T0 = a64::x9, T1 = a64::x10, T2 = a64::x11, T3 = a64::x12, T4 = a64::x13, T5 = a64::x14;
	const a64::Gp H = a64::x8;   // helper-internal (address materialization); never a value carrier

	// H = dev + off (address of a device array base). off is a compile-time struct offset.
	auto lea_dev = [&](int off) {
		if (off >= 0 && off <= 4095) a.add(H, r.dev, Imm(off));
		else { a.mov(H, Imm(off)); a.add(H, r.dev, H); }
	};
	// 32/64/8-bit device-field load/store. Direct [DEV, #off] when the scaled-imm12 form encodes;
	// otherwise via H. Never pass H/w8 as the value register.
	auto ld32 = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 16380 && !(off & 3)) a.ldr(w.w(), a64::ptr(r.dev, off));
		else { lea_dev(off); a.ldr(w.w(), a64::ptr(H)); }
	};
	auto st32 = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 16380 && !(off & 3)) a.str(w.w(), a64::ptr(r.dev, off));
		else { lea_dev(off); a.str(w.w(), a64::ptr(H)); }
	};
	auto ld64 = [&](a64::Gp x, int off) {
		if (off >= 0 && off <= 32760 && !(off & 7)) a.ldr(x.x(), a64::ptr(r.dev, off));
		else { lea_dev(off); a.ldr(x.x(), a64::ptr(H)); }
	};
	auto st64 = [&](a64::Gp x, int off) {
		if (off >= 0 && off <= 32760 && !(off & 7)) a.str(x.x(), a64::ptr(r.dev, off));
		else { lea_dev(off); a.str(x.x(), a64::ptr(H)); }
	};
	auto ldb = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 4095) a.ldrb(w.w(), a64::ptr(r.dev, off));
		else { lea_dev(off); a.ldrb(w.w(), a64::ptr(H)); }
	};
	auto stb = [&](a64::Gp w, int off) {
		if (off >= 0 && off <= 4095) a.strb(w.w(), a64::ptr(r.dev, off));
		else { lea_dev(off); a.strb(w.w(), a64::ptr(H)); }
	};
	// st1 |= bits / st1 = (st1 & keep) | set — RMW through s (value) + s2 (mask). Both clobbered.
	auto st1_rmw_or = [&](u32 bits, a64::Gp s, a64::Gp s2) {
		ld32(s, o.st1);
		a.mov(s2.w(), Imm(bits));
		a.orr(s.w(), s.w(), s2.w());
		st32(s, o.st1);
	};

	// element index (param|id + ba{dbp}) & mask{dbp} -> idx (64); clobbers tmp (64).
	auto dmem_index = [&](a64::Gp idx, a64::Gp tmp) {
		ldb(idx, d.dbp ? o.ba1 : o.ba0);
		if (d.dmode) { ldb(tmp, o.id); a.add(idx.x(), idx.x(), tmp.x()); }
		else a.add(idx.x(), idx.x(), r.param.x());
		a.and_(idx.x(), idx.x(), Imm(d.dbp ? 0x1f : 0xff));
	};
	// %d24 = dmem{dbp}[idx] -> dst32 ; clobbers idx(64)+tmp(64)+H.
	auto load_d24 = [&](a64::Gp dst, a64::Gp idx, a64::Gp tmp) {
		dmem_index(idx, tmp);
		lea_dev(d.dbp ? o.dmem1 : o.dmem0);
		a.ldr(dst.w(), a64::ptr(H, idx.x(), a64::lsl(2)));
	};
	// %c = cmem[cmode ? ca : param] (direct; jit_pooled_safe holds) -> dst32 ; clobbers tmp+H.
	// Then the CRM read view (match current_cmem_view(); scrm can flip it mid-frame -> runtime check).
	auto cmem_into = [&](a64::Gp dst, a64::Gp tmp) {
		if (d.cmode) {
			ldb(tmp, o.ca);
			lea_dev(o.cmem);
			a.ldr(dst.w(), a64::ptr(H, tmp.x(), a64::lsl(2)));
		} else {
			lea_dev(o.cmem);
			a.ldr(dst.w(), a64::ptr(H, r.param.x(), a64::lsl(2)));
		}
		ld32(tmp, o.st1);
		a.mov(H.w(), Imm(tms57002_device::jit_st1_crm()));
		a.and_(tmp.w(), tmp.w(), H.w());
		Label crm_16h = a.new_label(), crm_16l = a.new_label(), crm_done = a.new_label();
		a.mov(H.w(), Imm(tms57002_device::jit_st1_crm_16h()));
		a.cmp(tmp.w(), H.w());
		a.b_eq(crm_16h);
		a.mov(H.w(), Imm(tms57002_device::jit_st1_crm_16l()));
		a.cmp(tmp.w(), H.w());
		a.b_eq(crm_16l);
		a.b(crm_done);
		a.bind(crm_16h);
		a.mov(H.w(), Imm(0xffff0000u));
		a.and_(dst.w(), dst.w(), H.w());
		a.b(crm_done);
		a.bind(crm_16l);
		a.lsl(dst.w(), dst.w(), Imm(16));
		a.bind(crm_done);
	};
	// %a = aacc (<<7 if sfao) -> dst32.
	auto load_a = [&](a64::Gp dst) {
		a.mov(dst.w(), r.aacc.w());
		if (d.sfao) a.lsl(dst.w(), dst.w(), Imm(7));
	};
	// %wa(rr64) -> aacc(pinned): int32-overflow -> ST1_AOV; AOVM -> clamp. clobbers s1,s2,H.
	auto wa = [&](a64::Gp rr, a64::Gp s1, a64::Gp s2) {
		a.sxtw(s1.x(), rr.w());
		a.cmp(s1.x(), rr.x());
		Label noovf = a.new_label();
		a.b_eq(noovf);
		st1_rmw_or(tms57002_device::jit_st1_aov(), s1, s2);
		Label noclamp = a.new_label();
		ld32(s1, o.st1);
		a.mov(s2.w(), Imm(tms57002_device::jit_st1_aovm()));
		a.tst(s1.w(), s2.w());
		a.b_eq(noclamp);
		a.mov(s1.x(), Imm(2147483647LL));
		a.cmp(rr.x(), s1.x());
		a.csel(rr.x(), s1.x(), rr.x(), a64::CondCode::kGT);
		a.mov(s2.x(), Imm(-2147483648LL));
		a.cmp(rr.x(), s2.x());
		a.csel(rr.x(), s2.x(), rr.x(), a64::CondCode::kLT);
		a.bind(noclamp);
		a.bind(noovf);
		a.mov(r.aacc.w(), rr.w());
	};

	// %mo/%mn/%mv -> result in T0 (s64). Reads macc_read (pinned r.macc_r). Sets ST1_MOV and (movm)
	// clamps on overflow. kind: 'o'/'n'/'v'. Uses T0=result, T1=mr, T2=ov, T3/T4 scratch (+H via RMW).
	// Same constant tables as the x86 backend (incl. the fork RND-mode3 48-20 fix).
	auto macc_output = [&](char kind, unsigned sfmo, unsigned movm, unsigned rnd) {
		static const u64 PRE_MASK[4] = { 0xf800000000000ULL, 0xfe00000000000ULL, 0xff80000000000ULL, 0ULL };
		static const long long ROUNDING[5] = { 0, 0x8000, 0x800000, 0x8000000, 0x80000000LL };
		static const u64 RMASK[5] = { 0xffffffffffffffffULL, 0xffffffffffff0000ULL, 0xffffffffff000000ULL, 0xfffffffff0000000ULL, 0xffffffff00000000ULL };
		// MOVM clip is the 48-bit limit masked by the selected rounding mask.
		static const long long CLIP_POS[5] = { 0x00007fffffffffffLL, 0x00007fffffff0000LL, 0x00007fffff000000LL, 0x00007ffff0000000LL, 0x00007fff00000000LL };
		static const long long CLIP_NEG[5] = { (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL, (long long)0xffff800000000000ULL };
		const long long FIX_POS = 0x00007fffffffffffLL, FIX_NEG = (long long)0xffff800000000000ULL;
		const u64 POST_MASK = 0xf800000000000ULL;
		if (rnd > 4) rnd = 4;
		a.mov(T1.x(), r.macc_r.x());   // mr (for the sign test)
		a.mov(T0.x(), T1.x());         // m = macc_read
		a.mov(T2.x(), a64::xzr);       // ov = 0
		auto ovcheck = [&](u64 mask) {
			a.mov(T4.x(), Imm((long long)mask));
			a.ands(T3.x(), T0.x(), T4.x());
			Label no = a.new_label();
			a.b_eq(no);                 // (m & mask) == 0 -> not over
			a.cmp(T3.x(), T4.x());
			a.b_eq(no);                 // (m & mask) == mask -> not over
			a.mov(T2.x(), Imm(1));
			a.bind(no);
		};
		auto clamp = [&](bool table) {
			st1_rmw_or(tms57002_device::jit_st1_mov(), T3, T4);
			if (movm) {
				a.mov(T4.x(), Imm((long long)0x8000000000000ULL));
				a.and_(T3.x(), T1.x(), T4.x());   // neg = mr & topbit
				a.mov(T0.x(), Imm(table ? CLIP_POS[rnd] : FIX_POS));
				a.mov(T4.x(), Imm(table ? CLIP_NEG[rnd] : FIX_NEG));
				a.cmp(T3.x(), Imm(0));
				a.csel(T0.x(), T4.x(), T0.x(), a64::CondCode::kNE);
			}
		};
		if (kind == 'v') {
			if (sfmo != 3) {   // check_macc_overflow_3 returns macc_read unchanged
				ovcheck(PRE_MASK[sfmo]);
				Label noov = a.new_label();
				a.cbz(T2.x(), noov);
				clamp(false);
				a.bind(noov);
			}
			return;   // result in T0
		}
		if (sfmo != 3) ovcheck(PRE_MASK[sfmo]);
		if (sfmo == 1) a.lsl(T0.x(), T0.x(), Imm(2));
		else if (sfmo == 2) a.lsl(T0.x(), T0.x(), Imm(4));
		else if (sfmo == 3) a.asr(T0.x(), T0.x(), Imm(8));
		if (ROUNDING[rnd]) { a.mov(T3.x(), Imm(ROUNDING[rnd])); a.add(T0.x(), T0.x(), T3.x()); }
		a.mov(T3.x(), Imm((long long)RMASK[rnd]));
		a.and_(T0.x(), T0.x(), T3.x());
		ovcheck(POST_MASK);
		Label noov = a.new_label();
		a.cbz(T2.x(), noov);
		clamp(kind == 'o');
		a.bind(noov);
		// result in T0
	};

	// ---- flag-setters: st1 = (st1 & keep) | set (probe the oracle) -------------------------------
	if (d.type == 'f') {
		const void *icd = reinterpret_cast<const void *>(icd_addr);
		const u32 v0 = dsp.jit_probe_st1(op, icd, 0x00000000u);
		const u32 vF = dsp.jit_probe_st1(op, icd, 0xffffffffu);
		const u32 setb = v0, keep = vF & ~setb;
		if (keep == 0xffffffffu && setb == 0u)
			return;   // identity (x86 backend emits nothing here either)
		ld32(T0, o.st1);
		if (keep != 0xffffffffu) { a.mov(T1.w(), Imm(keep)); a.and_(T0.w(), T0.w(), T1.w()); }
		if (setb != 0u) { a.mov(T1.w(), Imm(setb)); a.orr(T0.w(), T0.w(), T1.w()); }
		st32(T0, o.st1);
		return;
	}
	// ---- trivial ----
	if (is("zmac")) { a.mov(r.macc.x(), a64::xzr); return; }
	if (is("zacc")) { a.mov(r.aacc.w(), a64::wzr); return; }
	if (is("lcak")) { stb(r.param, o.ca); return; }
	if (is("lirk")) { stb(r.param, o.id); return; }
	if (is("idle")) {
		ld32(T0, o.sti);
		a.mov(T1.w(), Imm(tms57002_device::jit_s_idle_mask()));
		a.orr(T0.w(), T0.w(), T1.w());
		st32(T0, o.sti);
		return;
	}
	if (is("lira")) { load_a(T0); a.lsr(T0.w(), T0.w(), Imm(24)); stb(T0, o.id); return; }
	if (is("lcaa")) { load_a(T0); a.lsr(T0.w(), T0.w(), Imm(24)); stb(T0, o.ca); return; }
	// ---- macc shift: sfml/sfmr ----
	if (is("sfml") || is("sfmr")) {
		a.mov(T1.x(), Imm(0x8000000000000LL));
		a.and_(T0.x(), r.macc.x(), T1.x());   // hi = macc & topbit
		if (is("sfml")) a.lsl(r.macc.x(), r.macc.x(), Imm(1));
		else a.lsr(r.macc.x(), r.macc.x(), Imm(1));
		a.mov(T1.x(), Imm(0x7ffffffffffffLL));
		a.and_(r.macc.x(), r.macc.x(), T1.x());
		a.orr(r.macc.x(), r.macc.x(), T0.x());
		return;
	}
	// ---- loads ----
	if (is("lacc")) { cmem_into(T0, T1); a.mov(r.aacc.w(), T0.w()); return; }
	if (is("lacd")) {
		load_d24(T0, T1, T2);
		a.lsl(T0.w(), T0.w(), Imm(8));
		if (d.sfai) a.asr(T0.w(), T0.w(), Imm(1));
		a.mov(r.aacc.w(), T0.w());
		return;
	}
	// ---- logic: and/or/xor (aacc is pinned -> operate on the register) ----
	if (is("and") || is("or") || is("xor")) {
		const unsigned base = is("and") ? 0x14 : is("or") ? 0x17 : 0x1a;
		const unsigned form = d.id - base;   // 0=%d,a 1=%c,a 2=%d,%c
		auto ap = [&](const a64::Gp &dst, const a64::Gp &src) {
			if (is("and")) a.and_(dst.w(), dst.w(), src.w());
			else if (is("or")) a.orr(dst.w(), dst.w(), src.w());
			else a.eor(dst.w(), dst.w(), src.w());
		};
		if (form == 1) {                                   // %c,a : aacc OP= %c
			cmem_into(T0, T1);
			ap(r.aacc, T0);
		} else {
			load_d24(T0, T1, T2);                          // %sfai(d,%d)
			a.lsl(T0.w(), T0.w(), Imm(8));
			if (d.sfai) a.asr(T0.w(), T0.w(), Imm(1));
			if (form == 0) {                               // %d,a : aacc OP= d
				ap(r.aacc, T0);
			} else {                                       // %d,%c : aacc = %c OP d
				cmem_into(T1, T3);
				ap(T1, T0);
				a.mov(r.aacc.w(), T1.w());
			}
		}
		return;
	}
	// ---- %wa arith: neg/abs/add/sub ----
	if (is("neg")) {
		load_a(T0);
		a.sxtw(T1.x(), T0.w());
		a.neg(T1.x(), T1.x());
		wa(T1, T2, T3);
		return;
	}
	if (is("abs")) {
		// .lst: `aacc = %a;` FIRST (unconditionally) — under sfao=1 aacc becomes aacc<<7 even when
		// positive (2026-07-05 audit; parity with the x86 body).
		load_a(T0);
		a.mov(r.aacc.w(), T0.w());
		Label done = a.new_label();
		a.tbz(T0.w(), Imm(31), done);      // %a >= 0 -> done
		a.neg(T0.w(), T0.w());
		a.mov(r.aacc.w(), T0.w());
		a.tbz(T0.w(), Imm(31), done);      // -%a >= 0 -> done
		st1_rmw_or(tms57002_device::jit_st1_aov(), T1, T2);
		if (dsp.jit_abs_saturate()) {
			a.mov(T1.w(), Imm(0x7fffffffu));
			a.mov(r.aacc.w(), T1.w());
		}
		else {
			Label no_clamp = a.new_label();
			ld32(T1, o.st1);
			a.mov(T2.w(), Imm(tms57002_device::jit_st1_aovm()));
			a.tst(T1.w(), T2.w());
			a.b_eq(no_clamp);
			a.mov(T1.w(), Imm(0x7fffffffu));
			a.mov(r.aacc.w(), T1.w());
			a.bind(no_clamp);
		}
		a.bind(done);
		return;
	}
	if (is("add") || is("sub")) {
		const unsigned lo = d.id & 0xf;
		const bool is_add = is("add");
		const bool mo_form = (lo == 0x05 || lo == 0x06 || lo == 0x0b || lo == 0x0c);
		if (!mo_form) {   // (int32)X OP (int32)Y -> %wa
			if ((is_add && lo == 0x03) || (!is_add && lo == 0x09)) {         // %d,a
				load_d24(T0, T1, T2);
				a.lsl(T0.w(), T0.w(), Imm(8));
				a.sxtw(T1.x(), T0.w());
				load_a(T0);
				a.sxtw(T2.x(), T0.w());
			} else if ((is_add && lo == 0x04) || (!is_add && lo == 0x0a)) {  // %c,a
				cmem_into(T0, T3);
				a.sxtw(T1.x(), T0.w());
				load_a(T0);
				a.sxtw(T2.x(), T0.w());
			} else {                                                          // 0x07 / 0x0d : %d,%c
				load_d24(T0, T1, T2);
				a.lsl(T0.w(), T0.w(), Imm(8));
				a.sxtw(T1.x(), T0.w());
				cmem_into(T0, T3);
				a.sxtw(T2.x(), T0.w());
			}
			if (is_add) a.add(T1.x(), T1.x(), T2.x());
			else a.sub(T1.x(), T1.x(), T2.x());
			wa(T1, T3, T4);
		} else {          // %d/%c , m : lhs(int32) OP (%mo>>16) -> %wa  (lhs kept in T5 across macc_output)
			const bool d_form = (lo == 0x05 || lo == 0x0b);
			if (d_form) {
				load_d24(T0, T1, T2);
				a.lsl(T0.w(), T0.w(), Imm(8));
				if (d.sfai) a.asr(T0.w(), T0.w(), Imm(1));
				a.sxtw(T5.x(), T0.w());
			} else {
				cmem_into(T0, T1);
				a.sxtw(T5.x(), T0.w());
			}
			macc_output('o', d.sfmo, d.movm, d.rnd);   // -> T0
			a.asr(T0.x(), T0.x(), Imm(16));
			if (is_add) a.add(T5.x(), T5.x(), T0.x());
			else a.sub(T5.x(), T5.x(), T0.x());
			wa(T5, T1, T2);
		}
		return;
	}
	// ---- mac / macs / macu / mpy / mpyu (sfma=0; cmem direct; forward modes excluded by the frame) ----
	if (is("mac") || is("macs") || is("macu") || is("mpy") || is("mpyu")) {
		int csrc, dsrc, shift; bool acc, cwrite;
		const unsigned id = d.id;
		if (is("mac")) { acc = true; cwrite = true;
			if (id == 0x24) { csrc = 0; dsrc = 0; shift = 7; } else if (id == 0x25) { csrc = 1; dsrc = 0; shift = 7; } else { csrc = 0; dsrc = 2; shift = 15; } }
		else if (is("macs")) { acc = true; cwrite = true; csrc = 0; dsrc = 2; shift = 14; }
		else if (is("macu")) { acc = true; cwrite = true; shift = 7; if (id == 0x29) { csrc = 0; dsrc = 1; } else { csrc = 1; dsrc = 1; } }
		else if (is("mpy")) { acc = false;
			if (id == 0x21) { csrc = 0; dsrc = 0; shift = 7; cwrite = true; } else if (id == 0x22) { csrc = 0; dsrc = 2; shift = 15; cwrite = true; } else { csrc = 2; dsrc = 0; shift = 7; cwrite = false; } }
		else { acc = false; cwrite = true; csrc = 0; dsrc = 1; shift = 7; }   // mpyu 0x28
		// The multiplier's A input is 24 bits. It receives %a only in the c,a forms (dsrc==2);
		// a,d routes %a to the full-width C input (csrc==1).
		auto mask_a = [&](a64::Gp v) {
			// and(w, w, imm) with 0xffffff00 IS encodable as an AArch64 logical-immediate (a rotated run of
			// ones), but go through H to stay safe from asmjit silently dropping a non-encodable imm.
			a.mov(H.w(), Imm(0xffffff00U));
			a.and_(v.w(), v.w(), H.w());
		};
		// c (T0.w) + optional creg write. csrc==1 receives the full %a view, including sfao<<7.
		if (csrc == 0) cmem_into(T0, T1);
		else if (csrc == 1) load_a(T0);
		else ld32(T0, o.creg);
		if (cwrite) st32(T0, o.creg);
		// r = (int64)(int32)c * (int64)d  -> T2 * T3
		a.sxtw(T2.x(), T0.w());
		if (dsrc == 2) {
			load_a(T4);                  // d-side %a: sfao<<7 applies before the (int32) cast
			mask_a(T4);                  // ...then the 24-bit A-port truncation, before the cast
			a.sxtw(T3.x(), T4.w());
		} else {
			load_d24(T4, T1, T5);
			if (dsrc == 0) {             // signed 24-bit: sign-extend bit23
				a.lsl(T4.w(), T4.w(), Imm(8));
				a.asr(T4.w(), T4.w(), Imm(8));
				a.sxtw(T3.x(), T4.w());
			} else {                     // unsigned 24-bit (zero-extend)
				a.mov(T3.w(), T4.w());
			}
		}
		a.mul(T2.x(), T2.x(), T3.x());
		a.asr(T2.x(), T2.x(), Imm(shift));
		if (acc) a.add(r.macc.x(), r.macc.x(), T2.x());
		else a.mov(r.macc.x(), T2.x());
		return;
	}
	// ---- macc loads: lmhc / lmhd / lmld (write macc AND macc_write pinned) ----
	if (is("lmhc")) {
		cmem_into(T0, T1);
		a.sxtw(T2.x(), T0.w());
		a.lsl(T2.x(), T2.x(), Imm(16));
		a.mov(r.macc.x(), T2.x());
		a.mov(r.macc_w.x(), T2.x());
		return;
	}
	if (is("lmhd")) {
		load_d24(T0, T1, T2);
		a.lsl(T0.w(), T0.w(), Imm(8));
		a.sxtw(T2.x(), T0.w());
		a.lsl(T2.x(), T2.x(), Imm(16));
		a.mov(r.macc.x(), T2.x());
		a.mov(r.macc_w.x(), T2.x());
		return;
	}
	if (is("lmld")) {
		load_d24(T0, T1, T2);
		a.mov(T2.x(), Imm((long long)~0xffffffLL));
		a.and_(T1.x(), r.macc.x(), T2.x());
		a.mov(T3.w(), T0.w());     // zero-extend %d24
		a.orr(T1.x(), T1.x(), T3.x());
		a.mov(r.macc.x(), T1.x());
		a.mov(r.macc_w.x(), T1.x());
		return;
	}
	// ---- macc-output writes: smhc (%wc) / smhd,smld,slmh,slml (%wd), via %mo/%mn/%mv ----
	if (is("smhc")) {
		macc_output('o', d.sfmo, d.movm, d.rnd);
		a.asr(T0.x(), T0.x(), Imm(16));
		if (const int tb = dsp.jit_smhc_trunc_bits()) {   // fork silicon fix: high-bits-only store
			a.mov(T1.w(), Imm(u32(~((1u << tb) - 1))));
			a.and_(T0.w(), T0.w(), T1.w());
		}
		if (d.cmode) {
			ldb(T1, o.ca);
			lea_dev(o.cmem);
			a.str(T0.w(), a64::ptr(H, T1.x(), a64::lsl(2)));
		} else {
			lea_dev(o.cmem);
			a.str(T0.w(), a64::ptr(H, r.param.x(), a64::lsl(2)));
		}
		return;
	}
	if (is("smhd") || is("smld") || is("slmh") || is("slml")) {
		char kind; int sh; unsigned mask;
		if (is("smhd")) { kind = dsp.jit_smhd_raw_path() ? 'v' : 'o'; sh = 24; mask = 0xffffff; }
		// smld stores MACC[23:0] — NO shift. 44e255ea05e fixed the interpreter's spurious >>8 (which
		// wrote MACC[31:8]); this emitter kept the stale shift and so diverged. sh=0 = parity.
		else if (is("smld")) { kind = 'v'; sh = 0; mask = 0xffffff; }
		else if (is("slmh")) { kind = 'v'; sh = 24; mask = 0xffff00; }
		else { kind = 'v'; sh = 8; mask = 0xffffff; }   // slml
		macc_output(kind, d.sfmo, d.movm, d.rnd);
		if (sh) a.asr(T0.x(), T0.x(), Imm(sh));
		a.mov(T3.w(), Imm(mask));
		a.and_(T0.w(), T0.w(), T3.w());
		dmem_index(T1, T2);
		lea_dev(d.dbp ? o.dmem1 : o.dmem0);
		a.str(T0.w(), a64::ptr(H, T1.x(), a64::lsl(2)));
		return;
	}
	// ---- writes ----
	if (is("sacc")) {
		load_a(T0);
		if (d.cmode) {
			ldb(T1, o.ca);
			lea_dev(o.cmem);
			a.str(T0.w(), a64::ptr(H, T1.x(), a64::lsl(2)));
		} else {
			lea_dev(o.cmem);
			a.str(T0.w(), a64::ptr(H, r.param.x(), a64::lsl(2)));
		}
		return;
	}
	if (is("sacd")) {
		load_a(T0);
		a.lsr(T0.w(), T0.w(), Imm(8));
		a.mov(T3.w(), Imm(0xffffffu));
		a.and_(T0.w(), T0.w(), T3.w());
		dmem_index(T1, T2);
		lea_dev(d.dbp ? o.dmem1 : o.dmem0);
		a.str(T0.w(), a64::ptr(H, T1.x(), a64::lsl(2)));
		return;
	}
	if (is("srbd")) {
		ld32(T0, o.xrd);
		a.mov(T3.w(), Imm(0xffffffu));
		a.and_(T0.w(), T0.w(), T3.w());
		dmem_index(T1, T2);
		lea_dev(d.dbp ? o.dmem1 : o.dmem0);
		a.str(T0.w(), a64::ptr(H, T1.x(), a64::lsl(2)));
		return;
	}
	// ---- serial output: dos (so[n]=%d24) / domh (so[n]=(%mo>>24)&0xffffff). n = opcode id & 3. ----
	if (is("dos")) {
		const int n = int(d.id & 3);
		load_d24(T0, T1, T2);
		a.mov(T3.w(), Imm(0xffffffu));
		a.and_(T0.w(), T0.w(), T3.w());
		st32(T0, o.so + n * 4);
		return;
	}
	if (is("domh")) {
		const int n = int(d.id & 3);
		macc_output('o', d.sfmo, d.movm, d.rnd);
		a.asr(T0.x(), T0.x(), Imm(24));
		a.mov(T3.w(), Imm(0xffffffu));
		a.and_(T0.w(), T0.w(), T3.w());
		st32(T0, o.so + n * 4);
		return;
	}
	// (unused in pooled bodies but kept referenced for parity notes)
	(void)o.xoa; (void)o.xwr; (void)ld64; (void)st64;
}

}  // namespace tms57002

#endif  // __aarch64__
