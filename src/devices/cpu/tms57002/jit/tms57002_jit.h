// license:BSD-3-Clause
//
// TMS57002 JIT (asmjit). The verbatim interpreter stays the default execution path and
// the bit-exact oracle (tms57002_jit_test); this compiles a loaded program to native
// code, op by op, falling back to a per-op interpreter trampoline (jit_step_one) for
// anything not yet emitted. asmjit is zlib-licensed (BSD-compatible).
//
// Phases: 0 = interpreter-seam harness (run_sample). Foundation = a real asmjit-
// compiled frame runner that drives the program by CALLing the trampoline
// (run_sample_compiled); those calls become native code per the 41-form coverage list.
#ifndef MAME_CPU_TMS57002_JIT_TMS57002_JIT_H
#define MAME_CPU_TMS57002_JIT_TMS57002_JIT_H

#include "emu.h"        // runtime/compat typedefs (u8, u32, ...) — tms57002.h needs them
#include "tms57002.h"   // tms57002_device

#include <asmjit/core.h>   // JitRuntime, CodeHolder, Environment (arch backend in the .cpp)

#include <array>
#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

namespace tms57002 {

class Jit {
public:
	Jit() = default;

	// Phase 0: drive one sample frame one instruction at a time through the
	// interpreter (the plain-C++ seam). Bit-exact with debug_run_sample_frame.
	std::array<u32, 4> run_sample(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps = 4096);

	// Foundation: same frame, but executed by a real asmjit-compiled function that
	// loops calling the per-op interpreter trampoline until idle or max_steps. Bit-
	// exact with run_sample; the per-op CALLs get replaced by native codegen later.
	// Falls back to run_sample() if codegen fails.
	std::array<u32, 4> run_sample_compiled(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps = 4096);

	// Native per-PC JIT: each PC's chain is compiled to a native function that calls
	// the decomposed seam (jit_pc_pre / jit_op_exec per micro-op / jit_pc_post) with the
	// chain UNROLLED at compile time — no runtime chain-walk. The frame loop calls the
	// compiled per-PC function for each chain head (compiled lazily, cached by ipc).
	// Bit-exact with the interpreter; falls back to run_sample() if codegen fails.
	std::array<u32, 4> run_sample_native(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps = 4096);

	// Whole-frame JIT: the ENTIRE frame compiled to ONE native function with every PC's body
	// inlined (no per-PC function calls — the overhead that made run_sample_native 0.41x). The
	// linear PC sequence is unrolled at compile time; a per-PC safety check falls back to the
	// interpreter for the rest of the frame if the program branches/repeats. Bit-exact.
	std::array<u32, 4> run_sample_frame_native(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps = 4096);

	// Live-machine entry: run a frame that the CALLER already began (route_dsp_serial ->
	// debug_begin_sample_frame) and will read serial_output from afterwards. Same as
	// run_sample_frame_native minus the begin + the end-return — so the machine keeps its exact
	// frame boundary and only the DSP execution becomes native. Bit-exact with the interpreter.
	void run_begun_frame(tms57002_device &dsp, int max_steps = 4096);

	// M5: the LIVE seam for the M1-M4 pooled dynarec. For a frame the caller ALREADY began
	// (route_dsp_serial -> debug_begin_sample_frame); the caller reads serial_output() afterwards — so NO
	// jit_begin_frame (double-begin) and NO end-return. Runs the pooled compiled frame natively; its only
	// interpreter re-entry is the compiled frame's MID-FRAME fallback (jit_w_run_rest -> jit_run_chain
	// loop under jit_running()'s icount/idle contract — timeslice-correct; test it with
	// KPROP_PF4_FORCE_MIDFRAME_FALLBACK=<pc>). [Historical note, corrected 2026-07-05: an earlier comment
	// here claimed jit_run_chain "derails the live H8" — that derailment predates 71cdb19 and was the
	// icount-override bug fixed there, not jit_run_chain itself.] Returns TRUE if it ran the frame;
	// FALSE means the caller must run the frame via the device's own rt_run (the pf=2 path) — used for the
	// fallback (jit_pooled_safe false, e.g. serial cycle model), a failed compile, AND the compile frame
	// itself (which builds the order + compiles, then lets the caller rt_run this sample; the compiled
	// frame drives from the next sample on). Keeping every interpret path in the caller's rt_run makes them
	// byte-identical to pf=2 (rt_run's cycle accounting feeds machine().time(), which the serial model
	// reads). Carries the M4 program-hash cache (reuse across PLOAD reloads).
	bool run_begun_frame_pooled(tms57002_device &dsp, int max_steps = 4096);

	// Milestone 2: the fusion-viable resident frame. Same register-resident execution as
	// run_sample_frame_native, but built with a hand-written x86-64 calling convention (pinned
	// macc/macc_read/macc_write/aacc in callee-saved regs) so op bodies can be a SHARED POOL called
	// per-PC instead of INLINED per-PC — keeping the code L1I-resident when 3 programs fuse (the
	// unrolled form is ~122KB/program -> ~374KB x3, which overflows cache and collapses). Bit-exact.
	std::array<u32, 4> run_sample_frame_pooled(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps = 4096);

	// VALIDATION: a fully CALL-FREE, register-cached frame for an all-nop program (machinery only —
	// MACC pipeline + pc/icount in registers, ZERO calls on the hot path). Tests whether call-free
	// register-resident machinery beats the clang-O2 interpreter (the step-function hypothesis).
	// Restricted to nop programs (returns to the interpreter via run_sample if any real op appears).
	std::array<u32, 4> run_sample_frame_callfree(tms57002_device &dsp, const std::array<u32, 4> &in, int max_steps = 4096);

	// Phase 0 toolchain check: construct a JitRuntime + report host arch id.
	static int asmjit_self_check();

	// Compiled-code function-pointer types (public: compile_fused hands FrameFn entry points to callers).
	using SampleFn = void (*)(tms57002_device *, int);
	using PcFn = int (*)(tms57002_device *);       // run one PC's chain, return next ipc
	using FrameFn = void (*)(tms57002_device *);   // run the whole frame

	// M3 instrumentation: proof that run_sample_frame_pooled actually executes the COMPILED pooled frame
	// (native branches) rather than silently falling back to the native frame. The DSP2/DSP3 gate asserts
	// pooled_runs() > 0 and pooled_fallbacks() == 0 so a regression to fallback can't pass unnoticed.
	long pooled_runs() const { return m_pooled_runs; }
	long pooled_fallbacks() const { return m_pooled_fallbacks; }
	long pooled_compiles() const { return m_pooled_compiles; }   // M4: distinct compiles (reuse doesn't bump)
	bool pooled_compiled() const { return m_active != nullptr && m_active->fn != nullptr && !m_active->failed; }
	bool pooled_resume_pending() const { return m_pooled_partial; }   // a partial frame is waiting to resume mid-PC

	// The program-order (pc, chain-head ipc) list built for the last pooled compile of `dsp` (needed by
	// compile_fused to re-emit the same driver against a SHARED pool). Populate it by first running one
	// warmup frame through run_sample_frame_pooled (which calls build_pooled_order + compiles).
	const std::vector<std::pair<int, int>> &pooled_order() const { return m_pooled_order; }

	// PROGRAM-SHARED POOL (idea ①): emit 3 per-program drivers + ONE shared op-body pool (the union of
	// all 3 programs' poolable ops) into a SINGLE code region, so the ~18KB pool stays L1I-resident when
	// the 3 fused per-sample instead of each frame carrying its own copy (~3x30KB). Fills out[3] with the
	// 3 driver entry points (each called as fn(device*), same convention as the pooled frame). devs[g]
	// must be the device whose orders[g] was built (the drivers bake per-device cache pointers). Returns
	// true on success; false (out[*]=null) if any driver shape is unsupported. Reports the fused region
	// size in *code_size if non-null. The JitRuntime of THIS Jit owns the region.
	bool compile_fused(tms57002_device *const devs[3], const std::vector<std::pair<int, int>> *const orders[3],
	                   FrameFn out[3], std::size_t *code_size = nullptr);

private:
	SampleFn compile_trampoline_loop();   // the bounded "call jit_step_one until idle" loop
	PcFn get_pc_fn(tms57002_device &dsp, int ipc);   // cache lookup / compile
	PcFn compile_pc(tms57002_device &dsp, int ipc);  // emit one PC's chain
	FrameFn compile_frame(tms57002_device &dsp, int max_steps, int start_pc);  // emit the whole frame
	FrameFn compile_frame_pooled(tms57002_device &dsp, int max_steps, int start_pc);  // M2 pooled+pinned frame
	void build_pooled_order(tms57002_device &dsp, int start_pc, int max_steps);  // M3: program-order (pc,ipc) list
	FrameFn compile_frame_callfree(tms57002_device &dsp, int max_steps, int start_pc);  // call-free nop frame (validation)

	asmjit::JitRuntime m_rt;               // owns compiled code
	SampleFn m_fn = nullptr;               // cached compiled frame runner
	FrameFn m_callfree_fn = nullptr;       // cached call-free frame (validation)
	int m_callfree_max = -1, m_callfree_start = -1;
	bool m_callfree_failed = false;
	std::vector<PcFn> m_pc_fns;            // ipc -> compiled per-PC fn (flat, O(1), no hash); nullptr = uncompiled
	bool m_pc_native_failed = false;       // codegen failed -> fall back
	FrameFn m_frame_fn = nullptr;          // cached whole-frame fn (keyed by max_steps + start_pc below)
	int m_frame_max_steps = -1;
	int m_frame_start_pc = -1;
	u32 m_frame_start_st1 = ~0u;            // st1 cache bits the trace was recorded under (recompile if changed)
	bool m_frame_failed = false;           // whole-frame codegen unavailable -> per-PC path
	// M4: per-program compile cache. A compiled pooled frame bakes the device's decode-cache layout, and
	// the live machine RELOADS DSP programs at PLOAD (flushing that cache), so a single cached frame keyed
	// on (pc,st1) goes stale on a reload. Instead cache one entry per (program hash, start_st1, max_steps):
	// a re-loaded patch reuses its compiled frame (skips the expensive asmjit codegen) and only re-primes
	// the decode cache (build_pooled_order, deterministic + cheap). Detected cheaply via the device program
	// version (re-hash only when it changes). m_pooled_order stays the scratch for the current build/compile.
	struct PooledEntry {
		FrameFn fn = nullptr;
		std::vector<std::pair<int, int>> order;   // this program's (pc,ipc) list (for re-prime + pooled_order())
		int max_steps = -1, start_pc = -1;
		u32 start_st1 = ~0u;
		bool failed = false;                       // compile returned null -> this program runs via interp
	};
	std::map<std::tuple<std::uint64_t, u32, int>, PooledEntry> m_pooled_cache;   // (hash, start_st1, max_steps)
	PooledEntry *m_active = nullptr;       // entry driving the current program (std::map nodes are pointer-stable)
	bool m_pooled_partial = false;         // last pooled run exited at icount<=0 mid-frame -> resume (mid-PC re-entry) next timeslice
	u32 m_last_prog_version = ~0u;         // last-seen device program version (cheap change detection)
	std::uint64_t m_cur_prog_hash = 0;     // hash of the current program (recomputed only on a version change)
	long m_pooled_runs = 0;         // frames executed via a compiled pooled frame (M3 native-branch path)
	long m_pooled_fallbacks = 0;    // frames routed to the native frame instead (fallback)
	long m_pooled_compiles = 0;     // M4: distinct asmjit compiles done (a cache reuse does NOT bump this)
	// M3: the program-order PC list (pc, chain-head ipc) for the pooled frame — EVERY reachable PC,
	// decoded at its deterministic st1 (branches skip no mode-setters), so both branch paths compile.
	std::vector<std::pair<int, int>> m_pooled_order;
	// F5 (2026-07-05): result of the per-program mode-safety check run by build_pooled_order — the
	// "no branch skips a mode-setter" assumption VERIFIED for this program (for every branch k->j,
	// st1-cache after k's decode == st1-cache on the fall-through arrival at j). False -> the
	// fall-through order walk decoded some PC at a wrong st1 mode for one of the paths, so the
	// compiled variants could be silently wrong: compile_frame_pooled refuses (program runs interp).
	bool m_pooled_order_mode_safe = true;
	// Per-PC decode trace, recorded by interpreting ONE frame before compiling. The TMS57002 decode
	// is st1-mode-dependent and st1 evolves mid-frame (data-independent for a branch-free program),
	// so compile_frame must decode each PC at the st1 it ACTUALLY has when reached, not frame-start.
	// m_frame_trace[k] = the chain-head ipc that ran at step k. (see docs/2026-06-25-jit-model-decision-gearmulator.md)
	std::vector<int> m_frame_trace;
};

}  // namespace tms57002

#endif  // MAME_CPU_TMS57002_JIT_TMS57002_JIT_H
