// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    tms57002.h

    TMS57002 "DASP" emulator.

***************************************************************************/
#ifndef MAME_CPU_TMS57002_TMS57002_H
#define MAME_CPU_TMS57002_TMS57002_H

#pragma once

#include <functional>
#include <string>

namespace tms57002 { class Jit; }   // DSP dynarec (jit/tms57002_jit.h); owned per-device, created lazily

class tms57002_device : public cpu_device, public device_sound_interface
{
public:
	enum class debug_macc_clip_mode : u8
	{
		none = 0,
		rounded,
		unrounded
	};

	struct debug_macc_eval_result
	{
		s64 value;
		bool mov;
	};

	struct debug_snapshot
	{
		u64 sound_updates = 0;
		u8 pc = 0;
		u8 hpc = 0;
		u8 ca = 0;
		u8 id = 0;
		u8 ba0 = 0;
		u8 ba1 = 0;
		u8 rptc = 0;
		u8 rptc_next = 0;
		u8 sa = 0;
		u8 hidx = 0;
		u8 allow_update = 0;
		u32 st0 = 0;
		u32 st1 = 0;
		u32 sti = 0;
		u32 aacc = 0;
		u64 macc = 0;
		u64 macc_read = 0;
		u64 macc_write = 0;
		u32 creg = 0;
		u32 xoa = 0;
		u32 xba = 0;
		u32 xwr = 0;
		u32 xrd = 0;
		u32 txrd = 0;
		u32 xm_adr = 0;
		u8 xm_cycles = 0;
		u8 xm_fetches = 0;
		std::array<u32, 4> si{};
		std::array<u32, 4> so{};
		std::array<u32, 4> serial_input_latch{};
		std::array<u32, 4> serial_input_active{};
		std::array<u32, 4> serial_input_frame{};
		std::array<u32, 4> serial_input_prev_frame{};
		std::array<u32, 4> serial_input_pending{};
		u8 serial_input_valid = 0;
		u8 serial_input_active_valid = 0;
		u8 serial_input_prev_valid = 0;
		u8 serial_input_pending_valid = 0;
		u8 serial_output_pending_valid = 0;
		u8 pending_pre_transfer = 0;
		u32 pending_pre_transfer_addr = 0;
		u32 pending_pre_transfer_value = 0;
		u8 serial_input_timing_mode = 0;
		u8 serial_frame_mode = 0;
		int serial_frame_clocks = 0;
		int serial_exec_halfcycles = 0;
		int serial_input_pending_halfcycle = 0;
		int serial_output_pending_halfcycle = 0;
		u8 sync_polarity_rising = 0;
		u8 serial_output_muted = 0;
		u8 serial_frame_flip_input = 0;
		u8 serial_frame_flip_output = 0;
		u8 update_counter_head = 0;
		u8 update_counter_tail = 0;
		u8 update_active = 0;
		u8 update_address_run = 0;
		std::array<u32, 256> cmem{};
		std::array<u32, 256> dmem0{};
		std::array<u32, 32> dmem1{};
		std::array<u32, 16> update{};
		std::array<u8, 16> update_sa{};
		std::array<u8, 16> update_address_run_for_entry{};
		std::array<u64, 16> update_enqueue_su{};
		std::array<u8, 16> update_read_delay_seen{};
	};

	tms57002_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto dready_callback() { return m_dready_callback.bind(); }
	auto pc0_callback() { return m_pc0_callback.bind(); }
	auto empty_callback() { return m_empty_callback.bind(); }

	u8 data_r();
	void data_w(u8 data);
	u32 serial_input(int index) const
	{
		if (index < 0 || index >= 4)
			return 0;
		if (BIT(m_serial_input_valid, index))
			return m_serial_input_latch[index];
		if (BIT(m_serial_input_active_valid, index))
			return m_serial_input_active[index];
		return si[index];
	}
	u32 serial_output(int index) const { return (index >= 0 && index < 4) ? serial_output_pin(index) : 0; }
	// Inter-DSP routing tap that bypasses the PC7 mute gate (A/B for whether
	// hardware mute is DAC-side only; see audit findings Â§3.2).
	u32 serial_output_unmuted(int index) const;
	void set_serial_input(int index, u32 value)
	{
		if (index >= 0 && index < 4)
		{
			m_serial_input_latch[index] = value & 0x00ffffffU;
			m_serial_input_valid |= u8(1U << index);
		}
	}
	void set_serial_frame_model(bool enable) { m_serial_frame_mode = enable ? serial_frame_mode::lrck_subframe : serial_frame_mode::snapshot; }
	void set_serial_frame_clocks(int clocks) { m_serial_frame_clocks = clocks > 0 ? clocks : 0; }
	void set_serial_frame_flip_input(bool enable) { m_serial_frame_flip_input = enable; }
	void set_serial_frame_flip_output(bool enable) { m_serial_frame_flip_output = enable; }
	void set_serial_output_write_handoff(bool enable) { m_serial_output_write_handoff = enable; }
	void set_stream_output_raw(bool enable) { m_stream_output_raw = enable; }
	// Generic serial-output event observer (SO writes / frame handoffs). Default-unset => no-op, zero
	// cost. Host drivers install one to observe DSP serial output (e.g. hardware-serial verification)
	// without an in-core trace or getenv.
	using serial_output_observer = std::function<void(const char *event, int index, u32 value, int halfcycles, u8 mask)>;
	void set_serial_output_observer(serial_output_observer cb) { m_serial_output_observer = std::move(cb); }
	void set_smhc_post_slot(bool enable) { m_smhc_post_slot = enable; }
	// SMHC stores only the high (24-bit) coefficient; low `bits` bits are forced to
	// zero on write. Real-silicon behavior (Korg Prophecy ramp-equilibrium bit-exact,
	// 2026-07-03); 8 = the 32-bit->24-bit truncation. 0 = legacy full macc>>16.
	virtual ~tms57002_device();   // out-of-line: m_jit's unique_ptr needs the complete Jit type (tms57002.cpp)

	void set_smhc_trunc_bits(int bits) { m_debug_smhc_trunc_bits = bits; }
	// Machine-config default for the pooled dynarec (the KPROP_DSP_PERFRAME=4 path). OFF for generic
	// TMS57002 users; the Korg driver turns it on (gated by the 2026-07-07 F6 corpus sweep). The env
	// still overrides at runtime: 4 = force on, any other value = force interpreter (kill switch).
	// Non-x86-64 hosts fall back to the interpreter automatically (compile returns nullptr).
	void set_dynarec_default(bool enable) { m_dynarec_default = enable; }
	// Delay CA post-increment commit by one instruction: a c*+ read returns the
	// current CA, but the increment only lands after the FOLLOWING instruction's
	// operand reads. Aligns the voice program's output-mixer gain walk
	// (lcak 37; mpy c*+; mac d,c*+; mac d,c*; mac d,c*) to one gain per bus
	// (c37,c37,c38,c39) as the hardware A12 staged image implies, instead of
	// (c37,c38,c39,c39). ID (d*+) increments are not delayed.
	void set_ca_inc_delayed(bool enable) { m_ca_inc_delayed = enable; }
	// abs of the most-negative AACC (0x80000000): saturate to 0x7fffffff
	// instead of wrapping back to 0x80000000 (MAME historically wraps and only
	// sets AOV). The Prophecy A12 voice program's decay loop starts from a
	// -1.0-seeded state; without saturating abs the loop never decays and the
	// idle voice drones forever (drone investigation, 2026-07-02).
	void set_abs_saturate(bool enable) { m_abs_saturate = enable; }
	void set_sync_polarity(int state) { m_sync_polarity_rising = bool(state); }
	void mute_w(int state) { m_serial_output_muted = !bool(state); }
	void debug_write_dmem0(u8 index, u32 value) { dmem0[index] = value & 0xffffff; }
	void debug_write_dmem1(u8 index, u32 value) { dmem1[index & 0x1f] = value & 0xffffff; }
	void debug_write_cmem(u8 index, u32 value) { cmem[index] = value; }
	void debug_set_exec_state(u8 pc_value, u8 ca_value, u8 id_value, u8 ba0_value, u8 ba1_value, u32 st0_value, u32 st1_value, u32 sti_value)
	{
		pc = pc_value;
		ca = ca_value;
		id = id_value;
		ba0 = ba0_value;
		ba1 = ba1_value;
		st0 = st0_value;
		st1 = st1_value;
		sti = sti_value;
		allow_update = 1;
		update_pc0();
		update_dready();
		update_empty();
	}
	void debug_set_accumulators(u32 aacc_value, u64 macc_value, u64 macc_read_value, u64 macc_write_value, u32 creg_value)
	{
		aacc = aacc_value;
		macc = s64(macc_value);
		macc_read = s64(macc_read_value);
		macc_write = s64(macc_write_value);
		creg = creg_value;
	}
	void debug_set_xmem_state(u32 xoa_value, u32 xba_value, u32 xwr_value, u32 xrd_value, u32 txrd_value, u32 xm_adr_value, u8 xm_cycles_value, u8 xm_fetches_value)
	{
		xoa = xoa_value;
		xba = xba_value;
		xwr = xwr_value;
		xrd = xrd_value;
		txrd = txrd_value;
		xm_adr = xm_adr_value;
		xm_cycles = xm_cycles_value;
		xm_fetches = xm_fetches_value;
	}
	void debug_load_program(const u32 *words, u32 count, u32 st0_value, u32 st1_value);
	void debug_reset_live_state(bool zero_accumulators);
	void debug_run_cycles(int max_cycles = 1);
	std::array<u32, 4> debug_run_sample_frame(const std::array<u32, 4> &frame, int max_cycles = 4096);

	// --- JIT harness seam (Phase 0) ------------------------------------------
	// debug_run_sample_frame() split so an external driver (the asmjit JIT) can walk
	// the loaded program itself and, op by op, replace interpreted stepping with
	// generated code. jit_step_one() is the trampoline target (run one program
	// instruction via the interpreter); the rest are the shared frame begin/idle/end.
	// The interpreter stays the default + the bit-exact oracle (tms57002_jit_test).
	void jit_begin_frame(const std::array<u32, 4> &frame);          // serial setup + sync_w
	bool jit_is_idle() const { return (sti & S_IDLE) != 0; }        // program hit `idle`
	// PLOAD mutates the program image and must always disengage pooled execution.
	// CLOAD only assembles a coefficient packet in host[]; no CMEM-visible state
	// changes until the complete packet is queued.  With the guarded CMEM entry
	// enabled, pooled execution can therefore continue across CLOAD as well as the
	// subsequent pending-update window.  Keep the old conservative behavior when
	// that entry is disabled.
	bool jit_host_loading_unsafe() const
	{
		return (sti & IN_PLOAD) || ((sti & IN_CLOAD) && !m_pf4_cmem_deopt);
	}
	void jit_step_one() { debug_run_cycles(1); }                    // one program instruction
	std::array<u32, 4> jit_end_frame()                              // the 4 serial outputs
	{ return { serial_output_pin(0), serial_output_pin(1), serial_output_pin(2), serial_output_pin(3) }; }

	// Execute one program instruction (one PC's chain + machinery) from chain-head
	// ipc, returning the next ipc. Trace-free replica of execute_run's per-PC body —
	// the unit the JIT drives compile-time-unrolled. jit_run_sample_frame runs a whole
	// frame through it (bit-exact vs debug_run_sample_frame, gated by tms57002_jit_test).
	int jit_run_chain(int ipc);
	std::array<u32, 4> jit_run_sample_frame(const std::array<u32, 4> &frame, int max_cycles = 4096);
	// jit_run_chain decomposed: the JIT unrolls the chain and emits these per micro-op.
	void jit_pc_pre();                        // xm_step + MACC pipeline
	void jit_op_exec(unsigned op, const void *icd_ptr);   // the ex_N dispatch (op >= 4)
	int jit_pc_post(int iipc, int ipc);       // transfer/serial/pc/branch -> next ipc
	void jit_term(unsigned op) { if (op == 1) ++ca; else if (op == 2) ++id; else if (op == 3) { ++ca; ++id; } }  // chain terminator op<4

	// Compile-time decode-cache access for the asmjit compiler to walk a PC's chain.
	unsigned jit_inst_op(int ipc) const { return cache.inst[ipc].op; }
	int jit_inst_next(int ipc) const { return cache.inst[ipc].next; }
	u64 jit_inst_addr(int ipc) { return reinterpret_cast<u64>(&cache.inst[ipc]); }
	int jit_decode(u8 p) { u8 save = pc; pc = p; int r = decode_get_pc(); pc = save; return r; }  // decode p's chain head
	u32 jit_sti() const { return sti; }
	// Frame-loop driving for the native JIT (icount/idle live on the device).
	void jit_set_icount(int n) { icount = n > 1 ? n : 1; }
	int jit_icount() const { return icount; }   // the scheduler's remaining timeslice at the pf4 hook
	bool jit_running() const { return icount > 0 && !(sti & (S_IDLE | IN_PLOAD)); }
	int jit_decode_current() { return decode_get_pc(); }
	int jit_pc() const { return pc; }   // frame-start pc (compile-time anchor for the whole-frame unroll)
	void jit_set_pc(u8 p) { pc = p; }   // M3: force program-order decode (build_pooled_order forces fall-through)
	void jit_cache_flush() { cache_flush(); }   // M3: fresh decodes (drop stale/colliding warmup-frame entries)
	// M3: decode-chaining control for the pooled-order walk. Mode-setters (sfmo/sfai/...) mutate st1's
	// decode-key fields AT DECODE TIME; with chaining ON, decode_get_pc decodes a whole run of PCs in one
	// call and over-advances st1 past the current PC. Disabling it makes each decode_get_pc decode EXACTLY
	// one PC, so a program-order walk sees the correct per-PC st1 (matching the interpreter's decoder).
	bool jit_disable_chaining() const { return m_disable_decode_chaining; }
	void jit_set_disable_chaining(bool b) { m_disable_decode_chaining = b; }
	// M4: program identity for the compile cache. jit_program_version() bumps on each (re)load (cheap
	// change detection); jit_program_hash() is FNV-1a over the 256 program words (dedups identical
	// programs across reloads, so a re-loaded patch reuses its compiled frame instead of recompiling).
	u32 jit_program_version() const { return m_program_version; }
	u32 dbg_st1() const { return st1; }   // JIT reads st1 mode bits for the compile cache key
	int jit_smhc_trunc_bits() const { return m_debug_smhc_trunc_bits; }   // fork silicon fix: smhc low-bit truncation (Korg=8)
	// Silicon has a 32-bit coefficient input and a 24-bit AACC input at the
	// multiplier. Only c,a forms route AACC through this port; a,d forms route
	// it through the full-width coefficient input instead.
	static constexpr u32 mpy_a_operand(u32 a) { return a & 0xffffff00U; }
	bool jit_abs_saturate() const { return m_abs_saturate; }              // fork silicon fix: abs saturates INT_MIN
	// DEFINED OUT-OF-LINE in tms57002.cpp — NOT inline. It dereferences `program` (the address_space*),
	// and the JIT library compiles tms57002.h against the compat emu.h whose device layout differs from
	// the runtime's; an inline body compiled in the JIT TU would read `program` at the wrong offset and
	// crash on a runtime-laid-out device (the live machine). The .cpp is compiled with the real layout.
	u64 jit_program_hash();
	void jit_run_rest(int ipc) { while (jit_running()) { if (ipc == -1) ipc = decode_get_pc(); ipc = jit_run_chain(ipc); } }  // finish frame via interpreter
	void jit_finalize_if_idle() { if (serial_cycle_model_enabled() && (sti & S_IDLE)) finalize_serial_output_build(); if (icount > 0) icount = 0; }
#ifdef TMS_POC_FRAME
	// PoC ceiling bench: GENERATED straight-line steady-state DSP1 frame (test/poc_dsp1_frame.inc,
	// included at the end of tms57002.cpp so every helper inlines). Local research artifact.
	void poc_frame();        // conservative: verbatim interpreter semantics, zero dispatch
	void poc_frame_fast();   // + the guarded substitutions a runtime JIT would make (the ceiling)
	// Distinct code copies of the same frame (same source => identical ~82KB footprint each),
	// so the fused-group experiment can alternate 2-3 large straight-line passes per sample and
	// see whether they stay resident (L1I 32K / L2 256K) or thrash. See test/tms57002_poc_test.cpp.
	void poc_frame_b();  void poc_frame_fast_b();
	void poc_frame_c();  void poc_frame_fast_c();
	// DENSE point on the code-shape curve: ONE shared table-driven pass (55 op bodies in a
	// switch, 335-op stream). One code copy, called by all 3 DSP devices in the fused test --
	// the shape that attacks the distinct-code L2 collapse directly. See test/poc_dsp1_dense.inc.
	void poc_frame_dense();   // dispatch shape: centralized `switch`
	void poc_frame_goto();    // dispatch shape: distributed threaded computed-goto (same bodies)
	// FUSION kernel: ONE shared 84-op switch (union across the real DSP1/2/3 programs) + per-
	// program step streams. poc_frame_pool(prog) runs program `prog` (0=osc,1=filter,2=fx)
	// through the one shared pool — 3 devices share one code region. See test/poc_dsp_pool.inc.
	void poc_frame_pool(int prog);
	bool poc_diverged = false;   // set if the trace assumption (no branch/repeat) breaks at runtime
#endif
	// Member offsets + masks for the JIT to emit native device-state access.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"   // JIT emits native access via __builtin_offsetof; tms57002_device is not standard-layout (device base classes)
	static u32 jit_off_sti() { return u32(__builtin_offsetof(tms57002_device, sti)); }
	static u32 jit_off_macc() { return u32(__builtin_offsetof(tms57002_device, macc)); }
	static u32 jit_off_macc_read() { return u32(__builtin_offsetof(tms57002_device, macc_read)); }
	static u32 jit_off_macc_write() { return u32(__builtin_offsetof(tms57002_device, macc_write)); }
	static u32 jit_off_ca() { return u32(__builtin_offsetof(tms57002_device, ca)); }
	static u32 jit_off_id() { return u32(__builtin_offsetof(tms57002_device, id)); }
	static u32 jit_off_icount() { return u32(__builtin_offsetof(tms57002_device, icount)); }
	static u32 jit_off_pc() { return u32(__builtin_offsetof(tms57002_device, pc)); }
	static u32 jit_off_rptc() { return u32(__builtin_offsetof(tms57002_device, rptc)); }
	static u32 jit_off_rptc_next() { return u32(__builtin_offsetof(tms57002_device, rptc_next)); }
	static u32 jit_off_pending_pre_transfer() { return u32(__builtin_offsetof(tms57002_device, m_pending_pre_transfer)); }
	static u32 jit_off_dmem0() { return u32(__builtin_offsetof(tms57002_device, dmem0)); }
	static u32 jit_off_dmem1() { return u32(__builtin_offsetof(tms57002_device, dmem1)); }   // dbp=1 bank
	static u32 jit_off_ba0() { return u32(__builtin_offsetof(tms57002_device, ba0)); }
	static u32 jit_off_ba1() { return u32(__builtin_offsetof(tms57002_device, ba1)); }
	static u32 jit_off_st1() { return u32(__builtin_offsetof(tms57002_device, st1)); }
	static u32 jit_off_xrd() { return u32(__builtin_offsetof(tms57002_device, xrd)); }
	static u32 jit_off_so() { return u32(__builtin_offsetof(tms57002_device, so)); }   // serial output regs (dos/domh)
	static constexpr u32 jit_st1_mov() { return ST1_MOV; }
	static constexpr u32 jit_st1_sfmo() { return ST1_SFMO; }   // sfmo mode field (op 1843 clears it)
	static constexpr u32 jit_st1_aov() { return ST1_AOV; }     // aacc-overflow flag (%wa saturation)
	static constexpr u32 jit_st1_aovm() { return ST1_AOVM; }   // aacc-overflow-clamp mode (%wa saturation)
	static constexpr u32 jit_st1_sfai() { return ST1_SFAI; }   // sfai flag (sfai op set/clear)
	static constexpr int jit_st1_sfmo_shift() { return ST1_SFMO_SHIFT; }   // sfmo field shift (op 1844-1846)
	static constexpr u32 jit_st1_cache() { return ST1_CACHE; }   // st1 bits that select op variants (decode key)
	static constexpr u32 jit_st1_crm() { return ST1_CRM; }         // coefficient-RAM mode field (current_cmem_view)
	static constexpr u32 jit_st1_crm_16h() { return ST1_CRM_16H; } // crm==1: cmem read -> value & 0xffff0000
	static constexpr u32 jit_st1_crm_16l() { return ST1_CRM_16L; } // crm==2: cmem read -> value << 16
	// Remaining ST1_CACHE field masks — used by the F5-refinement consumed-field table
	// (tms57002_ops.cpp) to map descriptor variant columns onto st1 mode bits.
	static constexpr u32 jit_st1_sfao() { return ST1_SFAO; }
	static constexpr u32 jit_st1_sfma() { return ST1_SFMA; }
	static constexpr u32 jit_st1_rnd_field() { return ST1_RND; }
	static constexpr u32 jit_st1_movm() { return ST1_MOVM; }
	static constexpr u32 jit_st1_dbp() { return ST1_DBP; }
	bool jit_smhd_raw_path() const { return m_debug_smhd_raw_path; }
	bool jit_smld_raw_path() const { return m_debug_smld_raw_path; }
	// The pooled (M2) op bodies assume normal render conditions — no pending cmem update (so a cmem
	// read is a direct cmem[addr]), no debug forces/forwards, no serial cycle model. If any of these
	// hold, the pooled frame falls back to the (unconditionally-correct) native frame. True == safe to
	// run the pooled frame for the whole sample. (The live machine / M5 must revisit mid-frame changes.)
	bool jit_pooled_safe() const {
		// The deopt mode admits a frame with pending CMEM updates because every CMEM-reading
		// pooled op guards and re-enters the interpreter until the queue drains.
		return (update_counter_head == update_counter_tail || m_pf4_force_cmem_unsafe || m_pf4_cmem_deopt)
			&& !m_debug_cmem_force
			&& !serial_cycle_model_enabled()
			&& !m_debug_mpy_smhd_forward && !m_debug_lmhd_srbd_forward
			&& !m_debug_smhd_raw_path && !m_debug_smld_raw_path
			// ca_inc_delayed: build_pooled_order decodes with chaining DISABLED, and decode_get_pc
			// merges a carried INC_CA into the CURRENT boundary op when chaining is off — so the
			// primed chains would differ from the interpreter's chaining-on chains (CA increments one
			// slot early at carry sites). Pooled must disengage rather than run divergent chains.
			&& !m_ca_inc_delayed
			// The pooled mac/mpy bodies bake the base product shifts (>>7/>>14/>>15) at emit time;
			// the dc/ca extra-shift debug knobs are invisible to them -> disengage when set.
			&& m_debug_ca_mpy_extra_shift == 0 && m_debug_dc_mpy_extra_shift == 0;
	}
	static constexpr u32 jit_s_idle_mask() { return S_IDLE; }
	static constexpr u32 jit_s_read_mask() { return S_READ; }
	static constexpr u32 jit_s_write_mask() { return S_WRITE; }
	static constexpr u32 jit_s_branch_mask() { return S_BRANCH; }
	static u32 jit_off_aacc() { return u32(__builtin_offsetof(tms57002_device, aacc)); }
	static u32 jit_off_cmem() { return u32(__builtin_offsetof(tms57002_device, cmem)); }
	static u32 jit_off_uc_head() { return u32(__builtin_offsetof(tms57002_device, update_counter_head)); }
	static u32 jit_off_uc_tail() { return u32(__builtin_offsetof(tms57002_device, update_counter_tail)); }
	static u32 jit_off_cmem_force() { return u32(__builtin_offsetof(tms57002_device, m_debug_cmem_force)); }
	static u32 jit_off_pf4_force() { return u32(__builtin_offsetof(tms57002_device, m_pf4_force_cmem_unsafe)); }
	bool jit_pf4_cmem_deopt() const { return m_pf4_cmem_deopt; }
	bool jit_cmem_pending() const { return update_counter_head != update_counter_tail; }
	static u32 jit_off_creg() { return u32(__builtin_offsetof(tms57002_device, creg)); }   // mac/mpy write creg = get_cmem(ca)
	static u32 jit_off_xoa() { return u32(__builtin_offsetof(tms57002_device, xoa)); }   // rde/wre XRAM offset addr
	static u32 jit_off_xwr() { return u32(__builtin_offsetof(tms57002_device, xwr)); }   // wre XRAM write data
#pragma GCC diagnostic pop
	void jit_xm_init() { xm_init(); }   // rde/wre arm the XRAM transaction (no macc)
	bool jit_mpy_forward_on() const { return m_debug_mpy_smhd_forward; }   // debug: if on, mac/mpy deopt (compile-time gate)
	bool jit_lmhd_forward_on() const { return m_debug_lmhd_srbd_forward; }  // debug: if on, lmhd deopt (compile-time gate)
	unsigned jit_inst_param(int ipc) const { return cache.inst[ipc].param; }   // icd cmem/dmem address operand
	u32 jit_get_cmem(u8 a) { return get_cmem(a); }   // deopt target for the lacc fast-path inline
	// Emit-time probe: run ex_op with st1 = st1_in and return the resulting st1, non-destructively
	// (full snapshot/restore). Flag-setter ops (`f` type: sfmo/sfai/sfao/sfma/rnd/scrm/...) are
	// affine in st1 — st1' = (st1 & keep) | set — so the op-emitter discovers (keep,set) by probing
	// the oracle at two st1 values instead of hard-coding the ST1_* field layout. icd may be any
	// valid instruction descriptor (flag-setters ignore i->param).
	u32 jit_probe_st1(unsigned op, const void *icd_ptr, u32 st1_in) {
		debug_snapshot s = debug_capture_snapshot();
		st1 = st1_in;
		jit_op_exec(op, icd_ptr);
		u32 out = st1;
		debug_restore_snapshot(s);
		return out;
	}
	const char *jit_op_mnemonic(unsigned op) const;   // op number -> mnemonic (compile-time op-inlining dispatch)
	unsigned jit_op_variant(unsigned op) const;       // variant index within mnemonic group (0 = simplest form; JIT inlines only variant 0)
	bool jit_serial_model() const { return serial_cycle_model_enabled(); }   // compile-time gate
	void jit_xm_step() { if (sti & S_READ) xm_step_read(); else xm_step_write(); }  // called only when S_READ|S_WRITE set

	// Distinct instruction-form mnemonics in the decode cache (i.e. used by the loaded
	// program). Maps generated op indices back to forms via the generated name table —
	// the JIT "implement these first" coverage list. Stable program -> call post-render.
	std::vector<std::string> jit_used_mnemonics() const;
	debug_snapshot debug_capture_snapshot() const;
	void debug_restore_snapshot(const debug_snapshot &snapshot);
	void debug_begin_sample_frame(const std::array<u32, 4> &frame);
	void maybe_state_zap();
	debug_macc_eval_result debug_eval_macc_output(u64 macc_value, int sfmo_mode, int rnd_mode, debug_macc_clip_mode clip_mode);
	u64 sound_update_count() const { return m_sound_updates; }
	u32 debug_serial_output_register(int index) const { return (index >= 0 && index < 4) ? so[index] : 0; }
	u64 debug_macc() const { return macc; }
	u64 debug_macc_read() const { return macc_read; }
	u64 debug_macc_write() const { return macc_write; }
	s32 debug_aacc() const { return aacc; }
	u32 debug_creg() const { return creg; }
	u32 debug_xoa() const { return xoa; }
	u32 debug_xba() const { return xba; }
	u32 debug_xrd() const { return xrd; }
	u32 debug_txrd() const { return txrd; }
	u32 debug_xwr() const { return xwr; }
	u32 debug_xm_adr() const { return xm_adr; }
	u8 debug_xm_cycles() const { return xm_cycles; }
	u8 debug_xm_fetches() const { return xm_fetches; }
	u32 cmem_value(u8 index) const { return cmem[index]; }
	u32 dmem0_value(u8 index) const { return dmem0[index]; }
	u32 dmem1_value(u8 index) const { return dmem1[index & 0x1f]; }
	u8 debug_ba0() const { return ba0; }
	u8 debug_ba1() const { return ba1; }
	void debug_set_banks(u8 ba0_value, u8 ba1_value) { ba0 = ba0_value; ba1 = ba1_value; }
	u32 status_bits() const { return sti; }
	u8 host_addr() const { return sa; }
	u8 host_index() const { return hidx; }
	u8 update_pending_count() const { return (update_counter_head - update_counter_tail) & 0x0f; }

	void pload_w(int state);
	void cload_w(int state);
	int empty_r();
	int dready_r();
	int pc0_r();
	void sync_w(int state);

	void internal_pgm(address_map &map) ATTR_COLD;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;
	virtual void sound_stream_update(sound_stream &stream) override;
	virtual space_config_vector memory_space_config() const override;
	virtual u32 execute_min_cycles() const noexcept override;
	virtual u32 execute_max_cycles() const noexcept override;
	virtual void execute_run() override;
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

private:
	enum {
		IN_PLOAD = 0x00000001,
		IN_CLOAD = 0x00000002,
		SU_CVAL  = 0x00000004,
		SU_MASK  = 0x00000018, SU_ST0 = 0x00, SU_ST1 = 0x08, SU_PRG = 0x10,
		S_IDLE   = 0x00000020,
		S_READ   = 0x00000040,
		S_WRITE  = 0x00000080,
		S_BRANCH = 0x00000100,
		S_HOST   = 0x00000200,
		S_UPDATE = 0x00000400,
	};

	enum class update_timing_mode : u8
	{
		normal = 0,
		eager,
		sample_delay_1,
		read_delay_1
	};

	enum class serial_input_timing_mode : u8
	{
		same_sample = 0,
		sample_delay_1
	};

	enum class serial_frame_mode : u8
	{
		snapshot = 0,
		lrck_subframe
	};

	enum class serial_input_immediate_side_mode : u8
	{
		auto_select = 0,
		left,
		right
	};

	enum class pending_pre_transfer_type : u8
	{
		none = 0,
		dis_dmem0,
		dis_dmem1
	};

	enum {
		ST0_INCS = 0x000001,
		ST0_DIRI = 0x000002,
		ST0_FI   = 0x000004,
		ST0_SIM  = 0x000008,
		ST0_PLRI = 0x000020,
		ST0_PBCI = 0x000040,
		ST0_DIRO = 0x000080,
		ST0_FO   = 0x000100,
		ST0_SOM  = 0x000600,
		ST0_PLRO = 0x000800,
		ST0_PBCO = 0x001000,
		ST0_CNS  = 0x002000,
		ST0_WORD = 0x004000,
		ST0_SEL  = 0x008000,
		ST0_M    = 0x030000, ST0_M_64K = 0x000000, ST0_M_256K = 0x010000, ST0_M_1M = 0x020000,
		ST0_SRAM = 0x200000,

		ST1_AOV  = 0x000001,
		ST1_SFAI = 0x000002,
		ST1_SFAO = 0x000004,
		ST1_AOVM = 0x000008, // undocumented!
		ST1_MOVM = 0x000020,
		ST1_MOV  = 0x000040,
		ST1_SFMA = 0x000180, ST1_SFMA_SHIFT = 7,
		ST1_SFMO = 0x001800, ST1_SFMO_SHIFT = 11,
		ST1_RND  = 0x038000, ST1_RND_SHIFT = 15,
		ST1_CRM  = 0x0C0000, ST1_CRM_SHIFT = 18, ST1_CRM_32 = 0x000000, ST1_CRM_16H = 0x040000, ST1_CRM_16L = 0x080000,
		ST1_DBP  = 0x100000,
		ST1_CAS  = 0x200000,

		ST1_CACHE = ST1_SFAI|ST1_SFAO|ST1_MOVM|ST1_SFMA|ST1_SFMO|ST1_RND|ST1_CRM|ST1_DBP
	};


	enum { BR_UB, BR_CB, BR_IDLE };

	enum { IBS = 8192, HBS = 4096 };

	enum { INC_CA = 1, INC_ID = 2 };

	struct icd {
		u16 op;
		s16 next;
		u8 param;
	};

	struct hcd {
		u32 st1;
		s16 ipc;
		s16 next;
	};

	struct cd {
		s16 hashbase[256];
		hcd hashnode[HBS];
		icd inst[IBS];
		int hused, iused;
	};

	struct cstate {
		int branch;
		int inc;
		s16 hnode;
		s16 ipc;
	};

	// macc_read and macc_write are used by non-pipelined instructions
	s64 macc, macc_read, macc_write;

	u32 cmem[256];
	u32 dmem0[256];
	u32 dmem1[32];

	u32 si[4], so[4];
	std::array<u32, 4> m_serial_input_latch;
	std::array<u32, 4> m_serial_input_active;
	std::array<u32, 4> m_serial_input_frame;
	std::array<u32, 4> m_serial_input_prev_frame;
	std::array<u32, 4> m_serial_input_pending;
	std::array<u32, 4> m_serial_output_frame;
	std::array<u32, 4> m_serial_output_build;
	serial_output_observer m_serial_output_observer;
	u8 m_serial_input_valid;
	u8 m_serial_input_active_valid;
	u8 m_serial_input_prev_valid;
	u8 m_serial_input_pending_valid;
	u8 m_serial_output_pending_valid;
	// (m_pending_pre_transfer itself lives next to rptc/rptc_next — see the exec-state block below.)
	u32 m_pending_pre_transfer_addr;
	u32 m_pending_pre_transfer_value;
	u8 m_pending_pre_transfer_pc;
	serial_input_timing_mode m_serial_input_timing_mode;
		serial_frame_mode m_serial_frame_mode;
		serial_input_immediate_side_mode m_serial_input_immediate_side_mode;
		int m_serial_frame_clocks;
		int m_serial_exec_halfcycles;
	int m_serial_input_pending_halfcycle;
	int m_serial_output_pending_halfcycle;
	// Left-subframe (SO0/SO2) DOS writes remain eligible later than the
	// right-subframe lanes on silicon.
	int m_serial_output_pending_halfcycle_left;
	int m_serial_input_pending_halfcycle_override;
	int m_serial_output_pending_halfcycle_override;
	int m_serial_output_pending_halfcycle_left_override;
		bool m_serial_output_write_handoff;
		bool m_sync_polarity_rising;
		bool m_serial_output_muted;
	bool m_serial_frame_flip_input;
	bool m_serial_frame_flip_output;
	bool m_stream_output_raw;
	bool m_stream_sync_enabled;
	bool m_smhc_post_slot;
	bool m_ca_inc_delayed;
	bool m_pf4_force_cmem_unsafe = false;
	bool m_pf4_cmem_deopt = false;
	bool m_abs_saturate;
	u64 m_sound_updates;
	bool m_disable_decode_chaining = false;
	u32 m_program_version = 0;   // M4: bumps on every program (re)load so the JIT detects PLOAD reloads
	bool m_input_sine = false;
	double m_input_sine_amp = 0.0;
	double m_input_sine_freq = 0.0;
		int m_dready_line;
		int m_pc0_line;
		int m_empty_line;
	update_timing_mode m_update_timing_mode;
	std::array<u8, 256> m_update_timing_addr{};
	bool m_update_timing_addr_any;
	std::array<u64, 16> m_update_enqueue_su{};
	std::array<u8, 16> m_update_read_delay_seen{};
	bool m_update_active;
	u8 m_update_address_run;
	u8 m_update_address_run_for_entry[16];
	bool m_debug_cmem_force;
			int m_debug_ca_mpy_extra_shift;
			int m_debug_dc_mpy_extra_shift;
			int m_debug_smhc_trunc_bits;
			bool m_debug_lmhd_srbd_forward;
			bool m_debug_mpy_smhd_forward;
				int m_debug_mpy_smhd_forward_pc;
				bool m_debug_smhd_raw_path;
				bool m_debug_smld_raw_path;

	u32 st0, st1, sti;
	u32 aacc, xoa, xba, xwr, xrd, txrd, creg;

	u8 pc, hpc, ca, id, ba0, ba1, rptc, rptc_next;
	// Declared HERE (not with the serial members) so the JIT's per-instruction slow-post trigger test
	// (pending pre-transfer | rptc | rptc_next) is ONE dword load over three adjacent u8s. The driver
	// verifies adjacency at emit time and falls back to three byte loads if this ever moves.
	pending_pre_transfer_type m_pending_pre_transfer;
	u8 sa;

	u32 xm_adr;
	u8 xm_cycles;
	u8 xm_fetches;

	u8 host[4], hidx, allow_update;

	u32 update[16];
	u8 update_sa[16];
	u8 update_counter_head, update_counter_tail;

	cd cache;

	devcb_write_line m_dready_callback;
	devcb_write_line m_pc0_callback;
	devcb_write_line m_empty_callback;

	const address_space_config program_config, data_config;

	address_space *program, *data;
	int icount;
	int unsupported_inst_warning;

	void decode_error(u32 opcode);
	void decode_cat1(u32 opcode, u16 *op, cstate *cs);
	void decode_cat2_pre(u32 opcode, u16 *op, cstate *cs);
	void decode_cat3(u32 opcode, u16 *op, cstate *cs);
	void decode_cat2_post(u32 opcode, u16 *op, cstate *cs);
	void prepare_serial_inputs(const std::array<u32, 4> &frame);
	void start_serial_frame(const std::array<u32, 4> &frame);
	void finalize_serial_output_build();
	void advance_serial_phase(int halfcycles);
	bool serial_cycle_model_enabled() const;
	void schedule_pre_dis_write(bool dmem1_bank, u32 addr, u32 value);
	void apply_pending_pre_transfer();
	void write_dmem(bool dmem1_bank, u32 addr, u32 value, const char *source, u8 pc_value);
	void write_cmem_from_dsp(u8 addr, u32 value);
	void write_cmem_direct(u8 addr, u32 value, const char *source, int queue_pos = -1);
	u32 serial_input_read(int index);
	void serial_output_write(int index, u32 value);
	u32 serial_output_pin(int index) const;

	inline int xmode(u32 opcode, char type, cstate *cs);
	inline int sfao(u32 st1);
	inline int dbp(u32 st1);
	inline int crm(u32 st1);
	inline int sfai(u32 st1);
	inline int sfmo(u32 st1);
	inline int rnd(u32 st1);
	inline int movm(u32 st1);
	inline int sfma(u32 st1);

	void update_dready();
	void update_pc0();
	void update_empty();
	bool update_timing_selected(u8 addr) const;
	u32 current_cmem_view(u8 addr) const;

	void xm_init();
	int xm_required_fetches() const;
	int xm_required_cycles() const;
	void xm_step_read();
	void xm_step_write();
	s64 macc_to_output_0(s64 rounding, u64 rmask);
	s64 macc_to_output_1(s64 rounding, u64 rmask);
	s64 macc_to_output_2(s64 rounding, u64 rmask);
	s64 macc_to_output_3(s64 rounding, u64 rmask);
	s64 macc_to_output_0s(s64 rounding, u64 rmask, int rnd_mode);
	s64 macc_to_output_1s(s64 rounding, u64 rmask, int rnd_mode);
	s64 macc_to_output_2s(s64 rounding, u64 rmask, int rnd_mode);
	s64 macc_to_output_3s(s64 rounding, u64 rmask, int rnd_mode);
	s64 macc_to_output_0n(s64 rounding, u64 rmask);
	s64 macc_to_output_1n(s64 rounding, u64 rmask);
	s64 macc_to_output_2n(s64 rounding, u64 rmask);
	s64 macc_to_output_3n(s64 rounding, u64 rmask);
	s64 check_macc_overflow_0();
	s64 check_macc_overflow_1();
	s64 check_macc_overflow_2();
	s64 check_macc_overflow_3();
	s64 check_macc_overflow_0s();
	s64 check_macc_overflow_1s();
		s64 check_macc_overflow_2s();
			s64 check_macc_overflow_3s();
			s64 ca_mpy_product_to_macc(s64 product, int base_shift) const;
			s64 dc_mpy_product_to_macc(s64 product) const;
			s64 smhc_store_shift(s64 macc_out) const;
			u32 lmhd_d_operand(u32 current, u8 param) const;
			u32 mpy_d_operand(u32 current, u8 param);
			void cache_flush();
	void add_one(cstate *cs, u16 op, u8 param);
	void decode_one(u32 opcode, cstate *cs, void (tms57002_device::*dec)(u32 opcode, u16 *op, cstate *cs));
	s16 get_hash(u8 adr, u32 st1, s16 *pnode);
	s16 get_hashnode(u8 adr, u32 st1, s16 pnode);
	int decode_get_pc();
	u32 get_cmem(u8 addr);

	void ex_4(const icd *i);
	void ex_5(const icd *i);
	void ex_6(const icd *i);
	void ex_7(const icd *i);
	void ex_8(const icd *i);
	void ex_9(const icd *i);
	void ex_10(const icd *i);
	void ex_11(const icd *i);
	void ex_12(const icd *i);
	void ex_13(const icd *i);
	void ex_14(const icd *i);
	void ex_15(const icd *i);
	void ex_16(const icd *i);
	void ex_17(const icd *i);
	void ex_18(const icd *i);
	void ex_19(const icd *i);
	void ex_20(const icd *i);
	void ex_21(const icd *i);
	void ex_22(const icd *i);
	void ex_23(const icd *i);
	void ex_24(const icd *i);
	void ex_25(const icd *i);
	void ex_26(const icd *i);
	void ex_27(const icd *i);
	void ex_28(const icd *i);
	void ex_29(const icd *i);
	void ex_30(const icd *i);
	void ex_31(const icd *i);
	void ex_32(const icd *i);
	void ex_33(const icd *i);
	void ex_34(const icd *i);
	void ex_35(const icd *i);
	void ex_36(const icd *i);
	void ex_37(const icd *i);
	void ex_38(const icd *i);
	void ex_39(const icd *i);
	void ex_40(const icd *i);
	void ex_41(const icd *i);
	void ex_42(const icd *i);
	void ex_43(const icd *i);
	void ex_44(const icd *i);
	void ex_45(const icd *i);
	void ex_46(const icd *i);
	void ex_47(const icd *i);
	void ex_48(const icd *i);
	void ex_49(const icd *i);
	void ex_50(const icd *i);
	void ex_51(const icd *i);
	void ex_52(const icd *i);
	void ex_53(const icd *i);
	void ex_54(const icd *i);
	void ex_55(const icd *i);
	void ex_56(const icd *i);
	void ex_57(const icd *i);
	void ex_58(const icd *i);
	void ex_59(const icd *i);
	void ex_60(const icd *i);
	void ex_61(const icd *i);
	void ex_62(const icd *i);
	void ex_63(const icd *i);
	void ex_64(const icd *i);
	void ex_65(const icd *i);
	void ex_66(const icd *i);
	void ex_67(const icd *i);
	void ex_68(const icd *i);
	void ex_69(const icd *i);
	void ex_70(const icd *i);
	void ex_71(const icd *i);
	void ex_72(const icd *i);
	void ex_73(const icd *i);
	void ex_74(const icd *i);
	void ex_75(const icd *i);
	void ex_76(const icd *i);
	void ex_77(const icd *i);
	void ex_78(const icd *i);
	void ex_79(const icd *i);
	void ex_80(const icd *i);
	void ex_81(const icd *i);
	void ex_82(const icd *i);
	void ex_83(const icd *i);
	void ex_84(const icd *i);
	void ex_85(const icd *i);
	void ex_86(const icd *i);
	void ex_87(const icd *i);
	void ex_88(const icd *i);
	void ex_89(const icd *i);
	void ex_90(const icd *i);
	void ex_91(const icd *i);
	void ex_92(const icd *i);
	void ex_93(const icd *i);
	void ex_94(const icd *i);
	void ex_95(const icd *i);
	void ex_96(const icd *i);
	void ex_97(const icd *i);
	void ex_98(const icd *i);
	void ex_99(const icd *i);
	void ex_100(const icd *i);
	void ex_101(const icd *i);
	void ex_102(const icd *i);
	void ex_103(const icd *i);
	void ex_104(const icd *i);
	void ex_105(const icd *i);
	void ex_106(const icd *i);
	void ex_107(const icd *i);
	void ex_108(const icd *i);
	void ex_109(const icd *i);
	void ex_110(const icd *i);
	void ex_111(const icd *i);
	void ex_112(const icd *i);
	void ex_113(const icd *i);
	void ex_114(const icd *i);
	void ex_115(const icd *i);
	void ex_116(const icd *i);
	void ex_117(const icd *i);
	void ex_118(const icd *i);
	void ex_119(const icd *i);
	void ex_120(const icd *i);
	void ex_121(const icd *i);
	void ex_122(const icd *i);
	void ex_123(const icd *i);
	void ex_124(const icd *i);
	void ex_125(const icd *i);
	void ex_126(const icd *i);
	void ex_127(const icd *i);
	void ex_128(const icd *i);
	void ex_129(const icd *i);
	void ex_130(const icd *i);
	void ex_131(const icd *i);
	void ex_132(const icd *i);
	void ex_133(const icd *i);
	void ex_134(const icd *i);
	void ex_135(const icd *i);
	void ex_136(const icd *i);
	void ex_137(const icd *i);
	void ex_138(const icd *i);
	void ex_139(const icd *i);
	void ex_140(const icd *i);
	void ex_141(const icd *i);
	void ex_142(const icd *i);
	void ex_143(const icd *i);
	void ex_144(const icd *i);
	void ex_145(const icd *i);
	void ex_146(const icd *i);
	void ex_147(const icd *i);
	void ex_148(const icd *i);
	void ex_149(const icd *i);
	void ex_150(const icd *i);
	void ex_151(const icd *i);
	void ex_152(const icd *i);
	void ex_153(const icd *i);
	void ex_154(const icd *i);
	void ex_155(const icd *i);
	void ex_156(const icd *i);
	void ex_157(const icd *i);
	void ex_158(const icd *i);
	void ex_159(const icd *i);
	void ex_160(const icd *i);
	void ex_161(const icd *i);
	void ex_162(const icd *i);
	void ex_163(const icd *i);
	void ex_164(const icd *i);
	void ex_165(const icd *i);
	void ex_166(const icd *i);
	void ex_167(const icd *i);
	void ex_168(const icd *i);
	void ex_169(const icd *i);
	void ex_170(const icd *i);
	void ex_171(const icd *i);
	void ex_172(const icd *i);
	void ex_173(const icd *i);
	void ex_174(const icd *i);
	void ex_175(const icd *i);
	void ex_176(const icd *i);
	void ex_177(const icd *i);
	void ex_178(const icd *i);
	void ex_179(const icd *i);
	void ex_180(const icd *i);
	void ex_181(const icd *i);
	void ex_182(const icd *i);
	void ex_183(const icd *i);
	void ex_184(const icd *i);
	void ex_185(const icd *i);
	void ex_186(const icd *i);
	void ex_187(const icd *i);
	void ex_188(const icd *i);
	void ex_189(const icd *i);
	void ex_190(const icd *i);
	void ex_191(const icd *i);
	void ex_192(const icd *i);
	void ex_193(const icd *i);
	void ex_194(const icd *i);
	void ex_195(const icd *i);
	void ex_196(const icd *i);
	void ex_197(const icd *i);
	void ex_198(const icd *i);
	void ex_199(const icd *i);
	void ex_200(const icd *i);
	void ex_201(const icd *i);
	void ex_202(const icd *i);
	void ex_203(const icd *i);
	void ex_204(const icd *i);
	void ex_205(const icd *i);
	void ex_206(const icd *i);
	void ex_207(const icd *i);
	void ex_208(const icd *i);
	void ex_209(const icd *i);
	void ex_210(const icd *i);
	void ex_211(const icd *i);
	void ex_212(const icd *i);
	void ex_213(const icd *i);
	void ex_214(const icd *i);
	void ex_215(const icd *i);
	void ex_216(const icd *i);
	void ex_217(const icd *i);
	void ex_218(const icd *i);
	void ex_219(const icd *i);
	void ex_220(const icd *i);
	void ex_221(const icd *i);
	void ex_222(const icd *i);
	void ex_223(const icd *i);
	void ex_224(const icd *i);
	void ex_225(const icd *i);
	void ex_226(const icd *i);
	void ex_227(const icd *i);
	void ex_228(const icd *i);
	void ex_229(const icd *i);
	void ex_230(const icd *i);
	void ex_231(const icd *i);
	void ex_232(const icd *i);
	void ex_233(const icd *i);
	void ex_234(const icd *i);
	void ex_235(const icd *i);
	void ex_236(const icd *i);
	void ex_237(const icd *i);
	void ex_238(const icd *i);
	void ex_239(const icd *i);
	void ex_240(const icd *i);
	void ex_241(const icd *i);
	void ex_242(const icd *i);
	void ex_243(const icd *i);
	void ex_244(const icd *i);
	void ex_245(const icd *i);
	void ex_246(const icd *i);
	void ex_247(const icd *i);
	void ex_248(const icd *i);
	void ex_249(const icd *i);
	void ex_250(const icd *i);
	void ex_251(const icd *i);
	void ex_252(const icd *i);
	void ex_253(const icd *i);
	void ex_254(const icd *i);
	void ex_255(const icd *i);
	void ex_256(const icd *i);
	void ex_257(const icd *i);
	void ex_258(const icd *i);
	void ex_259(const icd *i);
	void ex_260(const icd *i);
	void ex_261(const icd *i);
	void ex_262(const icd *i);
	void ex_263(const icd *i);
	void ex_264(const icd *i);
	void ex_265(const icd *i);
	void ex_266(const icd *i);
	void ex_267(const icd *i);
	void ex_268(const icd *i);
	void ex_269(const icd *i);
	void ex_270(const icd *i);
	void ex_271(const icd *i);
	void ex_272(const icd *i);
	void ex_273(const icd *i);
	void ex_274(const icd *i);
	void ex_275(const icd *i);
	void ex_276(const icd *i);
	void ex_277(const icd *i);
	void ex_278(const icd *i);
	void ex_279(const icd *i);
	void ex_280(const icd *i);
	void ex_281(const icd *i);
	void ex_282(const icd *i);
	void ex_283(const icd *i);
	void ex_284(const icd *i);
	void ex_285(const icd *i);
	void ex_286(const icd *i);
	void ex_287(const icd *i);
	void ex_288(const icd *i);
	void ex_289(const icd *i);
	void ex_290(const icd *i);
	void ex_291(const icd *i);
	void ex_292(const icd *i);
	void ex_293(const icd *i);
	void ex_294(const icd *i);
	void ex_295(const icd *i);
	void ex_296(const icd *i);
	void ex_297(const icd *i);
	void ex_298(const icd *i);
	void ex_299(const icd *i);
	void ex_300(const icd *i);
	void ex_301(const icd *i);
	void ex_302(const icd *i);
	void ex_303(const icd *i);
	void ex_304(const icd *i);
	void ex_305(const icd *i);
	void ex_306(const icd *i);
	void ex_307(const icd *i);
	void ex_308(const icd *i);
	void ex_309(const icd *i);
	void ex_310(const icd *i);
	void ex_311(const icd *i);
	void ex_312(const icd *i);
	void ex_313(const icd *i);
	void ex_314(const icd *i);
	void ex_315(const icd *i);
	void ex_316(const icd *i);
	void ex_317(const icd *i);
	void ex_318(const icd *i);
	void ex_319(const icd *i);
	void ex_320(const icd *i);
	void ex_321(const icd *i);
	void ex_322(const icd *i);
	void ex_323(const icd *i);
	void ex_324(const icd *i);
	void ex_325(const icd *i);
	void ex_326(const icd *i);
	void ex_327(const icd *i);
	void ex_328(const icd *i);
	void ex_329(const icd *i);
	void ex_330(const icd *i);
	void ex_331(const icd *i);
	void ex_332(const icd *i);
	void ex_333(const icd *i);
	void ex_334(const icd *i);
	void ex_335(const icd *i);
	void ex_336(const icd *i);
	void ex_337(const icd *i);
	void ex_338(const icd *i);
	void ex_339(const icd *i);
	void ex_340(const icd *i);
	void ex_341(const icd *i);
	void ex_342(const icd *i);
	void ex_343(const icd *i);
	void ex_344(const icd *i);
	void ex_345(const icd *i);
	void ex_346(const icd *i);
	void ex_347(const icd *i);
	void ex_348(const icd *i);
	void ex_349(const icd *i);
	void ex_350(const icd *i);
	void ex_351(const icd *i);
	void ex_352(const icd *i);
	void ex_353(const icd *i);
	void ex_354(const icd *i);
	void ex_355(const icd *i);
	void ex_356(const icd *i);
	void ex_357(const icd *i);
	void ex_358(const icd *i);
	void ex_359(const icd *i);
	void ex_360(const icd *i);
	void ex_361(const icd *i);
	void ex_362(const icd *i);
	void ex_363(const icd *i);
	void ex_364(const icd *i);
	void ex_365(const icd *i);
	void ex_366(const icd *i);
	void ex_367(const icd *i);
	void ex_368(const icd *i);
	void ex_369(const icd *i);
	void ex_370(const icd *i);
	void ex_371(const icd *i);
	void ex_372(const icd *i);
	void ex_373(const icd *i);
	void ex_374(const icd *i);
	void ex_375(const icd *i);
	void ex_376(const icd *i);
	void ex_377(const icd *i);
	void ex_378(const icd *i);
	void ex_379(const icd *i);
	void ex_380(const icd *i);
	void ex_381(const icd *i);
	void ex_382(const icd *i);
	void ex_383(const icd *i);
	void ex_384(const icd *i);
	void ex_385(const icd *i);
	void ex_386(const icd *i);
	void ex_387(const icd *i);
	void ex_388(const icd *i);
	void ex_389(const icd *i);
	void ex_390(const icd *i);
	void ex_391(const icd *i);
	void ex_392(const icd *i);
	void ex_393(const icd *i);
	void ex_394(const icd *i);
	void ex_395(const icd *i);
	void ex_396(const icd *i);
	void ex_397(const icd *i);
	void ex_398(const icd *i);
	void ex_399(const icd *i);
	void ex_400(const icd *i);
	void ex_401(const icd *i);
	void ex_402(const icd *i);
	void ex_403(const icd *i);
	void ex_404(const icd *i);
	void ex_405(const icd *i);
	void ex_406(const icd *i);
	void ex_407(const icd *i);
	void ex_408(const icd *i);
	void ex_409(const icd *i);
	void ex_410(const icd *i);
	void ex_411(const icd *i);
	void ex_412(const icd *i);
	void ex_413(const icd *i);
	void ex_414(const icd *i);
	void ex_415(const icd *i);
	void ex_416(const icd *i);
	void ex_417(const icd *i);
	void ex_418(const icd *i);
	void ex_419(const icd *i);
	void ex_420(const icd *i);
	void ex_421(const icd *i);
	void ex_422(const icd *i);
	void ex_423(const icd *i);
	void ex_424(const icd *i);
	void ex_425(const icd *i);
	void ex_426(const icd *i);
	void ex_427(const icd *i);
	void ex_428(const icd *i);
	void ex_429(const icd *i);
	void ex_430(const icd *i);
	void ex_431(const icd *i);
	void ex_432(const icd *i);
	void ex_433(const icd *i);
	void ex_434(const icd *i);
	void ex_435(const icd *i);
	void ex_436(const icd *i);
	void ex_437(const icd *i);
	void ex_438(const icd *i);
	void ex_439(const icd *i);
	void ex_440(const icd *i);
	void ex_441(const icd *i);
	void ex_442(const icd *i);
	void ex_443(const icd *i);
	void ex_444(const icd *i);
	void ex_445(const icd *i);
	void ex_446(const icd *i);
	void ex_447(const icd *i);
	void ex_448(const icd *i);
	void ex_449(const icd *i);
	void ex_450(const icd *i);
	void ex_451(const icd *i);
	void ex_452(const icd *i);
	void ex_453(const icd *i);
	void ex_454(const icd *i);
	void ex_455(const icd *i);
	void ex_456(const icd *i);
	void ex_457(const icd *i);
	void ex_458(const icd *i);
	void ex_459(const icd *i);
	void ex_460(const icd *i);
	void ex_461(const icd *i);
	void ex_462(const icd *i);
	void ex_463(const icd *i);
	void ex_464(const icd *i);
	void ex_465(const icd *i);
	void ex_466(const icd *i);
	void ex_467(const icd *i);
	void ex_468(const icd *i);
	void ex_469(const icd *i);
	void ex_470(const icd *i);
	void ex_471(const icd *i);
	void ex_472(const icd *i);
	void ex_473(const icd *i);
	void ex_474(const icd *i);
	void ex_475(const icd *i);
	void ex_476(const icd *i);
	void ex_477(const icd *i);
	void ex_478(const icd *i);
	void ex_479(const icd *i);
	void ex_480(const icd *i);
	void ex_481(const icd *i);
	void ex_482(const icd *i);
	void ex_483(const icd *i);
	void ex_484(const icd *i);
	void ex_485(const icd *i);
	void ex_486(const icd *i);
	void ex_487(const icd *i);
	void ex_488(const icd *i);
	void ex_489(const icd *i);
	void ex_490(const icd *i);
	void ex_491(const icd *i);
	void ex_492(const icd *i);
	void ex_493(const icd *i);
	void ex_494(const icd *i);
	void ex_495(const icd *i);
	void ex_496(const icd *i);
	void ex_497(const icd *i);
	void ex_498(const icd *i);
	void ex_499(const icd *i);
	void ex_500(const icd *i);
	void ex_501(const icd *i);
	void ex_502(const icd *i);
	void ex_503(const icd *i);
	void ex_504(const icd *i);
	void ex_505(const icd *i);
	void ex_506(const icd *i);
	void ex_507(const icd *i);
	void ex_508(const icd *i);
	void ex_509(const icd *i);
	void ex_510(const icd *i);
	void ex_511(const icd *i);
	void ex_512(const icd *i);
	void ex_513(const icd *i);
	void ex_514(const icd *i);
	void ex_515(const icd *i);
	void ex_516(const icd *i);
	void ex_517(const icd *i);
	void ex_518(const icd *i);
	void ex_519(const icd *i);
	void ex_520(const icd *i);
	void ex_521(const icd *i);
	void ex_522(const icd *i);
	void ex_523(const icd *i);
	void ex_524(const icd *i);
	void ex_525(const icd *i);
	void ex_526(const icd *i);
	void ex_527(const icd *i);
	void ex_528(const icd *i);
	void ex_529(const icd *i);
	void ex_530(const icd *i);
	void ex_531(const icd *i);
	void ex_532(const icd *i);
	void ex_533(const icd *i);
	void ex_534(const icd *i);
	void ex_535(const icd *i);
	void ex_536(const icd *i);
	void ex_537(const icd *i);
	void ex_538(const icd *i);
	void ex_539(const icd *i);
	void ex_540(const icd *i);
	void ex_541(const icd *i);
	void ex_542(const icd *i);
	void ex_543(const icd *i);
	void ex_544(const icd *i);
	void ex_545(const icd *i);
	void ex_546(const icd *i);
	void ex_547(const icd *i);
	void ex_548(const icd *i);
	void ex_549(const icd *i);
	void ex_550(const icd *i);
	void ex_551(const icd *i);
	void ex_552(const icd *i);
	void ex_553(const icd *i);
	void ex_554(const icd *i);
	void ex_555(const icd *i);
	void ex_556(const icd *i);
	void ex_557(const icd *i);
	void ex_558(const icd *i);
	void ex_559(const icd *i);
	void ex_560(const icd *i);
	void ex_561(const icd *i);
	void ex_562(const icd *i);
	void ex_563(const icd *i);
	void ex_564(const icd *i);
	void ex_565(const icd *i);
	void ex_566(const icd *i);
	void ex_567(const icd *i);
	void ex_568(const icd *i);
	void ex_569(const icd *i);
	void ex_570(const icd *i);
	void ex_571(const icd *i);
	void ex_572(const icd *i);
	void ex_573(const icd *i);
	void ex_574(const icd *i);
	void ex_575(const icd *i);
	void ex_576(const icd *i);
	void ex_577(const icd *i);
	void ex_578(const icd *i);
	void ex_579(const icd *i);
	void ex_580(const icd *i);
	void ex_581(const icd *i);
	void ex_582(const icd *i);
	void ex_583(const icd *i);
	void ex_584(const icd *i);
	void ex_585(const icd *i);
	void ex_586(const icd *i);
	void ex_587(const icd *i);
	void ex_588(const icd *i);
	void ex_589(const icd *i);
	void ex_590(const icd *i);
	void ex_591(const icd *i);
	void ex_592(const icd *i);
	void ex_593(const icd *i);
	void ex_594(const icd *i);
	void ex_595(const icd *i);
	void ex_596(const icd *i);
	void ex_597(const icd *i);
	void ex_598(const icd *i);
	void ex_599(const icd *i);
	void ex_600(const icd *i);
	void ex_601(const icd *i);
	void ex_602(const icd *i);
	void ex_603(const icd *i);
	void ex_604(const icd *i);
	void ex_605(const icd *i);
	void ex_606(const icd *i);
	void ex_607(const icd *i);
	void ex_608(const icd *i);
	void ex_609(const icd *i);
	void ex_610(const icd *i);
	void ex_611(const icd *i);
	void ex_612(const icd *i);
	void ex_613(const icd *i);
	void ex_614(const icd *i);
	void ex_615(const icd *i);
	void ex_616(const icd *i);
	void ex_617(const icd *i);
	void ex_618(const icd *i);
	void ex_619(const icd *i);
	void ex_620(const icd *i);
	void ex_621(const icd *i);
	void ex_622(const icd *i);
	void ex_623(const icd *i);
	void ex_624(const icd *i);
	void ex_625(const icd *i);
	void ex_626(const icd *i);
	void ex_627(const icd *i);
	void ex_628(const icd *i);
	void ex_629(const icd *i);
	void ex_630(const icd *i);
	void ex_631(const icd *i);
	void ex_632(const icd *i);
	void ex_633(const icd *i);
	void ex_634(const icd *i);
	void ex_635(const icd *i);
	void ex_636(const icd *i);
	void ex_637(const icd *i);
	void ex_638(const icd *i);
	void ex_639(const icd *i);
	void ex_640(const icd *i);
	void ex_641(const icd *i);
	void ex_642(const icd *i);
	void ex_643(const icd *i);
	void ex_644(const icd *i);
	void ex_645(const icd *i);
	void ex_646(const icd *i);
	void ex_647(const icd *i);
	void ex_648(const icd *i);
	void ex_649(const icd *i);
	void ex_650(const icd *i);
	void ex_651(const icd *i);
	void ex_652(const icd *i);
	void ex_653(const icd *i);
	void ex_654(const icd *i);
	void ex_655(const icd *i);
	void ex_656(const icd *i);
	void ex_657(const icd *i);
	void ex_658(const icd *i);
	void ex_659(const icd *i);
	void ex_660(const icd *i);
	void ex_661(const icd *i);
	void ex_662(const icd *i);
	void ex_663(const icd *i);
	void ex_664(const icd *i);
	void ex_665(const icd *i);
	void ex_666(const icd *i);
	void ex_667(const icd *i);
	void ex_668(const icd *i);
	void ex_669(const icd *i);
	void ex_670(const icd *i);
	void ex_671(const icd *i);
	void ex_672(const icd *i);
	void ex_673(const icd *i);
	void ex_674(const icd *i);
	void ex_675(const icd *i);
	void ex_676(const icd *i);
	void ex_677(const icd *i);
	void ex_678(const icd *i);
	void ex_679(const icd *i);
	void ex_680(const icd *i);
	void ex_681(const icd *i);
	void ex_682(const icd *i);
	void ex_683(const icd *i);
	void ex_684(const icd *i);
	void ex_685(const icd *i);
	void ex_686(const icd *i);
	void ex_687(const icd *i);
	void ex_688(const icd *i);
	void ex_689(const icd *i);
	void ex_690(const icd *i);
	void ex_691(const icd *i);
	void ex_692(const icd *i);
	void ex_693(const icd *i);
	void ex_694(const icd *i);
	void ex_695(const icd *i);
	void ex_696(const icd *i);
	void ex_697(const icd *i);
	void ex_698(const icd *i);
	void ex_699(const icd *i);
	void ex_700(const icd *i);
	void ex_701(const icd *i);
	void ex_702(const icd *i);
	void ex_703(const icd *i);
	void ex_704(const icd *i);
	void ex_705(const icd *i);
	void ex_706(const icd *i);
	void ex_707(const icd *i);
	void ex_708(const icd *i);
	void ex_709(const icd *i);
	void ex_710(const icd *i);
	void ex_711(const icd *i);
	void ex_712(const icd *i);
	void ex_713(const icd *i);
	void ex_714(const icd *i);
	void ex_715(const icd *i);
	void ex_716(const icd *i);
	void ex_717(const icd *i);
	void ex_718(const icd *i);
	void ex_719(const icd *i);
	void ex_720(const icd *i);
	void ex_721(const icd *i);
	void ex_722(const icd *i);
	void ex_723(const icd *i);
	void ex_724(const icd *i);
	void ex_725(const icd *i);
	void ex_726(const icd *i);
	void ex_727(const icd *i);
	void ex_728(const icd *i);
	void ex_729(const icd *i);
	void ex_730(const icd *i);
	void ex_731(const icd *i);
	void ex_732(const icd *i);
	void ex_733(const icd *i);
	void ex_734(const icd *i);
	void ex_735(const icd *i);
	void ex_736(const icd *i);
	void ex_737(const icd *i);
	void ex_738(const icd *i);
	void ex_739(const icd *i);
	void ex_740(const icd *i);
	void ex_741(const icd *i);
	void ex_742(const icd *i);
	void ex_743(const icd *i);
	void ex_744(const icd *i);
	void ex_745(const icd *i);
	void ex_746(const icd *i);
	void ex_747(const icd *i);
	void ex_748(const icd *i);
	void ex_749(const icd *i);
	void ex_750(const icd *i);
	void ex_751(const icd *i);
	void ex_752(const icd *i);
	void ex_753(const icd *i);
	void ex_754(const icd *i);
	void ex_755(const icd *i);
	void ex_756(const icd *i);
	void ex_757(const icd *i);
	void ex_758(const icd *i);
	void ex_759(const icd *i);
	void ex_760(const icd *i);
	void ex_761(const icd *i);
	void ex_762(const icd *i);
	void ex_763(const icd *i);
	void ex_764(const icd *i);
	void ex_765(const icd *i);
	void ex_766(const icd *i);
	void ex_767(const icd *i);
	void ex_768(const icd *i);
	void ex_769(const icd *i);
	void ex_770(const icd *i);
	void ex_771(const icd *i);
	void ex_772(const icd *i);
	void ex_773(const icd *i);
	void ex_774(const icd *i);
	void ex_775(const icd *i);
	void ex_776(const icd *i);
	void ex_777(const icd *i);
	void ex_778(const icd *i);
	void ex_779(const icd *i);
	void ex_780(const icd *i);
	void ex_781(const icd *i);
	void ex_782(const icd *i);
	void ex_783(const icd *i);
	void ex_784(const icd *i);
	void ex_785(const icd *i);
	void ex_786(const icd *i);
	void ex_787(const icd *i);
	void ex_788(const icd *i);
	void ex_789(const icd *i);
	void ex_790(const icd *i);
	void ex_791(const icd *i);
	void ex_792(const icd *i);
	void ex_793(const icd *i);
	void ex_794(const icd *i);
	void ex_795(const icd *i);
	void ex_796(const icd *i);
	void ex_797(const icd *i);
	void ex_798(const icd *i);
	void ex_799(const icd *i);
	void ex_800(const icd *i);
	void ex_801(const icd *i);
	void ex_802(const icd *i);
	void ex_803(const icd *i);
	void ex_804(const icd *i);
	void ex_805(const icd *i);
	void ex_806(const icd *i);
	void ex_807(const icd *i);
	void ex_808(const icd *i);
	void ex_809(const icd *i);
	void ex_810(const icd *i);
	void ex_811(const icd *i);
	void ex_812(const icd *i);
	void ex_813(const icd *i);
	void ex_814(const icd *i);
	void ex_815(const icd *i);
	void ex_816(const icd *i);
	void ex_817(const icd *i);
	void ex_818(const icd *i);
	void ex_819(const icd *i);
	void ex_820(const icd *i);
	void ex_821(const icd *i);
	void ex_822(const icd *i);
	void ex_823(const icd *i);
	void ex_824(const icd *i);
	void ex_825(const icd *i);
	void ex_826(const icd *i);
	void ex_827(const icd *i);
	void ex_828(const icd *i);
	void ex_829(const icd *i);
	void ex_830(const icd *i);
	void ex_831(const icd *i);
	void ex_832(const icd *i);
	void ex_833(const icd *i);
	void ex_834(const icd *i);
	void ex_835(const icd *i);
	void ex_836(const icd *i);
	void ex_837(const icd *i);
	void ex_838(const icd *i);
	void ex_839(const icd *i);
	void ex_840(const icd *i);
	void ex_841(const icd *i);
	void ex_842(const icd *i);
	void ex_843(const icd *i);
	void ex_844(const icd *i);
	void ex_845(const icd *i);
	void ex_846(const icd *i);
	void ex_847(const icd *i);
	void ex_848(const icd *i);
	void ex_849(const icd *i);
	void ex_850(const icd *i);
	void ex_851(const icd *i);
	void ex_852(const icd *i);
	void ex_853(const icd *i);
	void ex_854(const icd *i);
	void ex_855(const icd *i);
	void ex_856(const icd *i);
	void ex_857(const icd *i);
	void ex_858(const icd *i);
	void ex_859(const icd *i);
	void ex_860(const icd *i);
	void ex_861(const icd *i);
	void ex_862(const icd *i);
	void ex_863(const icd *i);
	void ex_864(const icd *i);
	void ex_865(const icd *i);
	void ex_866(const icd *i);
	void ex_867(const icd *i);
	void ex_868(const icd *i);
	void ex_869(const icd *i);
	void ex_870(const icd *i);
	void ex_871(const icd *i);
	void ex_872(const icd *i);
	void ex_873(const icd *i);
	void ex_874(const icd *i);
	void ex_875(const icd *i);
	void ex_876(const icd *i);
	void ex_877(const icd *i);
	void ex_878(const icd *i);
	void ex_879(const icd *i);
	void ex_880(const icd *i);
	void ex_881(const icd *i);
	void ex_882(const icd *i);
	void ex_883(const icd *i);
	void ex_884(const icd *i);
	void ex_885(const icd *i);
	void ex_886(const icd *i);
	void ex_887(const icd *i);
	void ex_888(const icd *i);
	void ex_889(const icd *i);
	void ex_890(const icd *i);
	void ex_891(const icd *i);
	void ex_892(const icd *i);
	void ex_893(const icd *i);
	void ex_894(const icd *i);
	void ex_895(const icd *i);
	void ex_896(const icd *i);
	void ex_897(const icd *i);
	void ex_898(const icd *i);
	void ex_899(const icd *i);
	void ex_900(const icd *i);
	void ex_901(const icd *i);
	void ex_902(const icd *i);
	void ex_903(const icd *i);
	void ex_904(const icd *i);
	void ex_905(const icd *i);
	void ex_906(const icd *i);
	void ex_907(const icd *i);
	void ex_908(const icd *i);
	void ex_909(const icd *i);
	void ex_910(const icd *i);
	void ex_911(const icd *i);
	void ex_912(const icd *i);
	void ex_913(const icd *i);
	void ex_914(const icd *i);
	void ex_915(const icd *i);
	void ex_916(const icd *i);
	void ex_917(const icd *i);
	void ex_918(const icd *i);
	void ex_919(const icd *i);
	void ex_920(const icd *i);
	void ex_921(const icd *i);
	void ex_922(const icd *i);
	void ex_923(const icd *i);
	void ex_924(const icd *i);
	void ex_925(const icd *i);
	void ex_926(const icd *i);
	void ex_927(const icd *i);
	void ex_928(const icd *i);
	void ex_929(const icd *i);
	void ex_930(const icd *i);
	void ex_931(const icd *i);
	void ex_932(const icd *i);
	void ex_933(const icd *i);
	void ex_934(const icd *i);
	void ex_935(const icd *i);
	void ex_936(const icd *i);
	void ex_937(const icd *i);
	void ex_938(const icd *i);
	void ex_939(const icd *i);
	void ex_940(const icd *i);
	void ex_941(const icd *i);
	void ex_942(const icd *i);
	void ex_943(const icd *i);
	void ex_944(const icd *i);
	void ex_945(const icd *i);
	void ex_946(const icd *i);
	void ex_947(const icd *i);
	void ex_948(const icd *i);
	void ex_949(const icd *i);
	void ex_950(const icd *i);
	void ex_951(const icd *i);
	void ex_952(const icd *i);
	void ex_953(const icd *i);
	void ex_954(const icd *i);
	void ex_955(const icd *i);
	void ex_956(const icd *i);
	void ex_957(const icd *i);
	void ex_958(const icd *i);
	void ex_959(const icd *i);
	void ex_960(const icd *i);
	void ex_961(const icd *i);
	void ex_962(const icd *i);
	void ex_963(const icd *i);
	void ex_964(const icd *i);
	void ex_965(const icd *i);
	void ex_966(const icd *i);
	void ex_967(const icd *i);
	void ex_968(const icd *i);
	void ex_969(const icd *i);
	void ex_970(const icd *i);
	void ex_971(const icd *i);
	void ex_972(const icd *i);
	void ex_973(const icd *i);
	void ex_974(const icd *i);
	void ex_975(const icd *i);
	void ex_976(const icd *i);
	void ex_977(const icd *i);
	void ex_978(const icd *i);
	void ex_979(const icd *i);
	void ex_980(const icd *i);
	void ex_981(const icd *i);
	void ex_982(const icd *i);
	void ex_983(const icd *i);
	void ex_984(const icd *i);
	void ex_985(const icd *i);
	void ex_986(const icd *i);
	void ex_987(const icd *i);
	void ex_988(const icd *i);
	void ex_989(const icd *i);
	void ex_990(const icd *i);
	void ex_991(const icd *i);
	void ex_992(const icd *i);
	void ex_993(const icd *i);
	void ex_994(const icd *i);
	void ex_995(const icd *i);
	void ex_996(const icd *i);
	void ex_997(const icd *i);
	void ex_998(const icd *i);
	void ex_999(const icd *i);
	void ex_1000(const icd *i);
	void ex_1001(const icd *i);
	void ex_1002(const icd *i);
	void ex_1003(const icd *i);
	void ex_1004(const icd *i);
	void ex_1005(const icd *i);
	void ex_1006(const icd *i);
	void ex_1007(const icd *i);
	void ex_1008(const icd *i);
	void ex_1009(const icd *i);
	void ex_1010(const icd *i);
	void ex_1011(const icd *i);
	void ex_1012(const icd *i);
	void ex_1013(const icd *i);
	void ex_1014(const icd *i);
	void ex_1015(const icd *i);
	void ex_1016(const icd *i);
	void ex_1017(const icd *i);
	void ex_1018(const icd *i);
	void ex_1019(const icd *i);
	void ex_1020(const icd *i);
	void ex_1021(const icd *i);
	void ex_1022(const icd *i);
	void ex_1023(const icd *i);
	void ex_1024(const icd *i);
	void ex_1025(const icd *i);
	void ex_1026(const icd *i);
	void ex_1027(const icd *i);
	void ex_1028(const icd *i);
	void ex_1029(const icd *i);
	void ex_1030(const icd *i);
	void ex_1031(const icd *i);
	void ex_1032(const icd *i);
	void ex_1033(const icd *i);
	void ex_1034(const icd *i);
	void ex_1035(const icd *i);
	void ex_1036(const icd *i);
	void ex_1037(const icd *i);
	void ex_1038(const icd *i);
	void ex_1039(const icd *i);
	void ex_1040(const icd *i);
	void ex_1041(const icd *i);
	void ex_1042(const icd *i);
	void ex_1043(const icd *i);
	void ex_1044(const icd *i);
	void ex_1045(const icd *i);
	void ex_1046(const icd *i);
	void ex_1047(const icd *i);
	void ex_1048(const icd *i);
	void ex_1049(const icd *i);
	void ex_1050(const icd *i);
	void ex_1051(const icd *i);
	void ex_1052(const icd *i);
	void ex_1053(const icd *i);
	void ex_1054(const icd *i);
	void ex_1055(const icd *i);
	void ex_1056(const icd *i);
	void ex_1057(const icd *i);
	void ex_1058(const icd *i);
	void ex_1059(const icd *i);
	void ex_1060(const icd *i);
	void ex_1061(const icd *i);
	void ex_1062(const icd *i);
	void ex_1063(const icd *i);
	void ex_1064(const icd *i);
	void ex_1065(const icd *i);
	void ex_1066(const icd *i);
	void ex_1067(const icd *i);
	void ex_1068(const icd *i);
	void ex_1069(const icd *i);
	void ex_1070(const icd *i);
	void ex_1071(const icd *i);
	void ex_1072(const icd *i);
	void ex_1073(const icd *i);
	void ex_1074(const icd *i);
	void ex_1075(const icd *i);
	void ex_1076(const icd *i);
	void ex_1077(const icd *i);
	void ex_1078(const icd *i);
	void ex_1079(const icd *i);
	void ex_1080(const icd *i);
	void ex_1081(const icd *i);
	void ex_1082(const icd *i);
	void ex_1083(const icd *i);
	void ex_1084(const icd *i);
	void ex_1085(const icd *i);
	void ex_1086(const icd *i);
	void ex_1087(const icd *i);
	void ex_1088(const icd *i);
	void ex_1089(const icd *i);
	void ex_1090(const icd *i);
	void ex_1091(const icd *i);
	void ex_1092(const icd *i);
	void ex_1093(const icd *i);
	void ex_1094(const icd *i);
	void ex_1095(const icd *i);
	void ex_1096(const icd *i);
	void ex_1097(const icd *i);
	void ex_1098(const icd *i);
	void ex_1099(const icd *i);
	void ex_1100(const icd *i);
	void ex_1101(const icd *i);
	void ex_1102(const icd *i);
	void ex_1103(const icd *i);
	void ex_1104(const icd *i);
	void ex_1105(const icd *i);
	void ex_1106(const icd *i);
	void ex_1107(const icd *i);
	void ex_1108(const icd *i);
	void ex_1109(const icd *i);
	void ex_1110(const icd *i);
	void ex_1111(const icd *i);
	void ex_1112(const icd *i);
	void ex_1113(const icd *i);
	void ex_1114(const icd *i);
	void ex_1115(const icd *i);
	void ex_1116(const icd *i);
	void ex_1117(const icd *i);
	void ex_1118(const icd *i);
	void ex_1119(const icd *i);
	void ex_1120(const icd *i);
	void ex_1121(const icd *i);
	void ex_1122(const icd *i);
	void ex_1123(const icd *i);
	void ex_1124(const icd *i);
	void ex_1125(const icd *i);
	void ex_1126(const icd *i);
	void ex_1127(const icd *i);
	void ex_1128(const icd *i);
	void ex_1129(const icd *i);
	void ex_1130(const icd *i);
	void ex_1131(const icd *i);
	void ex_1132(const icd *i);
	void ex_1133(const icd *i);
	void ex_1134(const icd *i);
	void ex_1135(const icd *i);
	void ex_1136(const icd *i);
	void ex_1137(const icd *i);
	void ex_1138(const icd *i);
	void ex_1139(const icd *i);
	void ex_1140(const icd *i);
	void ex_1141(const icd *i);
	void ex_1142(const icd *i);
	void ex_1143(const icd *i);
	void ex_1144(const icd *i);
	void ex_1145(const icd *i);
	void ex_1146(const icd *i);
	void ex_1147(const icd *i);
	void ex_1148(const icd *i);
	void ex_1149(const icd *i);
	void ex_1150(const icd *i);
	void ex_1151(const icd *i);
	void ex_1152(const icd *i);
	void ex_1153(const icd *i);
	void ex_1154(const icd *i);
	void ex_1155(const icd *i);
	void ex_1156(const icd *i);
	void ex_1157(const icd *i);
	void ex_1158(const icd *i);
	void ex_1159(const icd *i);
	void ex_1160(const icd *i);
	void ex_1161(const icd *i);
	void ex_1162(const icd *i);
	void ex_1163(const icd *i);
	void ex_1164(const icd *i);
	void ex_1165(const icd *i);
	void ex_1166(const icd *i);
	void ex_1167(const icd *i);
	void ex_1168(const icd *i);
	void ex_1169(const icd *i);
	void ex_1170(const icd *i);
	void ex_1171(const icd *i);
	void ex_1172(const icd *i);
	void ex_1173(const icd *i);
	void ex_1174(const icd *i);
	void ex_1175(const icd *i);
	void ex_1176(const icd *i);
	void ex_1177(const icd *i);
	void ex_1178(const icd *i);
	void ex_1179(const icd *i);
	void ex_1180(const icd *i);
	void ex_1181(const icd *i);
	void ex_1182(const icd *i);
	void ex_1183(const icd *i);
	void ex_1184(const icd *i);
	void ex_1185(const icd *i);
	void ex_1186(const icd *i);
	void ex_1187(const icd *i);
	void ex_1188(const icd *i);
	void ex_1189(const icd *i);
	void ex_1190(const icd *i);
	void ex_1191(const icd *i);
	void ex_1192(const icd *i);
	void ex_1193(const icd *i);
	void ex_1194(const icd *i);
	void ex_1195(const icd *i);
	void ex_1196(const icd *i);
	void ex_1197(const icd *i);
	void ex_1198(const icd *i);
	void ex_1199(const icd *i);
	void ex_1200(const icd *i);
	void ex_1201(const icd *i);
	void ex_1202(const icd *i);
	void ex_1203(const icd *i);
	void ex_1204(const icd *i);
	void ex_1205(const icd *i);
	void ex_1206(const icd *i);
	void ex_1207(const icd *i);
	void ex_1208(const icd *i);
	void ex_1209(const icd *i);
	void ex_1210(const icd *i);
	void ex_1211(const icd *i);
	void ex_1212(const icd *i);
	void ex_1213(const icd *i);
	void ex_1214(const icd *i);
	void ex_1215(const icd *i);
	void ex_1216(const icd *i);
	void ex_1217(const icd *i);
	void ex_1218(const icd *i);
	void ex_1219(const icd *i);
	void ex_1220(const icd *i);
	void ex_1221(const icd *i);
	void ex_1222(const icd *i);
	void ex_1223(const icd *i);
	void ex_1224(const icd *i);
	void ex_1225(const icd *i);
	void ex_1226(const icd *i);
	void ex_1227(const icd *i);
	void ex_1228(const icd *i);
	void ex_1229(const icd *i);
	void ex_1230(const icd *i);
	void ex_1231(const icd *i);
	void ex_1232(const icd *i);
	void ex_1233(const icd *i);
	void ex_1234(const icd *i);
	void ex_1235(const icd *i);
	void ex_1236(const icd *i);
	void ex_1237(const icd *i);
	void ex_1238(const icd *i);
	void ex_1239(const icd *i);
	void ex_1240(const icd *i);
	void ex_1241(const icd *i);
	void ex_1242(const icd *i);
	void ex_1243(const icd *i);
	void ex_1244(const icd *i);
	void ex_1245(const icd *i);
	void ex_1246(const icd *i);
	void ex_1247(const icd *i);
	void ex_1248(const icd *i);
	void ex_1249(const icd *i);
	void ex_1250(const icd *i);
	void ex_1251(const icd *i);
	void ex_1252(const icd *i);
	void ex_1253(const icd *i);
	void ex_1254(const icd *i);
	void ex_1255(const icd *i);
	void ex_1256(const icd *i);
	void ex_1257(const icd *i);
	void ex_1258(const icd *i);
	void ex_1259(const icd *i);
	void ex_1260(const icd *i);
	void ex_1261(const icd *i);
	void ex_1262(const icd *i);
	void ex_1263(const icd *i);
	void ex_1264(const icd *i);
	void ex_1265(const icd *i);
	void ex_1266(const icd *i);
	void ex_1267(const icd *i);
	void ex_1268(const icd *i);
	void ex_1269(const icd *i);
	void ex_1270(const icd *i);
	void ex_1271(const icd *i);
	void ex_1272(const icd *i);
	void ex_1273(const icd *i);
	void ex_1274(const icd *i);
	void ex_1275(const icd *i);
	void ex_1276(const icd *i);
	void ex_1277(const icd *i);
	void ex_1278(const icd *i);
	void ex_1279(const icd *i);
	void ex_1280(const icd *i);
	void ex_1281(const icd *i);
	void ex_1282(const icd *i);
	void ex_1283(const icd *i);
	void ex_1284(const icd *i);
	void ex_1285(const icd *i);
	void ex_1286(const icd *i);
	void ex_1287(const icd *i);
	void ex_1288(const icd *i);
	void ex_1289(const icd *i);
	void ex_1290(const icd *i);
	void ex_1291(const icd *i);
	void ex_1292(const icd *i);
	void ex_1293(const icd *i);
	void ex_1294(const icd *i);
	void ex_1295(const icd *i);
	void ex_1296(const icd *i);
	void ex_1297(const icd *i);
	void ex_1298(const icd *i);
	void ex_1299(const icd *i);
	void ex_1300(const icd *i);
	void ex_1301(const icd *i);
	void ex_1302(const icd *i);
	void ex_1303(const icd *i);
	void ex_1304(const icd *i);
	void ex_1305(const icd *i);
	void ex_1306(const icd *i);
	void ex_1307(const icd *i);
	void ex_1308(const icd *i);
	void ex_1309(const icd *i);
	void ex_1310(const icd *i);
	void ex_1311(const icd *i);
	void ex_1312(const icd *i);
	void ex_1313(const icd *i);
	void ex_1314(const icd *i);
	void ex_1315(const icd *i);
	void ex_1316(const icd *i);
	void ex_1317(const icd *i);
	void ex_1318(const icd *i);
	void ex_1319(const icd *i);
	void ex_1320(const icd *i);
	void ex_1321(const icd *i);
	void ex_1322(const icd *i);
	void ex_1323(const icd *i);
	void ex_1324(const icd *i);
	void ex_1325(const icd *i);
	void ex_1326(const icd *i);
	void ex_1327(const icd *i);
	void ex_1328(const icd *i);
	void ex_1329(const icd *i);
	void ex_1330(const icd *i);
	void ex_1331(const icd *i);
	void ex_1332(const icd *i);
	void ex_1333(const icd *i);
	void ex_1334(const icd *i);
	void ex_1335(const icd *i);
	void ex_1336(const icd *i);
	void ex_1337(const icd *i);
	void ex_1338(const icd *i);
	void ex_1339(const icd *i);
	void ex_1340(const icd *i);
	void ex_1341(const icd *i);
	void ex_1342(const icd *i);
	void ex_1343(const icd *i);
	void ex_1344(const icd *i);
	void ex_1345(const icd *i);
	void ex_1346(const icd *i);
	void ex_1347(const icd *i);
	void ex_1348(const icd *i);
	void ex_1349(const icd *i);
	void ex_1350(const icd *i);
	void ex_1351(const icd *i);
	void ex_1352(const icd *i);
	void ex_1353(const icd *i);
	void ex_1354(const icd *i);
	void ex_1355(const icd *i);
	void ex_1356(const icd *i);
	void ex_1357(const icd *i);
	void ex_1358(const icd *i);
	void ex_1359(const icd *i);
	void ex_1360(const icd *i);
	void ex_1361(const icd *i);
	void ex_1362(const icd *i);
	void ex_1363(const icd *i);
	void ex_1364(const icd *i);
	void ex_1365(const icd *i);
	void ex_1366(const icd *i);
	void ex_1367(const icd *i);
	void ex_1368(const icd *i);
	void ex_1369(const icd *i);
	void ex_1370(const icd *i);
	void ex_1371(const icd *i);
	void ex_1372(const icd *i);
	void ex_1373(const icd *i);
	void ex_1374(const icd *i);
	void ex_1375(const icd *i);
	void ex_1376(const icd *i);
	void ex_1377(const icd *i);
	void ex_1378(const icd *i);
	void ex_1379(const icd *i);
	void ex_1380(const icd *i);
	void ex_1381(const icd *i);
	void ex_1382(const icd *i);
	void ex_1383(const icd *i);
	void ex_1384(const icd *i);
	void ex_1385(const icd *i);
	void ex_1386(const icd *i);
	void ex_1387(const icd *i);
	void ex_1388(const icd *i);
	void ex_1389(const icd *i);
	void ex_1390(const icd *i);
	void ex_1391(const icd *i);
	void ex_1392(const icd *i);
	void ex_1393(const icd *i);
	void ex_1394(const icd *i);
	void ex_1395(const icd *i);
	void ex_1396(const icd *i);
	void ex_1397(const icd *i);
	void ex_1398(const icd *i);
	void ex_1399(const icd *i);
	void ex_1400(const icd *i);
	void ex_1401(const icd *i);
	void ex_1402(const icd *i);
	void ex_1403(const icd *i);
	void ex_1404(const icd *i);
	void ex_1405(const icd *i);
	void ex_1406(const icd *i);
	void ex_1407(const icd *i);
	void ex_1408(const icd *i);
	void ex_1409(const icd *i);
	void ex_1410(const icd *i);
	void ex_1411(const icd *i);
	void ex_1412(const icd *i);
	void ex_1413(const icd *i);
	void ex_1414(const icd *i);
	void ex_1415(const icd *i);
	void ex_1416(const icd *i);
	void ex_1417(const icd *i);
	void ex_1418(const icd *i);
	void ex_1419(const icd *i);
	void ex_1420(const icd *i);
	void ex_1421(const icd *i);
	void ex_1422(const icd *i);
	void ex_1423(const icd *i);
	void ex_1424(const icd *i);
	void ex_1425(const icd *i);
	void ex_1426(const icd *i);
	void ex_1427(const icd *i);
	void ex_1428(const icd *i);
	void ex_1429(const icd *i);
	void ex_1430(const icd *i);
	void ex_1431(const icd *i);
	void ex_1432(const icd *i);
	void ex_1433(const icd *i);
	void ex_1434(const icd *i);
	void ex_1435(const icd *i);
	void ex_1436(const icd *i);
	void ex_1437(const icd *i);
	void ex_1438(const icd *i);
	void ex_1439(const icd *i);
	void ex_1440(const icd *i);
	void ex_1441(const icd *i);
	void ex_1442(const icd *i);
	void ex_1443(const icd *i);
	void ex_1444(const icd *i);
	void ex_1445(const icd *i);
	void ex_1446(const icd *i);
	void ex_1447(const icd *i);
	void ex_1448(const icd *i);
	void ex_1449(const icd *i);
	void ex_1450(const icd *i);
	void ex_1451(const icd *i);
	void ex_1452(const icd *i);
	void ex_1453(const icd *i);
	void ex_1454(const icd *i);
	void ex_1455(const icd *i);
	void ex_1456(const icd *i);
	void ex_1457(const icd *i);
	void ex_1458(const icd *i);
	void ex_1459(const icd *i);
	void ex_1460(const icd *i);
	void ex_1461(const icd *i);
	void ex_1462(const icd *i);
	void ex_1463(const icd *i);
	void ex_1464(const icd *i);
	void ex_1465(const icd *i);
	void ex_1466(const icd *i);
	void ex_1467(const icd *i);
	void ex_1468(const icd *i);
	void ex_1469(const icd *i);
	void ex_1470(const icd *i);
	void ex_1471(const icd *i);
	void ex_1472(const icd *i);
	void ex_1473(const icd *i);
	void ex_1474(const icd *i);
	void ex_1475(const icd *i);
	void ex_1476(const icd *i);
	void ex_1477(const icd *i);
	void ex_1478(const icd *i);
	void ex_1479(const icd *i);
	void ex_1480(const icd *i);
	void ex_1481(const icd *i);
	void ex_1482(const icd *i);
	void ex_1483(const icd *i);
	void ex_1484(const icd *i);
	void ex_1485(const icd *i);
	void ex_1486(const icd *i);
	void ex_1487(const icd *i);
	void ex_1488(const icd *i);
	void ex_1489(const icd *i);
	void ex_1490(const icd *i);
	void ex_1491(const icd *i);
	void ex_1492(const icd *i);
	void ex_1493(const icd *i);
	void ex_1494(const icd *i);
	void ex_1495(const icd *i);
	void ex_1496(const icd *i);
	void ex_1497(const icd *i);
	void ex_1498(const icd *i);
	void ex_1499(const icd *i);
	void ex_1500(const icd *i);
	void ex_1501(const icd *i);
	void ex_1502(const icd *i);
	void ex_1503(const icd *i);
	void ex_1504(const icd *i);
	void ex_1505(const icd *i);
	void ex_1506(const icd *i);
	void ex_1507(const icd *i);
	void ex_1508(const icd *i);
	void ex_1509(const icd *i);
	void ex_1510(const icd *i);
	void ex_1511(const icd *i);
	void ex_1512(const icd *i);
	void ex_1513(const icd *i);
	void ex_1514(const icd *i);
	void ex_1515(const icd *i);
	void ex_1516(const icd *i);
	void ex_1517(const icd *i);
	void ex_1518(const icd *i);
	void ex_1519(const icd *i);
	void ex_1520(const icd *i);
	void ex_1521(const icd *i);
	void ex_1522(const icd *i);
	void ex_1523(const icd *i);
	void ex_1524(const icd *i);
	void ex_1525(const icd *i);
	void ex_1526(const icd *i);
	void ex_1527(const icd *i);
	void ex_1528(const icd *i);
	void ex_1529(const icd *i);
	void ex_1530(const icd *i);
	void ex_1531(const icd *i);
	void ex_1532(const icd *i);
	void ex_1533(const icd *i);
	void ex_1534(const icd *i);
	void ex_1535(const icd *i);
	void ex_1536(const icd *i);
	void ex_1537(const icd *i);
	void ex_1538(const icd *i);
	void ex_1539(const icd *i);
	void ex_1540(const icd *i);
	void ex_1541(const icd *i);
	void ex_1542(const icd *i);
	void ex_1543(const icd *i);
	void ex_1544(const icd *i);
	void ex_1545(const icd *i);
	void ex_1546(const icd *i);
	void ex_1547(const icd *i);
	void ex_1548(const icd *i);
	void ex_1549(const icd *i);
	void ex_1550(const icd *i);
	void ex_1551(const icd *i);
	void ex_1552(const icd *i);
	void ex_1553(const icd *i);
	void ex_1554(const icd *i);
	void ex_1555(const icd *i);
	void ex_1556(const icd *i);
	void ex_1557(const icd *i);
	void ex_1558(const icd *i);
	void ex_1559(const icd *i);
	void ex_1560(const icd *i);
	void ex_1561(const icd *i);
	void ex_1562(const icd *i);
	void ex_1563(const icd *i);
	void ex_1564(const icd *i);
	void ex_1565(const icd *i);
	void ex_1566(const icd *i);
	void ex_1567(const icd *i);
	void ex_1568(const icd *i);
	void ex_1569(const icd *i);
	void ex_1570(const icd *i);
	void ex_1571(const icd *i);
	void ex_1572(const icd *i);
	void ex_1573(const icd *i);
	void ex_1574(const icd *i);
	void ex_1575(const icd *i);
	void ex_1576(const icd *i);
	void ex_1577(const icd *i);
	void ex_1578(const icd *i);
	void ex_1579(const icd *i);
	void ex_1580(const icd *i);
	void ex_1581(const icd *i);
	void ex_1582(const icd *i);
	void ex_1583(const icd *i);
	void ex_1584(const icd *i);
	void ex_1585(const icd *i);
	void ex_1586(const icd *i);
	void ex_1587(const icd *i);
	void ex_1588(const icd *i);
	void ex_1589(const icd *i);
	void ex_1590(const icd *i);
	void ex_1591(const icd *i);
	void ex_1592(const icd *i);
	void ex_1593(const icd *i);
	void ex_1594(const icd *i);
	void ex_1595(const icd *i);
	void ex_1596(const icd *i);
	void ex_1597(const icd *i);
	void ex_1598(const icd *i);
	void ex_1599(const icd *i);
	void ex_1600(const icd *i);
	void ex_1601(const icd *i);
	void ex_1602(const icd *i);
	void ex_1603(const icd *i);
	void ex_1604(const icd *i);
	void ex_1605(const icd *i);
	void ex_1606(const icd *i);
	void ex_1607(const icd *i);
	void ex_1608(const icd *i);
	void ex_1609(const icd *i);
	void ex_1610(const icd *i);
	void ex_1611(const icd *i);
	void ex_1612(const icd *i);
	void ex_1613(const icd *i);
	void ex_1614(const icd *i);
	void ex_1615(const icd *i);
	void ex_1616(const icd *i);
	void ex_1617(const icd *i);
	void ex_1618(const icd *i);
	void ex_1619(const icd *i);
	void ex_1620(const icd *i);
	void ex_1621(const icd *i);
	void ex_1622(const icd *i);
	void ex_1623(const icd *i);
	void ex_1624(const icd *i);
	void ex_1625(const icd *i);
	void ex_1626(const icd *i);
	void ex_1627(const icd *i);
	void ex_1628(const icd *i);
	void ex_1629(const icd *i);
	void ex_1630(const icd *i);
	void ex_1631(const icd *i);
	void ex_1632(const icd *i);
	void ex_1633(const icd *i);
	void ex_1634(const icd *i);
	void ex_1635(const icd *i);
	void ex_1636(const icd *i);
	void ex_1637(const icd *i);
	void ex_1638(const icd *i);
	void ex_1639(const icd *i);
	void ex_1640(const icd *i);
	void ex_1641(const icd *i);
	void ex_1642(const icd *i);
	void ex_1643(const icd *i);
	void ex_1644(const icd *i);
	void ex_1645(const icd *i);
	void ex_1646(const icd *i);
	void ex_1647(const icd *i);
	void ex_1648(const icd *i);
	void ex_1649(const icd *i);
	void ex_1650(const icd *i);
	void ex_1651(const icd *i);
	void ex_1652(const icd *i);
	void ex_1653(const icd *i);
	void ex_1654(const icd *i);
	void ex_1655(const icd *i);
	void ex_1656(const icd *i);
	void ex_1657(const icd *i);
	void ex_1658(const icd *i);
	void ex_1659(const icd *i);
	void ex_1660(const icd *i);
	void ex_1661(const icd *i);
	void ex_1662(const icd *i);
	void ex_1663(const icd *i);
	void ex_1664(const icd *i);
	void ex_1665(const icd *i);
	void ex_1666(const icd *i);
	void ex_1667(const icd *i);
	void ex_1668(const icd *i);
	void ex_1669(const icd *i);
	void ex_1670(const icd *i);
	void ex_1671(const icd *i);
	void ex_1672(const icd *i);
	void ex_1673(const icd *i);
	void ex_1674(const icd *i);
	void ex_1675(const icd *i);
	void ex_1676(const icd *i);
	void ex_1677(const icd *i);
	void ex_1678(const icd *i);
	void ex_1679(const icd *i);
	void ex_1680(const icd *i);
	void ex_1681(const icd *i);
	void ex_1682(const icd *i);
	void ex_1683(const icd *i);
	void ex_1684(const icd *i);
	void ex_1685(const icd *i);
	void ex_1686(const icd *i);
	void ex_1687(const icd *i);
	void ex_1688(const icd *i);
	void ex_1689(const icd *i);
	void ex_1690(const icd *i);
	void ex_1691(const icd *i);
	void ex_1692(const icd *i);
	void ex_1693(const icd *i);
	void ex_1694(const icd *i);
	void ex_1695(const icd *i);
	void ex_1696(const icd *i);
	void ex_1697(const icd *i);
	void ex_1698(const icd *i);
	void ex_1699(const icd *i);
	void ex_1700(const icd *i);
	void ex_1701(const icd *i);
	void ex_1702(const icd *i);
	void ex_1703(const icd *i);
	void ex_1704(const icd *i);
	void ex_1705(const icd *i);
	void ex_1706(const icd *i);
	void ex_1707(const icd *i);
	void ex_1708(const icd *i);
	void ex_1709(const icd *i);
	void ex_1710(const icd *i);
	void ex_1711(const icd *i);
	void ex_1712(const icd *i);
	void ex_1713(const icd *i);
	void ex_1714(const icd *i);
	void ex_1715(const icd *i);
	void ex_1716(const icd *i);
	void ex_1717(const icd *i);
	void ex_1718(const icd *i);
	void ex_1719(const icd *i);
	void ex_1720(const icd *i);
	void ex_1721(const icd *i);
	void ex_1722(const icd *i);
	void ex_1723(const icd *i);
	void ex_1724(const icd *i);
	void ex_1725(const icd *i);
	void ex_1726(const icd *i);
	void ex_1727(const icd *i);
	void ex_1728(const icd *i);
	void ex_1729(const icd *i);
	void ex_1730(const icd *i);
	void ex_1731(const icd *i);
	void ex_1732(const icd *i);
	void ex_1733(const icd *i);
	void ex_1734(const icd *i);
	void ex_1735(const icd *i);
	void ex_1736(const icd *i);
	void ex_1737(const icd *i);
	void ex_1738(const icd *i);
	void ex_1739(const icd *i);
	void ex_1740(const icd *i);
	void ex_1741(const icd *i);
	void ex_1742(const icd *i);
	void ex_1743(const icd *i);
	void ex_1744(const icd *i);
	void ex_1745(const icd *i);
	void ex_1746(const icd *i);
	void ex_1747(const icd *i);
	void ex_1748(const icd *i);
	void ex_1749(const icd *i);
	void ex_1750(const icd *i);
	void ex_1751(const icd *i);
	void ex_1752(const icd *i);
	void ex_1753(const icd *i);
	void ex_1754(const icd *i);
	void ex_1755(const icd *i);
	void ex_1756(const icd *i);
	void ex_1757(const icd *i);
	void ex_1758(const icd *i);
	void ex_1759(const icd *i);
	void ex_1760(const icd *i);
	void ex_1761(const icd *i);
	void ex_1762(const icd *i);
	void ex_1763(const icd *i);
	void ex_1764(const icd *i);
	void ex_1765(const icd *i);
	void ex_1766(const icd *i);
	void ex_1767(const icd *i);
	void ex_1768(const icd *i);
	void ex_1769(const icd *i);
	void ex_1770(const icd *i);
	void ex_1771(const icd *i);
	void ex_1772(const icd *i);
	void ex_1773(const icd *i);
	void ex_1774(const icd *i);
	void ex_1775(const icd *i);
	void ex_1776(const icd *i);
	void ex_1777(const icd *i);
	void ex_1778(const icd *i);
	void ex_1779(const icd *i);
	void ex_1780(const icd *i);
	void ex_1781(const icd *i);
	void ex_1782(const icd *i);
	void ex_1783(const icd *i);
	void ex_1784(const icd *i);
	void ex_1785(const icd *i);
	void ex_1786(const icd *i);
	void ex_1787(const icd *i);
	void ex_1788(const icd *i);
	void ex_1789(const icd *i);
	void ex_1790(const icd *i);
	void ex_1791(const icd *i);
	void ex_1792(const icd *i);
	void ex_1793(const icd *i);
	void ex_1794(const icd *i);
	void ex_1795(const icd *i);
	void ex_1796(const icd *i);
	void ex_1797(const icd *i);
	void ex_1798(const icd *i);
	void ex_1799(const icd *i);
	void ex_1800(const icd *i);
	void ex_1801(const icd *i);
	void ex_1802(const icd *i);
	void ex_1803(const icd *i);
	void ex_1804(const icd *i);
	void ex_1805(const icd *i);
	void ex_1806(const icd *i);
	void ex_1807(const icd *i);
	void ex_1808(const icd *i);
	void ex_1809(const icd *i);
	void ex_1810(const icd *i);
	void ex_1811(const icd *i);
	void ex_1812(const icd *i);
	void ex_1813(const icd *i);
	void ex_1814(const icd *i);
	void ex_1815(const icd *i);
	void ex_1816(const icd *i);
	void ex_1817(const icd *i);
	void ex_1818(const icd *i);
	void ex_1819(const icd *i);
	void ex_1820(const icd *i);
	void ex_1821(const icd *i);
	void ex_1822(const icd *i);
	void ex_1823(const icd *i);
	void ex_1824(const icd *i);
	void ex_1825(const icd *i);
	void ex_1826(const icd *i);
	void ex_1827(const icd *i);
	void ex_1828(const icd *i);
	void ex_1829(const icd *i);
	void ex_1830(const icd *i);
	void ex_1831(const icd *i);
	void ex_1832(const icd *i);
	void ex_1833(const icd *i);
	void ex_1834(const icd *i);
	void ex_1835(const icd *i);
	void ex_1836(const icd *i);
	void ex_1837(const icd *i);
	void ex_1838(const icd *i);
	void ex_1839(const icd *i);
	void ex_1840(const icd *i);
	void ex_1841(const icd *i);
	void ex_1842(const icd *i);
	void ex_1843(const icd *i);
	void ex_1844(const icd *i);
	void ex_1845(const icd *i);
	void ex_1846(const icd *i);
	void ex_1847(const icd *i);
	void ex_1848(const icd *i);
	void ex_1849(const icd *i);
	void ex_1850(const icd *i);
	void ex_1851(const icd *i);
	void ex_1852(const icd *i);
	void ex_1853(const icd *i);
	void ex_1854(const icd *i);
	void ex_1855(const icd *i);
	void ex_1856(const icd *i);
	void ex_1857(const icd *i);
	void ex_1858(const icd *i);
	void ex_1859(const icd *i);
	void ex_1860(const icd *i);
	void ex_1861(const icd *i);
	void ex_1862(const icd *i);
	void ex_1863(const icd *i);
	void ex_1864(const icd *i);

	// --- pooled dynarec ownership (added at the END of the class: prior member offsets — which the
	// JIT TU bakes via the jit_off_* accessors — stay untouched across emu.h layout variants) ---
	bool m_dynarec_default = false;             // machine-config default (set_dynarec_default; env overrides)
	std::unique_ptr<tms57002::Jit> m_jit;       // per-device Jit, created lazily at the execute_run hook
	};

enum {
	TMS57002_PC=1,
	TMS57002_AACC,
	TMS57002_BA0,
	TMS57002_BA1,
	TMS57002_CREG,
	TMS57002_CA,
	TMS57002_DREG,
	TMS57002_ID,
	TMS57002_MACC,
	TMS57002_HIDX,
	TMS57002_HOST0,
	TMS57002_HOST1,
	TMS57002_HOST2,
	TMS57002_HOST3,
	TMS57002_RPTC,
	TMS57002_SA,
	TMS57002_ST0,
	TMS57002_ST1,
	TMS57002_TREG,
	TMS57002_XBA,
	TMS57002_XOA,
	TMS57002_XRD,
	TMS57002_XWR
};

DECLARE_DEVICE_TYPE(TMS57002, tms57002_device)

#endif // MAME_CPU_TMS57002_TMS57002_H
