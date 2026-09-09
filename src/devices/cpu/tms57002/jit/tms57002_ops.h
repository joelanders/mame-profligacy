// license:BSD-3-Clause
//
// TMS57002 dynarec op-emitter (milestone 1). One entry point that emits a STANDALONE, memory-based
// asmjit body for a single op-id (the interpreter's ex_N, reproduced bit-exact), so the per-op gate
// (test/tms57002_ops_test.cpp) can validate each op in isolation against the oracle.
//
// This is the semantic layer: correctness only, state via the device struct (no register residency).
// Milestone 2 re-emits the SAME per-mnemonic lowerings under a pinned-register private convention as
// a shared op-body pool; keeping the lowerings here (descriptor-driven, one place) lets both share.
//
// The op-id -> (mnemonic, variant bits) mapping comes from the generated CINTRPDESC descriptor table
// (tmsmake.py), so this dispatches on decomposed bits (cmode/dmode/sfmo/...) that the runtime
// jit_op_variant() index does not expose. Flag-setter ops are handled by probing the oracle at emit
// time (jit_probe_st1) rather than hard-coding the ST1_* field layout.
//
// 2026-07-07: dual-backend. x86-64 keeps emit_op_mem (M1) + emit_op_pooled; aarch64 gets a pooled
// emitter ONLY (emit_op_mem's per-op gate lives in the profligacy standalone tree — port it there
// if that tree ever migrates). op_poolable / tms_op_desc_count are arch-independent and shared, so
// the two backends can never diverge on COVERAGE — only on lowerings, which the byte-identity gate
// (scripts/korgprophecy_pf4_gate.sh) checks.
#pragma once

#include "emu.h"        // runtime/compat typedefs (u8, u32, ...) — tms57002.h needs them
#include "tms57002.h"   // tms57002_device

#if defined(__x86_64__) || defined(__aarch64__)

#include <cstdint>

namespace tms57002 {

// Number of generated op-ids in the descriptor table (for coverage reporting / bounds).
unsigned tms_op_desc_count();

// True if op `op` has a pooled body (emit_op_pooled will emit it). Ops without one deopt to the
// interp. ARCH-INDEPENDENT: both backends emit exactly this set (coverage parity by construction).
bool op_poolable(tms57002_device &dsp, unsigned op);
bool op_reads_cmem(tms57002_device &dsp, unsigned op);   // poolable op whose body reads cmem (deopt under pending)

// F5 refinement support: which ST1_CACHE mode fields did DECODE consume to select op `op`?
// Computed from the generated descriptor table and shared by both pooled backends.
u32 op_mode_consumed_mask(unsigned op);

// True if `op` is a decode-time st1 mode-setter (descriptor type 'f').
bool op_is_mode_setter(unsigned op);

}  // namespace tms57002

#endif  // __x86_64__ || __aarch64__

#if defined(__x86_64__)
#include <asmjit/x86.h>

namespace tms57002 {

// Emit op `op` (op >= 4) into cc, with the device pointer in `dev`. `param` is the instruction's
// immediate operand (jit_inst_param); `icd_addr` is the runtime icd pointer (jit_inst_addr), used
// for emit-time oracle probes and for deopt calls. Returns false if the op-id is not yet natively
// emitted — the caller (gate) skips it; the eventual dynarec deopts it to the interpreter.
bool emit_op_mem(asmjit::x86::Compiler &cc, asmjit::x86::Gp dev,
                 tms57002_device &dsp, unsigned op, unsigned param, std::uint64_t icd_addr);

// ---- Milestone 2: pooled (register-resident) op bodies -------------------------------------------
// The M2 dynarec pins hot state in callee-saved registers across the frame and calls a SHARED op-body
// pool. These bodies are the same semantics as emit_op_mem, but operate on pinned registers instead of
// device memory, and read the per-instruction operand from `param` (a scratch reg) so ONE body serves
// every PC of an op-id. They assume jit_pooled_safe() (no pending cmem update etc.) — so a cmem read is
// a direct cmem[addr], with NO C-calls in the body (no stack-alignment / scratch-clobber hazards).
struct PoolRegs {
	asmjit::x86::Gp dev;      // device base (callee-saved, e.g. rbx)
	asmjit::x86::Gp macc;     // s64 macc          (callee-saved, e.g. r12)
	asmjit::x86::Gp macc_r;   // s64 macc_read     (callee-saved, e.g. r13)
	asmjit::x86::Gp macc_w;   // s64 macc_write    (callee-saved, e.g. r14)
	asmjit::x86::Gp aacc;     // u32 aacc (use .r32) (callee-saved, e.g. r15)
	asmjit::x86::Gp param;    // u8 instruction operand, per-call (scratch, e.g. r11)
};

// Emit op `op`'s pooled body into `a` (NO prologue/epilogue/ret — the caller frames it). Operates on
// `r` (pinned regs) + caller-saved scratch (rax/rcx/rdx/rsi/rdi/r8/r9/r10). `icd_addr` is a
// representative instruction descriptor (used only for the emit-time flag-setter probe). Precondition:
// op_poolable(dsp, op).
void emit_op_pooled(asmjit::x86::Assembler &a, const PoolRegs &r, tms57002_device &dsp,
                    unsigned op, std::uint64_t icd_addr);

}  // namespace tms57002

#elif defined(__aarch64__)
#include <asmjit/a64.h>

namespace tms57002 {

// a64 pooled convention (2026-07-07 port).
// Same shape as the x86 PoolRegs: hot state pinned in callee-saved regs, per-instruction operand in
// a caller-saved reg the driver loads before each `bl`. Bodies make NO C-calls and end in `ret`
// (x30 = the driver's bl return address; bodies must not clobber x30).
struct PoolRegs {
	asmjit::a64::Gp dev;      // device base    (callee-saved, x19)
	asmjit::a64::Gp macc;     // s64 macc       (callee-saved, x20)
	asmjit::a64::Gp macc_r;   // s64 macc_read  (callee-saved, x21)
	asmjit::a64::Gp macc_w;   // s64 macc_write (callee-saved, x22)
	asmjit::a64::Gp aacc;     // u32 aacc — operate on .w() (callee-saved, x23)
	asmjit::a64::Gp param;    // u8 instruction operand, per-call (caller-saved, w15; driver writes
	                          // w15 so x15's upper 32 bits are always zero)
};

// Emit op `op`'s pooled body into `a` (NO prologue/epilogue/ret — the caller frames it). Operates
// on `r` (pinned regs) + caller-saved scratch x8-x14 (x16/x17 left to veneers, x30 to the driver).
// `icd_addr` is a representative instruction descriptor (emit-time flag-setter probe only).
// Precondition: op_poolable(dsp, op). Semantics MUST match the x86 emitter bit-for-bit — the
// byte-identity gate is the arbiter.
void emit_op_pooled(asmjit::a64::Assembler &a, const PoolRegs &r, tms57002_device &dsp,
                    unsigned op, std::uint64_t icd_addr);

}  // namespace tms57002

#endif  // __x86_64__ / __aarch64__
