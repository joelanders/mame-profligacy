#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""ROM-free regression for the H8/3003 external-bus timing model.

The arithmetic under test is not copied into Python.  This runner extracts the
exact production C++ implementations of the H8/3003 reset, bus-control register
accessors, and memory_access_cycles, compiles them in a minimal type-compatible
shim, and executes focused vectors.  It also checks the integration points that
the shim cannot exercise: native register mapping/save-state wiring, charging
all generic H8 memory paths through the virtual hook, and the Prophecy's
default-on timing configuration with an explicit diagnostic opt-out.

This deliberately proves a source-level timing contract without requiring Korg
ROMs or private logic-analyser captures.  It does not emulate WAIT-pin-low
extensions (the Prophecy ties WAIT high) or establish end-to-end
MIDI-to-host-bus phase.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


class TestFailure(RuntimeError):
    """A regression contract was not satisfied."""


def extract_method(source: str, qualified_name: str) -> str:
    """Return one complete out-of-class C++ method definition."""

    match = re.search(
        rf"(?m)^[^\n]*\b{re.escape(qualified_name)}\s*\([^\n]*\)\s*(?:const\s*)?\{{",
        source,
    )
    if not match:
        raise TestFailure(f"production method not found: {qualified_name}")

    start = match.start()
    opening = source.find("{", match.start(), match.end())
    depth = 0
    in_string: str | None = None
    escaped = False
    line_comment = False
    block_comment = False

    index = opening
    while index < len(source):
        char = source[index]
        following = source[index + 1] if index + 1 < len(source) else ""

        if line_comment:
            if char == "\n":
                line_comment = False
            index += 1
            continue
        if block_comment:
            if char == "*" and following == "/":
                block_comment = False
                index += 2
            else:
                index += 1
            continue
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == in_string:
                in_string = None
            index += 1
            continue

        if char == "/" and following == "/":
            line_comment = True
            index += 2
            continue
        if char == "/" and following == "*":
            block_comment = True
            index += 2
            continue
        # Apostrophes between digits are C++ numeric separators, not the
        # beginning of character literals (for example XTAL(24'576'000)).
        previous = source[index - 1] if index else ""
        if char == "'" and previous.isdigit() and following.isdigit():
            index += 1
            continue
        if char in {'"', "'"}:
            in_string = char
            index += 1
            continue
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
        index += 1

    raise TestFailure(f"unterminated production method: {qualified_name}")


def extract_timer_callback(source: str, qualified_name: str) -> str:
    """Return one TIMER_CALLBACK_MEMBER definition."""

    marker = f"TIMER_CALLBACK_MEMBER({qualified_name})"
    start = source.find(marker)
    if start < 0:
        raise TestFailure(f"production timer callback not found: {qualified_name}")
    opening = source.find("{", start + len(marker))
    if opening < 0:
        raise TestFailure(f"timer callback has no body: {qualified_name}")

    depth = 0
    index = opening
    while index < len(source):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
        index += 1
    raise TestFailure(f"unterminated timer callback: {qualified_name}")


def compact(source: str) -> str:
    """Remove whitespace for small, formatting-insensitive wiring checks."""

    return re.sub(r"\s+", "", source)


def require_tokens(source: str, description: str, tokens: list[str]) -> None:
    compact_source = compact(source)
    missing = [token for token in tokens if compact(token) not in compact_source]
    if missing:
        rendered = "\n  ".join(missing)
        raise TestFailure(f"{description} is incomplete; missing:\n  {rendered}")


def check_static_wiring(root: Path, h83003_cpp: str, h83003_h: str) -> None:
    require_tokens(
        h83003_cpp,
        "H8/3003 bus-control register map",
        [
            "map(base | 0xffec, base | 0xffec).rw(FUNC(h83003_device::abwcr_r), FUNC(h83003_device::abwcr_w));",
            "map(base | 0xffed, base | 0xffed).rw(FUNC(h83003_device::astcr_r), FUNC(h83003_device::astcr_w));",
            "map(base | 0xffee, base | 0xffee).rw(FUNC(h83003_device::wcr_r), FUNC(h83003_device::wcr_w));",
            "map(base | 0xffef, base | 0xffef).rw(FUNC(h83003_device::wcer_r), FUNC(h83003_device::wcer_w));",
            "map(base | 0xfff1, base | 0xfff1).r(FUNC(h83003_device::mdcr_r));",
            "map(base | 0xfff3, base | 0xfff3).rw(FUNC(h83003_device::brcr_r), FUNC(h83003_device::brcr_w));",
        ],
    )
    require_tokens(
        h83003_cpp,
        "H8/3003 RAME-controlled internal RAM view",
        [
            "map(base | 0xfd10, base | 0xff0f).view(m_ram_view);",
            "m_ram_view[0](base | 0xfd10, base | 0xff0f).ram().share(m_internal_ram);",
            "m_ram_view.select(0);",
            "m_ram_view.disable();",
            "m_intc->set_nmi_edge(BIT(data, 2));",
            "m_standby_pending = BIT(data, 7);",
        ],
    )
    require_tokens(
        h83003_cpp,
        "H8/3003 bus-control save-state registration",
        [
            "save_item(NAME(m_abwcr));",
            "save_item(NAME(m_astcr));",
            "save_item(NAME(m_wcr));",
            "save_item(NAME(m_wcer));",
            "save_item(NAME(m_brcr));",
        ],
    )
    require_tokens(
        h83003_h,
        "H8/3003 timing API",
        [
            "void set_mode_a20(bool initial_bus_16bit = false) { m_mode_a20 = true; m_initial_bus_16bit = initial_bus_16bit; }",
            "void set_external_bus_timing(bool enable) { m_external_bus_timing = enable; }",
            "virtual int reset_processing_cycles() const override;",
            "virtual int interrupt_priority_cycles() const override;",
			"virtual void interrupt_priority_complete() override;",
            "virtual bool interrupt_post_accept_prefetch() const override;",
            "virtual bool internal_phase_checkpointing_enabled() const override;",
            "virtual int dma_bus_acquisition_cycles(int channel) const override;",
            "virtual int memory_access_cycles(u32 address, int size) const override;",
            "bool m_initial_bus_16bit = false;",
            "bool m_external_bus_timing = false;",
            "memory_view m_ram_view;",
        ],
    )

    h8_h = (root / "src/devices/cpu/h8/h8.h").read_text(encoding="utf-8")
    require_tokens(
        h8_h,
        "generic H8 timing hooks",
        [
            "virtual int reset_processing_cycles() const;",
            "virtual int interrupt_priority_cycles() const;",
			"virtual void interrupt_priority_complete();",
            "virtual bool interrupt_post_accept_prefetch() const;",
            "virtual bool internal_phase_checkpointing_enabled() const;",
            "virtual int dma_bus_acquisition_cycles(int channel) const;",
            "virtual int memory_access_cycles(u32 address, int size) const;",
            "void begin_dma_bus_cycle(int channel);",
            "void charge_memory_access(u32 address, int size);",
            "void refund_memory_access();",
            "m_dma_bus_owner",
            "m_last_memory_access_cycles",
        ],
    )

    h8_cpp = (root / "src/devices/cpu/h8/h8.cpp").read_text(encoding="utf-8")
    require_tokens(
        h8_cpp,
        "generic H8 retry-charge lifecycle",
        [
            "save_item(NAME(m_dma_bus_owner));",
            "save_item(NAME(m_last_memory_access_cycles));",
        ],
    )
    h8_reset = extract_method(h8_cpp, "h8_device::device_reset")
    if compact("m_last_memory_access_cycles = 0;") not in compact(h8_reset):
        raise TestFailure("H8 reset does not clear deferred-access charge state")
    if compact("m_dma_bus_owner = -1;") not in compact(h8_reset):
        raise TestFailure("H8 reset does not release saved DMAC bus ownership")
    charged_methods = {
        "h8_device::read16i": "charge_memory_access(adr, 2);",
        "h8_device::read8": "charge_memory_access(adr, 1);",
        "h8_device::write8": "charge_memory_access(adr, 1);",
        "h8_device::read16": "charge_memory_access(adr, 2);",
        "h8_device::write16": "charge_memory_access(adr, 2);",
    }
    for method, charge in charged_methods.items():
        definition = extract_method(h8_cpp, method)
        if compact(charge) not in compact(definition):
            raise TestFailure(f"{method} does not charge through memory_access_cycles")

    charge_method = extract_method(h8_cpp, "h8_device::charge_memory_access")
    require_tokens(
        charge_method,
        "generic H8 exact memory-access charge tracking",
        [
            "m_last_memory_access_cycles = memory_access_cycles(address, size);",
            "m_icount -= m_last_memory_access_cycles;",
        ],
    )
    refund_method = extract_method(h8_cpp, "h8_device::refund_memory_access")
    if compact("m_icount += m_last_memory_access_cycles;") not in compact(refund_method):
        raise TestFailure("H8 deferred-access retry does not refund the exact bus charge")

    base_irq_priority = extract_method(h8_cpp, "h8_device::interrupt_priority_cycles")
    if compact("return 0;") not in compact(base_irq_priority):
        raise TestFailure(
            "generic H8 devices must preserve their prior IRQ totals unless they opt in"
        )
    base_irq_complete = extract_method(h8_cpp, "h8_device::interrupt_priority_complete")
    if "m_taken_irq_vector" in base_irq_complete or "m_taken_irq_level" in base_irq_complete:
        raise TestFailure(
            "generic H8 devices must not reselect an interrupt without opting in"
        )
    base_irq_postfetch = extract_method(
        h8_cpp, "h8_device::interrupt_post_accept_prefetch"
    )
    if compact("return false;") not in compact(base_irq_postfetch):
        raise TestFailure(
            "generic H8 devices must not gain an unverified discarded IRQ fetch"
        )
    base_internal_checkpoints = extract_method(
        h8_cpp, "h8_device::internal_phase_checkpointing_enabled"
    )
    if compact("return false;") not in compact(base_internal_checkpoints):
        raise TestFailure(
            "generic H8 devices must preserve instruction-boundary event visibility"
        )
    base_reset_processing = extract_method(h8_cpp, "h8_device::reset_processing_cycles")
    if compact("return 0;") not in compact(base_reset_processing):
        raise TestFailure(
            "generic H8 devices must preserve their prior reset totals unless they opt in"
        )
    base_dma_acquisition = extract_method(
        h8_cpp, "h8_device::dma_bus_acquisition_cycles"
    )
    if compact("return 0;") not in compact(base_dma_acquisition):
        raise TestFailure(
            "generic H8 devices must preserve prior DMA totals unless they opt in"
        )
    begin_dma = extract_method(h8_cpp, "h8_device::begin_dma_bus_cycle")
    require_tokens(
        begin_dma,
        "DMA bus-owner transition accounting",
        [
            "const int acquisition_cycles = dma_bus_acquisition_cycles(channel);",
            "if(acquisition_cycles && m_dma_bus_owner != channel)",
            "m_icount -= acquisition_cycles;",
            "m_dma_bus_owner = channel;",
            "else if(!acquisition_cycles)",
            "m_dma_bus_owner = -1;",
        ],
    )
    prefetch_done = extract_method(h8_cpp, "h8_device::prefetch_done")
    require_tokens(
        prefetch_done,
        "DMA bus release on return to CPU execution",
        [
            "if(m_inst_state != STATE_DMA)",
            "m_dma_bus_owner = -1;",
        ],
    )
    if any(token in compact(prefetch_done) for token in ("m_icount", "internal(")):
        raise TestFailure(
            "DMA bus release must not charge a trailing Td; H8/3003 figure 8-15 "
            "shows the CPU cycle immediately after the final destination write"
        )

    h8make = (root / "src/devices/cpu/h8/h8make.py").read_text(encoding="utf-8")
    require_tokens(
        h8make,
        "H8 generator retry refund selection",
        [
            'return "refund_memory_access();"',
            'print("\\t\\t\\t%s" % memory_access_refund(line), file=f)',
            'print("\\t\\t\\tabort_timeslice();", file=f)',
            '# GT913 indirect-bank helpers',
            'return "m_icount++;"',
            "return None",
            "def has_memory(ins):",
            "return memory_access_refund(ins) is not None",
            "def has_timed_phase(ins):",
            "def timed_phase_checkpoint_condition(ins):",
            'return "internal_phase_checkpointing_enabled() && m_icount <= m_bcount"',
            'or "reset_processing_cycles()" in ins',
            'or "interrupt_priority_cycles()" in ins',
            'or "begin_dma_bus_cycle(" in ins',
        ],
    )
    if h8make.count('print("\\t\\t\\t%s" % memory_access_refund(line), file=f)') != 2:
        raise TestFailure("both H8 full/partial retry paths must select the correct refund")
    if h8make.count('print("\\t\\t\\tabort_timeslice();", file=f)') != 2:
        raise TestFailure("both H8 retry paths must yield after refunding an access")
    if h8make.count(
        'print("\\tif(%s) { m_inst_substate = %d; return; }" % (timed_phase_checkpoint_condition(line), substate), file=f)'
    ) != 2:
        raise TestFailure("both H8 full/partial paths must checkpoint timed phases")

    h8_lst = (root / "src/devices/cpu/h8/h8.lst").read_text(encoding="utf-8")
    irq_charge = "m_icount -= interrupt_priority_cycles();"
    if h8_lst.count(irq_charge) != 2:
        raise TestFailure(
            "both H8 interrupt-entry variants must charge through "
            "interrupt_priority_cycles"
        )
    if h8_lst.count("interrupt_priority_complete();") != 2:
        raise TestFailure(
            "both H8 interrupt-entry variants must finalize the winner after the priority phase"
        )
    if h8_lst.count("if(interrupt_post_accept_prefetch()) {") != 2:
        raise TestFailure(
            "both H8 interrupt-entry variants must gate the post-accept fetch"
        )
    reset_charge = "m_icount -= reset_processing_cycles();"
    if h8_lst.count(reset_charge) != 1:
        raise TestFailure(
            "H8 reset entry must charge the device-specific post-vector processing phase"
        )

    driver_cpp = (root / "src/mame/korg/korgprophecy.cpp").read_text(encoding="utf-8")
    prophecy_config = extract_method(driver_cpp, "korgprophecy_state::prophecy")
    require_tokens(
        prophecy_config,
        "Korg Prophecy default-on native H8 bus timing",
        [
            "subcpu.set_mode_a20(true);",
            "subcpu.set_external_bus_timing(!std::getenv(\"KPROP_H8_DISABLE_NATIVE_BUS_TIMING\"));",
        ],
    )

    if "set_perfect_quantum" in prophecy_config:
        raise TestFailure(
            "Prophecy must not enable perfect H8 quantum by default; it prevents real-time operation"
        )

    h8_map = extract_method(driver_cpp, "korgprophecy_state::h8_map")
    if "h8_busctrl" in h8_map or "0x0fffec" in compact(h8_map).lower():
        raise TestFailure(
            "Prophecy h8_map still shadows the H8/3003 native FFEC-FFEF registers"
        )

def check_generated_irq_sequence(definition: str, label: str) -> None:
    priority_position = definition.find("m_icount -= interrupt_priority_cycles();")
    priority_complete_position = definition.find("interrupt_priority_complete();")
    discarded_position = definition.find("m_PIR = read16i(m_PC);")
    discarded_pc_increment = definition.find("m_PC += 2;", discarded_position)
    first_internal_position = definition.find("internal(1);", discarded_position)
    stack_position = definition.find("write16(", first_internal_position)
    setup_position = definition.find("irq_setup();", stack_position)
    vector_position = definition.find("read16i(", setup_position)
    second_internal_position = definition.find("internal(1);", vector_position)
    ack_position = definition.find("interrupt_taken();", second_internal_position)
    isr_prefetch_position = definition.rfind("m_PIR = read16i(m_PC);")
    if not (
        definition.count("m_PIR = read16i(m_PC);") == 2
        and 0
        <= priority_position
        < priority_complete_position
        < discarded_position
        < discarded_pc_increment
        < first_internal_position
        < stack_position
        < setup_position
        < vector_position
        < second_internal_position
        < ack_position
        < isr_prefetch_position
    ):
        raise TestFailure(
            f"generated {label} must order priority, discarded fetch, internal, "
            "stack, mask, vector, internal, controller ack, and ISR prefetch"
        )
    if "if(m_icount <= m_bcount)" not in definition[
        priority_position:discarded_position
    ]:
        raise TestFailure(
            f"generated {label} can replay priority or cross its event boundary"
        )
    internal_phases = list(re.finditer(r"internal\([^;\n]*\);", definition))
    for phase in internal_phases:
        if not re.match(
            r"\s*if\(internal_phase_checkpointing_enabled\(\) && m_icount <= m_bcount\)",
            definition[phase.end() :],
        ):
            raise TestFailure(
                f"generated {label} lacks a continuation after an internal phase"
            )


def check_generated_retry_paths(root: Path) -> None:
    """Run h8make and validate the generated retry branches, not just its source."""

    generator = root / "src/devices/cpu/h8/h8make.py"
    cases = (
        ("h8.lst", "o", False),
        ("h8.lst", "h", False),
        ("h8.lst", "s20", False),
        ("h8.lst", "s26", False),
        ("gt913.lst", "g", True),
    )
    with tempfile.TemporaryDirectory(prefix="h8make-retry-") as temp_name:
        temp = Path(temp_name)
        for listing_name, chip_type, expects_indirect_refunds in cases:
            output = temp / f"{chip_type}.hxx"
            result = subprocess.run(
                [
                    sys.executable,
                    str(generator),
                    str(generator.with_name(listing_name)),
                    "s",
                    chip_type,
                    str(output),
                ],
                cwd=root,
                check=False,
                text=True,
                capture_output=True,
            )
            if result.returncode:
                raise TestFailure(
                    f"h8make failed for {listing_name}:\n"
                    + result.stdout
                    + result.stderr
                )

            generated = output.read_text(encoding="utf-8")
            retry_count = generated.count("access_to_be_redone()")
            exact_refund_count = generated.count("refund_memory_access();")
            indirect_refund_count = generated.count("m_icount++;")
            actual_memory_count = sum(
                generated.count(helper)
                for helper in (
                    "read16i(",
                    "read16(",
                    "read8(",
                    "write16(",
                    "write8(",
                    "read16ib(",
                    "read8ib(",
                    "write16ib(",
                    "write8ib(",
                )
            )
            retry_abort_count = generated.count("abort_timeslice();")
            if retry_count == 0 or retry_count != exact_refund_count + indirect_refund_count:
                raise TestFailure(
                    f"generated {listing_name} retry branches are not all accounted for: "
                    f"retries={retry_count}, exact={exact_refund_count}, "
                    f"indirect={indirect_refund_count}"
                )
            if retry_count != actual_memory_count:
                raise TestFailure(
                    f"generated {listing_name}/{chip_type} classifies non-memory "
                    f"helpers as retryable accesses: retries={retry_count}, "
                    f"actual memory calls={actual_memory_count}"
                )
            if retry_abort_count != retry_count:
                raise TestFailure(
                    f"generated {listing_name} does not yield after every retry refund: "
                    f"retries={retry_count}, aborts={retry_abort_count}"
                )
            for redo_body in re.findall(
                r"if\(access_to_be_redone\(\)\)\s*\{(.*?)\n\s*\}",
                generated,
                re.DOTALL,
            ):
                refund_position = max(
                    redo_body.find("refund_memory_access();"),
                    redo_body.find("m_icount++;"),
                )
                abort_position = redo_body.find("abort_timeslice();")
                substate_position = redo_body.find("m_inst_substate")
                if not 0 <= refund_position < abort_position < substate_position:
                    raise TestFailure(
                        f"generated {listing_name} retry order is not refund/yield/substate"
                    )
            if expects_indirect_refunds != bool(indirect_refund_count):
                raise TestFailure(
                    f"generated {listing_name} has unexpected GT913 indirect refunds: "
                    f"{indirect_refund_count}"
                )
            if listing_name == "h8.lst" and chip_type == "o":
                for reset_method in ("state_reset_full", "state_reset_partial"):
                    definition = extract_method(
                        generated, f"h8_device::{reset_method}"
                    )
                    require_tokens(
                        definition,
                        f"generated {reset_method} post-vector processing charge",
                        ["m_icount -= reset_processing_cycles();"],
                    )
                    reset_charge_position = definition.find(
                        "m_icount -= reset_processing_cycles();"
                    )
                    reset_filter_position = definition.find(
                        "update_irq_filter();", reset_charge_position
                    )
                    if "if(m_icount <= m_bcount)" not in definition[
                        reset_charge_position:reset_filter_position
                    ]:
                        raise TestFailure(
                            f"generated {reset_method} lacks a post-reset-phase checkpoint"
                        )
                for irq_method in ("state_irq_full", "state_irq_partial"):
                    definition = extract_method(
                        generated, f"h8_device::{irq_method}"
                    )
                    check_generated_irq_sequence(definition, irq_method)
                    setup_position = definition.find("irq_setup();")
                    stack_position = definition.rfind(
                        "write16(", 0, setup_position
                    )
                    vector_position = definition.find("read16i(", setup_position)
                    final_internal_position = definition.find(
                        "internal(1);", vector_position
                    )
                    ack_position = definition.find(
                        "interrupt_taken();", final_internal_position
                    )
                    if not (
                        definition.count("irq_setup();") == 1
                        and definition.count("interrupt_taken();") == 1
                        and 0
                        <= stack_position
                        < setup_position
                        < vector_position
                        < final_internal_position
                        < ack_position
                    ):
                        raise TestFailure(
                            f"generated {irq_method} must save old state, mask it, "
                            "fetch its latched vector, then acknowledge the controller"
                        )
                for dma_method in ("state_dma_full", "state_dma_partial"):
                    definition = extract_method(
                        generated, f"h8_device::{dma_method}"
                    )
                    if definition.count("begin_dma_bus_cycle(m_TMP2);") != 1:
                        raise TestFailure(
                            f"generated {dma_method} must account for bus acquisition "
                            "before its first transfer"
                        )
                    acquisition_position = definition.find(
                        "begin_dma_bus_cycle(m_TMP2);"
                    )
                    source_position = min(
                        position
                        for position in (
                            definition.find("read16(", acquisition_position),
                            definition.find("read8(", acquisition_position),
                        )
                        if position >= 0
                    )
                    if "if(m_icount <= m_bcount)" not in definition[
                        acquisition_position:source_position
                    ]:
                        raise TestFailure(
                            f"generated {dma_method} lacks a post-acquisition checkpoint"
                        )
                partial_dma = extract_method(
                    generated, "h8_device::state_dma_partial"
                )
                if partial_dma.find("begin_dma_bus_cycle(m_TMP2);") > partial_dma.find(
                    "case 1:;"
                ):
                    raise TestFailure(
                        "DMA acquisition charge would be replayed after a source-read retry"
                    )
            if listing_name == "h8.lst" and chip_type == "h":
                for irq_method in ("state_irq_full", "state_irq_partial"):
                    definition = extract_method(
                        generated, f"h8h_device::{irq_method}"
                    )
                    check_generated_irq_sequence(definition, irq_method)
                    if definition.count("internal(1);") != 2:
                        raise TestFailure(
                            f"generated {irq_method} must contain both documented "
                            "interrupt internal-processing phases"
                        )
                    if definition.count(
                        "m_icount -= interrupt_priority_cycles();"
                    ) != 1:
                        raise TestFailure(
                            f"generated {irq_method} must charge one interrupt "
                            "priority-decision phase"
                        )
                    setup_position = definition.find("irq_setup();")
                    stack_position = definition.rfind(
                        "write16(", 0, setup_position
                    )
                    vector_position = definition.find("read16i(", setup_position)
                    final_internal_position = definition.find(
                        "internal(1);", vector_position
                    )
                    ack_position = definition.find(
                        "interrupt_taken();", final_internal_position
                    )
                    if not (
                        definition.count("irq_setup();") == 1
                        and definition.count("interrupt_taken();") == 1
                        and 0
                        <= stack_position
                        < setup_position
                        < vector_position
                        < final_internal_position
                        < ack_position
                    ):
                        raise TestFailure(
                            f"generated {irq_method} must save old state, mask it, "
                            "fetch its latched vector, then acknowledge the controller"
                        )

def make_shim(h83003_cpp: str, h8_cpp: str) -> str:
    methods = "\n\n".join(
        extract_method(h83003_cpp, f"h83003_device::{name}")
        for name in (
            "device_reset",
            "reset_processing_cycles",
            "interrupt_priority_cycles",
			"interrupt_priority_complete",
            "interrupt_post_accept_prefetch",
            "internal_phase_checkpointing_enabled",
            "dma_bus_acquisition_cycles",
            "memory_access_cycles",
            "abwcr_r",
            "abwcr_w",
            "astcr_r",
            "astcr_w",
            "wcr_r",
            "wcr_w",
            "wcer_r",
            "wcer_w",
            "mdcr_r",
            "brcr_r",
            "brcr_w",
            "syscr_r",
            "syscr_w",
        )
    )
    core_methods = "\n\n".join(
        extract_method(h8_cpp, f"h8_device::{name}")
        for name in (
            "interrupt_post_accept_prefetch",
            "internal_phase_checkpointing_enabled",
            "dma_bus_acquisition_cycles",
            "begin_dma_bus_cycle",
            "charge_memory_access",
            "refund_memory_access",
            "internal",
        )
    )

    return f"""// Generated by h83003_external_bus_timing_test.py.
#include <cstdint>
#include <cstdio>

using u8 = std::uint8_t;
using u32 = std::uint32_t;

#define BIT(value, bit) (((value) >> (bit)) & 1U)
#define logerror(...) ((void)0)

class h8gen_dma_channel_device
{{
public:
    enum {{ AUTOREQ_B = -4 }};
}};

struct h8_dma_state
{{
    int m_trigger_vector = 0;
}};

class h8_device
{{
public:
    int m_icount = 0;
    int m_last_memory_access_cycles = 0;
    int m_dma_bus_owner = -1;
    int m_base_access_cycles = 2;
    bool m_has_exr = false;
	int m_irq_vector = 0;
	int m_taken_irq_vector = 0;
	int m_irq_level = 0;
	int m_taken_irq_level = 0;
    h8_dma_state *m_dma_channel[8] = {{}};
    int m_inst_state = 0;

    virtual ~h8_device() = default;
    virtual int reset_processing_cycles() const {{ return 0; }}
	virtual void interrupt_priority_complete() {{ }}
    virtual bool interrupt_post_accept_prefetch() const;
    virtual bool internal_phase_checkpointing_enabled() const;
    virtual int dma_bus_acquisition_cycles(int channel) const;
    virtual int memory_access_cycles(u32, int) const {{ return m_base_access_cycles; }}
    void begin_dma_bus_cycle(int channel);
    void charge_memory_access(u32 address, int size);
    void refund_memory_access();
    void internal(int cycles);
}};

{core_methods}

class h8h_device : public h8_device
{{
public:
    bool m_mode_a20 = true;

    virtual ~h8h_device() = default;
    virtual void device_reset() {{ }}
    virtual int interrupt_priority_cycles() const {{ return 0; }}
    virtual int memory_access_cycles(u32, int) const {{ return m_base_access_cycles; }}
}};

class ram_view_stub
{{
public:
    bool enabled = false;
    void select(int) {{ enabled = true; }}
    void disable() {{ enabled = false; }}
}};

class intc_stub
{{
public:
    bool nmi_rising_edge = false;
    void set_nmi_edge(bool rising) {{ nmi_rising_edge = rising; }}
    intc_stub *operator->() {{ return this; }}
}};

class h83003_device : public h8h_device
{{
public:
    u8 m_syscr = 0;
    u8 m_rtmcsr = 0;
    u8 m_abwcr = 0;
    u8 m_astcr = 0;
    u8 m_wcr = 0;
    u8 m_wcer = 0;
    u8 m_brcr = 0;
    bool m_initial_bus_16bit = false;
    bool m_external_bus_timing = false;
    bool m_standby_pending = false;
    ram_view_stub m_ram_view;
    intc_stub m_intc;

    void update_irq_filter() {{ }}
    void device_reset() override;
    int reset_processing_cycles() const override;
    int interrupt_priority_cycles() const override;
	void interrupt_priority_complete() override;
    bool interrupt_post_accept_prefetch() const override;
    bool internal_phase_checkpointing_enabled() const override;
    int dma_bus_acquisition_cycles(int channel) const override;
    int memory_access_cycles(u32 address, int size) const override;
    u8 abwcr_r();
    void abwcr_w(u8 data);
    u8 astcr_r();
    void astcr_w(u8 data);
    u8 wcr_r();
    void wcr_w(u8 data);
    u8 wcer_r();
    void wcer_w(u8 data);
    u8 mdcr_r();
    u8 brcr_r();
    void brcr_w(u8 data);
    u8 syscr_r();
    void syscr_w(u8 data);
}};

{methods}

static int failures = 0;

template <typename Actual, typename Expected>
void expect_eq(const char *label, Actual actual, Expected expected)
{{
    if (actual != expected)
    {{
        std::fprintf(stderr, "FAIL %-42s actual=%lld expected=%lld\\n", label,
            static_cast<long long>(actual), static_cast<long long>(expected));
        failures++;
    }}
}}

int main()
{{
    h8_device retry_cpu;
    retry_cpu.m_base_access_cycles = 12;
    retry_cpu.m_icount = 100;
    retry_cpu.charge_memory_access(0x0c0000, 2);
    expect_eq("deferred access initial charge", retry_cpu.m_icount, 88);
    retry_cpu.refund_memory_access();
    expect_eq("deferred access exact refund", retry_cpu.m_icount, 100);
    retry_cpu.charge_memory_access(0x0c0000, 2);
    expect_eq("retried access charged exactly once", retry_cpu.m_icount, 88);

    h83003_device cpu;

    cpu.m_syscr = 0xaa;
    cpu.m_rtmcsr = 0xaa;
    cpu.m_abwcr = 0;
    cpu.m_astcr = 0;
    cpu.m_wcr = 0;
    cpu.m_wcer = 0;
    cpu.m_brcr = 0;
    cpu.m_initial_bus_16bit = false;
    cpu.device_reset();
    expect_eq("reset SYSCR", cpu.syscr_r(), 0x0b);
    expect_eq("reset enables on-chip RAM view", cpu.m_ram_view.enabled, true);
    expect_eq("reset RTMCSR", cpu.m_rtmcsr, 0x00);
    expect_eq("mode 1 reset ABWCR", cpu.abwcr_r(), 0xff);
    expect_eq("mode 1 reset ASTCR", cpu.astcr_r(), 0xff);
    expect_eq("mode 1 reset WCR", cpu.wcr_r(), 0xf3);
    expect_eq("mode 1 reset WCER", cpu.wcer_r(), 0xff);
    expect_eq("mode 1 MDCR", cpu.mdcr_r(), 0xc1);
    expect_eq("reset BRCR", cpu.brcr_r(), 0xfe);
    expect_eq("reset timing opt-out uses base H8 behavior", cpu.reset_processing_cycles(), 0);
    expect_eq("internal checkpoint opt-out uses base H8 behavior",
        cpu.internal_phase_checkpointing_enabled(), false);

    cpu.m_external_bus_timing = true;
    expect_eq("reset post-vector internal processing", cpu.reset_processing_cycles(), 2);
    expect_eq("native internal checkpoints enabled",
        cpu.internal_phase_checkpointing_enabled(), true);
    expect_eq("mode 1 reset ROM word (8-bit bus)",
        cpu.memory_access_cycles(0x000000, 2), 12);

    cpu.m_initial_bus_16bit = true;
    cpu.device_reset();
    expect_eq("mode 2 reset ABWCR", cpu.abwcr_r(), 0x00);
    expect_eq("mode 2 reset ASTCR", cpu.astcr_r(), 0xff);
    expect_eq("mode 2 reset WCR", cpu.wcr_r(), 0xf3);
    expect_eq("mode 2 reset WCER", cpu.wcer_r(), 0xff);
    expect_eq("mode 2 MDCR", cpu.mdcr_r(), 0xc2);
    expect_eq("mode 2 reset ROM word (16-bit bus)",
        cpu.memory_access_cycles(0x000000, 2), 6);

    cpu.m_mode_a20 = false;
    cpu.m_initial_bus_16bit = false;
    cpu.device_reset();
    expect_eq("mode 3 MDCR", cpu.mdcr_r(), 0xc3);
    cpu.m_initial_bus_16bit = true;
    cpu.device_reset();
    expect_eq("mode 4 MDCR", cpu.mdcr_r(), 0xc4);
    cpu.m_mode_a20 = true;
    cpu.m_initial_bus_16bit = true;

    cpu.brcr_w(0xff);
    expect_eq("BRCR writable BRLE bit", cpu.brcr_r(), 0xff);
    cpu.brcr_w(0x00);
    expect_eq("BRCR reserved bits read one", cpu.brcr_r(), 0xfe);

    cpu.syscr_w(0x00);
    expect_eq("SYSCR reserved bit reads one", cpu.syscr_r(), 0x02);
    expect_eq("RAME=0 disables on-chip RAM view", cpu.m_ram_view.enabled, false);
    expect_eq("NMIEG=0 selects falling edge", cpu.m_intc.nmi_rising_edge, false);
    expect_eq("SSBY=0 clears standby request", cpu.m_standby_pending, false);
    cpu.syscr_w(0x84);
    expect_eq("NMIEG=1 selects rising edge", cpu.m_intc.nmi_rising_edge, true);
    expect_eq("SSBY=1 arms software standby", cpu.m_standby_pending, true);
    cpu.syscr_w(0x09);
    expect_eq("SYSCR write preserves reserved bit", cpu.syscr_r(), 0x0b);
    expect_eq("RAME=1 enables on-chip RAM view", cpu.m_ram_view.enabled, true);

    cpu.m_taken_irq_vector = 12;
    expect_eq("external IRQ0 lower boundary", cpu.interrupt_priority_cycles(), 2);
    cpu.m_taken_irq_vector = 13;
    expect_eq("external IRQ1 priority decision", cpu.interrupt_priority_cycles(), 2);
    cpu.m_taken_irq_vector = 19;
    expect_eq("external IRQ7 upper boundary", cpu.interrupt_priority_cycles(), 2);
    cpu.m_taken_irq_vector = 20;
    expect_eq("internal interrupt lower boundary", cpu.interrupt_priority_cycles(), 1);
    cpu.m_taken_irq_vector = 53;
    expect_eq("internal SCI0 RXI priority decision", cpu.interrupt_priority_cycles(), 1);

    cpu.m_taken_irq_vector = 13;
    cpu.m_icount = 100;
    cpu.m_icount -= cpu.interrupt_priority_cycles();
    cpu.internal(1);
    cpu.internal(1);
    expect_eq("external IRQ priority + internal processing", cpu.m_icount, 94);
    cpu.m_taken_irq_vector = 53;
    cpu.m_icount = 100;
    cpu.m_icount -= cpu.interrupt_priority_cycles();
    cpu.internal(1);
    cpu.internal(1);
    expect_eq("internal IRQ priority + internal processing", cpu.m_icount, 95);

	// The instruction-boundary winner is tentative until the separately timed
	// H8/3003 priority-decision phase completes.
	cpu.m_taken_irq_vector = 13;
	cpu.m_taken_irq_level = 0;
	cpu.m_irq_vector = 53;
	cpu.m_irq_level = 1;
	cpu.interrupt_priority_complete();
	expect_eq("higher-priority SCI0 RXI supersedes IRQ1", cpu.m_taken_irq_vector, 53);
	expect_eq("replacement priority level is latched", cpu.m_taken_irq_level, 1);
	cpu.m_taken_irq_vector = 53;
	cpu.m_taken_irq_level = 1;
	cpu.m_irq_vector = 13;
	cpu.m_irq_level = 1;
	cpu.interrupt_priority_complete();
	expect_eq("same-level lower vector wins priority phase", cpu.m_taken_irq_vector, 13);
	cpu.m_taken_irq_vector = 13;
	cpu.m_taken_irq_level = 1;
	cpu.m_irq_vector = 53;
	cpu.m_irq_level = 1;
	cpu.interrupt_priority_complete();
	expect_eq("same-level higher vector cannot steal entry", cpu.m_taken_irq_vector, 13);
	cpu.m_irq_vector = 53;
	cpu.m_irq_level = 0;
	cpu.interrupt_priority_complete();
	expect_eq("lower-priority request cannot steal entry", cpu.m_taken_irq_vector, 13);
	cpu.m_irq_vector = 0;
	cpu.m_irq_level = 2;
	cpu.interrupt_priority_complete();
	expect_eq("zero vector cannot replace tentative winner", cpu.m_taken_irq_vector, 13);

    cpu.abwcr_w(0xfc);
    cpu.astcr_w(0x7c);
    cpu.wcr_w(0x09);
    cpu.wcer_w(0x48);
    expect_eq("programmed ABWCR", cpu.abwcr_r(), 0xfc);
    expect_eq("programmed ASTCR", cpu.astcr_r(), 0x7c);
    expect_eq("programmed WCR reserved high nibble", cpu.wcr_r(), 0xf9);
    expect_eq("programmed WCER", cpu.wcer_r(), 0x48);
    expect_eq("native H8/3003 post-accept fetch enabled",
        cpu.interrupt_post_accept_prefetch(), true);
    const int irq_rom_word = cpu.memory_access_cycles(0x000000, 2);
    const int irq_onchip_stack_word = cpu.memory_access_cycles(0x0ffe00, 2);
    const int irq_area2_stack_word = cpu.memory_access_cycles(0x040000, 2);
    const int external_irq_onchip_stack_states = 2 + irq_rom_word + 2
        + 2 * irq_onchip_stack_word + 2 * irq_rom_word + 2 + irq_rom_word;
    const int internal_irq_onchip_stack_states = 1 + irq_rom_word + 2
        + 2 * irq_onchip_stack_word + 2 * irq_rom_word + 2 + irq_rom_word;
    const int external_irq_area2_stack_states = 2 + irq_rom_word + 2
        + 2 * irq_area2_stack_word + 2 * irq_rom_word + 2 + irq_rom_word;
    const int internal_irq_area2_stack_states = 1 + irq_rom_word + 2
        + 2 * irq_area2_stack_word + 2 * irq_rom_word + 2 + irq_rom_word;
    expect_eq("mode 2 external IRQ entry with on-chip stack",
        external_irq_onchip_stack_states, 18);
    expect_eq("mode 2 internal IRQ entry with on-chip stack",
        internal_irq_onchip_stack_states, 17);
    expect_eq("mode 2 external IRQ entry with area-2 task stack",
        external_irq_area2_stack_states, 26);
    expect_eq("mode 2 internal IRQ entry with area-2 task stack",
        internal_irq_area2_stack_states, 25);

    cpu.wcr_w(0xa5);
    expect_eq("WCR write-mask behavior", cpu.wcr_r(), 0xf5);
    cpu.wcr_w(0x09);

    cpu.m_external_bus_timing = false;
    cpu.m_base_access_cycles = 7;
    expect_eq("timing opt-out uses base H8 behavior",
        cpu.memory_access_cycles(0x0c0000, 1), 7);
    expect_eq("reset timing opt-out remains base behavior",
        cpu.reset_processing_cycles(), 0);
    cpu.m_taken_irq_vector = 13;
    expect_eq("IRQ priority timing opt-out remains base behavior",
        cpu.interrupt_priority_cycles(), 0);
	cpu.m_taken_irq_vector = 13;
	cpu.m_taken_irq_level = 0;
	cpu.m_irq_vector = 53;
	cpu.m_irq_level = 1;
	cpu.interrupt_priority_complete();
	expect_eq("priority replacement opt-out remains base behavior",
		cpu.m_taken_irq_vector, 13);
    expect_eq("post-accept fetch timing opt-out remains base behavior",
        cpu.interrupt_post_accept_prefetch(), false);
    expect_eq("internal checkpoint timing opt-out remains base behavior",
        cpu.internal_phase_checkpointing_enabled(), false);
    cpu.m_base_access_cycles = 2;
    cpu.m_external_bus_timing = true;
    cpu.m_mode_a20 = true;

    // Prophecy firmware values: ABWCR=FC ASTCR=7C WCR=F9 WCER=48.
    expect_eq("A20 area 0 byte ROM read", cpu.memory_access_cycles(0x000000, 1), 2);
    expect_eq("A20 area 0 word ROM fetch", cpu.memory_access_cycles(0x010000, 2), 2);
    expect_eq("A20 area 2 byte SRAM access", cpu.memory_access_cycles(0x040000, 1), 3);
    expect_eq("A20 area 2 word on 8-bit bus", cpu.memory_access_cycles(0x040000, 2), 6);
    expect_eq("A20 area 6 byte DSP host write", cpu.memory_access_cycles(0x0c0000, 1), 4);
    expect_eq("A20 upper effective-address bits ignored", cpu.memory_access_cycles(0x1c0000, 1), 4);
    expect_eq("A20 area 6 word on 8-bit bus", cpu.memory_access_cycles(0x0c0000, 2), 8);
    expect_eq("A20 byte before RAM is external", cpu.memory_access_cycles(0x0ffd0f, 1), 2);
    expect_eq("A20 word before RAM uses area 7 width", cpu.memory_access_cycles(0x0ffd0f, 2), 4);
    expect_eq("A20 first on-chip RAM byte", cpu.memory_access_cycles(0x0ffd10, 1), 2);
    expect_eq("A20 first on-chip RAM word", cpu.memory_access_cycles(0x0ffd10, 2), 2);
    expect_eq("A20 last on-chip RAM word", cpu.memory_access_cycles(0x0fff0f, 2), 2);
    expect_eq("A20 first external-window byte", cpu.memory_access_cycles(0x0fff10, 1), 2);
    expect_eq("A20 first external-window word", cpu.memory_access_cycles(0x0fff10, 2), 4);
    expect_eq("A20 last external-window word", cpu.memory_access_cycles(0x0fff1b, 2), 4);
    expect_eq("A20 first on-chip register byte", cpu.memory_access_cycles(0x0fff1c, 1), 3);
    expect_eq("A20 last on-chip register byte", cpu.memory_access_cycles(0x0fffff, 1), 3);
    expect_eq("A20 DMAC bus8 word", cpu.memory_access_cycles(0x0fff20, 2), 6);
    expect_eq("A20 WDT bus8 word", cpu.memory_access_cycles(0x0fffa8, 2), 6);
    expect_eq("A20 bus-controller bus8 word", cpu.memory_access_cycles(0x0fffec, 2), 6);
    expect_eq("A20 ITU0 TCNT bus16 word", cpu.memory_access_cycles(0x0fff68, 2), 3);
    expect_eq("A20 ITU0 GRB bus16 word", cpu.memory_access_cycles(0x0fff6c, 2), 3);
    expect_eq("A20 ITU2 GRB bus16 word", cpu.memory_access_cycles(0x0fff80, 2), 3);
    expect_eq("A20 ITU3 TCNT bus16 word", cpu.memory_access_cycles(0x0fff86, 2), 3);
    expect_eq("A20 ITU4 BRB bus16 word", cpu.memory_access_cycles(0x0fff9e, 2), 3);
    expect_eq("A20 DMAC MOV.L four-byte total",
        2 * cpu.memory_access_cycles(0x0fff20, 2), 12);
    expect_eq("A20 RAM upper-bit alias", cpu.memory_access_cycles(0x1ffd10, 2), 2);
    expect_eq("A20 register upper-bit alias", cpu.memory_access_cycles(0x1fff1c, 1), 3);
    expect_eq("A20 bus8-word upper-bit alias", cpu.memory_access_cycles(0x1fff20, 2), 6);
    expect_eq("A20 bus16-word upper-bit alias", cpu.memory_access_cycles(0x1fff68, 2), 3);

    cpu.syscr_w(cpu.syscr_r() & ~1);
    expect_eq("A20 disabled on-chip RAM follows area 7", cpu.memory_access_cycles(0x0ffd10, 2), 4);
    cpu.syscr_w(cpu.syscr_r() | 1);

    const int tight_loop_byte_cadence =
        cpu.memory_access_cycles(0x010000, 2) +
        cpu.memory_access_cycles(0x0c0000, 1);
    expect_eq("tight ROM-fetch + DSP-write cadence", tight_loop_byte_cadence, 6);
    expect_eq("774-byte stream first-to-last span", 773 * tight_loop_byte_cadence, 4638);
    expect_eq("1024-byte stream first-to-last span", 1023 * tight_loop_byte_cadence, 6138);

    // The observed uploader uses full-address auto-request burst mode.  The
    // H8/3003 inserts one dead state when that burst acquires the bus, then
    // retains ownership for back-to-back source/destination pairs.  Figure
    // 8-15 returns directly to CPU T1 after the final destination T2, so bus
    // release clears ownership without charging a second Td.
    h8_dma_state burst_dma;
    burst_dma.m_trigger_vector = h8gen_dma_channel_device::AUTOREQ_B;
    h8_dma_state nonburst_dma;
    nonburst_dma.m_trigger_vector = -3;
    cpu.m_dma_channel[1] = &burst_dma;
    cpu.m_dma_channel[0] = &nonburst_dma;
    cpu.m_icount = 100;
    cpu.m_dma_bus_owner = -1;
    cpu.begin_dma_bus_cycle(1);
    cpu.charge_memory_access(0x010000, 1);
    cpu.charge_memory_access(0x0c0000, 1);
    expect_eq("DMA burst first host write completion", cpu.m_icount, 93);
    cpu.begin_dma_bus_cycle(1);
    cpu.charge_memory_access(0x010001, 1);
    cpu.charge_memory_access(0x0c0000, 1);
    expect_eq("DMA burst second write has no new Td", cpu.m_icount, 87);
    cpu.begin_dma_bus_cycle(1);
    cpu.charge_memory_access(0x010002, 1);
    cpu.charge_memory_access(0x0c0000, 1);
    expect_eq("three-byte DMA burst total is 19 states", cpu.m_icount, 81);
    // prefetch_done() performs this ownership-only release in production; its
    // static contract above rejects any release-time m_icount debit.
    cpu.m_dma_bus_owner = -1;
    expect_eq("completed burst release has no trailing Td", cpu.m_icount, 81);
    expect_eq("completed burst releases ownership", cpu.m_dma_bus_owner, -1);
    cpu.charge_memory_access(0x010004, 2);
    expect_eq("CPU fetch starts immediately after burst", cpu.m_icount, 79);
    cpu.begin_dma_bus_cycle(1);
    expect_eq("second burst reacquires bus", cpu.m_icount, 78);
    cpu.begin_dma_bus_cycle(0);
    expect_eq("non-burst modes preserve prior timing", cpu.m_icount, 78);
    cpu.begin_dma_bus_cycle(1);
    expect_eq("burst after non-burst reacquires bus", cpu.m_icount, 77);
    h8_dma_state second_burst_dma;
    second_burst_dma.m_trigger_vector = h8gen_dma_channel_device::AUTOREQ_B;
    cpu.m_dma_channel[3] = &second_burst_dma;
    cpu.begin_dma_bus_cycle(3);
    expect_eq("burst channel switch reacquires bus", cpu.m_icount, 76);
    cpu.m_external_bus_timing = false;
    cpu.m_dma_bus_owner = -1;
    cpu.begin_dma_bus_cycle(1);
    expect_eq("timing opt-out omits DMA acquisition Td", cpu.m_icount, 76);
    cpu.m_external_bus_timing = true;

    cpu.m_mode_a20 = false;
    expect_eq("A24 area 0 word ROM fetch", cpu.memory_access_cycles(0x100000, 2), 2);
    expect_eq("A24 area 2 byte access", cpu.memory_access_cycles(0x400000, 1), 3);
    expect_eq("A24 area 6 byte access", cpu.memory_access_cycles(0xc00000, 1), 4);
    expect_eq("A24 upper effective-address bits ignored", cpu.memory_access_cycles(0x01c00000, 1), 4);
    expect_eq("A24 byte before RAM is external", cpu.memory_access_cycles(0xfffd0f, 1), 2);
    expect_eq("A24 word before RAM uses area 7 width", cpu.memory_access_cycles(0xfffd0f, 2), 4);
    expect_eq("A24 first on-chip RAM word", cpu.memory_access_cycles(0xfffd10, 2), 2);
    expect_eq("A24 last on-chip RAM word", cpu.memory_access_cycles(0xffff0f, 2), 2);
    expect_eq("A24 first external-window word", cpu.memory_access_cycles(0xffff10, 2), 4);
    expect_eq("A24 last external-window word", cpu.memory_access_cycles(0xffff1b, 2), 4);
    expect_eq("A24 first on-chip register byte", cpu.memory_access_cycles(0xffff1c, 1), 3);
    expect_eq("A24 last on-chip register byte", cpu.memory_access_cycles(0xffffff, 1), 3);
    expect_eq("A24 DMAC bus8 word", cpu.memory_access_cycles(0xffff20, 2), 6);
    expect_eq("A24 WDT bus8 word", cpu.memory_access_cycles(0xffffa8, 2), 6);
    expect_eq("A24 bus-controller bus8 word", cpu.memory_access_cycles(0xffffec, 2), 6);
    expect_eq("A24 ITU0 TCNT bus16 word", cpu.memory_access_cycles(0xffff68, 2), 3);
    expect_eq("A24 ITU4 BRB bus16 word", cpu.memory_access_cycles(0xffff9e, 2), 3);
    expect_eq("A24 RAM upper-bit alias", cpu.memory_access_cycles(0x01fffd10, 2), 2);
    expect_eq("A24 register upper-bit alias", cpu.memory_access_cycles(0x01ffff1c, 1), 3);
    expect_eq("A24 bus8-word upper-bit alias", cpu.memory_access_cycles(0x01ffff20, 2), 6);
    expect_eq("A24 bus16-word upper-bit alias", cpu.memory_access_cycles(0x01ffff68, 2), 3);

    cpu.m_mode_a20 = true;
    cpu.m_astcr = 0x40;
    cpu.m_wcer = 0x40;
    cpu.m_abwcr = 0x00;
    cpu.m_wcr = 0xf2;
    expect_eq("programmable wait mode, two waits", cpu.memory_access_cycles(0x0c0000, 1), 5);
    cpu.m_wcr = 0xf4;
    expect_eq("WMS=01 inserts no WSC waits", cpu.memory_access_cycles(0x0c0000, 1), 3);
    cpu.m_wcr = 0xf9;
    expect_eq("pin-wait mode 1, one wait and WAIT high", cpu.memory_access_cycles(0x0c0000, 1), 4);
    cpu.m_wcr = 0xfd;
    expect_eq("pin auto-wait mode with WAIT high", cpu.memory_access_cycles(0x0c0000, 1), 3);

    if (failures)
        return 1;
    std::puts("PASS: H8/3003 mode 1-4 reset, register bus widths, areas 0/2/6, and 6-cycle cadence");
    return 0;
}}
"""


def compile_and_run(shim: str, cxx: str) -> str:
    with tempfile.TemporaryDirectory(prefix="h83003-bus-timing-") as temp_name:
        temp = Path(temp_name)
        source_path = temp / "h83003_bus_timing_test.cpp"
        binary_path = temp / "h83003_bus_timing_test"
        source_path.write_text(shim, encoding="utf-8")

        compile_result = subprocess.run(
            [
                cxx,
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                str(source_path),
                "-o",
                str(binary_path),
            ],
            check=False,
            text=True,
            capture_output=True,
        )
        if compile_result.returncode:
            raise TestFailure(
                "failed to compile extracted production timing methods:\n"
                + compile_result.stdout
                + compile_result.stderr
            )

        run_result = subprocess.run(
            [str(binary_path)],
            check=False,
            text=True,
            capture_output=True,
        )
        if run_result.returncode:
            raise TestFailure(
                "extracted production timing vectors failed:\n"
                + run_result.stdout
                + run_result.stderr
            )
        return run_result.stdout.strip()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--worktree",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="MAME worktree root (default: inferred from this script)",
    )
    parser.add_argument(
        "--cxx",
        default=os.environ.get("CXX", "c++"),
        help="C++ compiler used for the extracted-method shim (default: CXX or c++)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.worktree.resolve()
    cxx = shutil.which(args.cxx)
    if cxx is None:
        print(f"ERROR: C++ compiler not found: {args.cxx}", file=sys.stderr)
        return 2

    try:
        h83003_cpp = (root / "src/devices/cpu/h8/h83003.cpp").read_text(encoding="utf-8")
        h83003_h = (root / "src/devices/cpu/h8/h83003.h").read_text(encoding="utf-8")
        check_static_wiring(root, h83003_cpp, h83003_h)
        check_generated_retry_paths(root)
        h8_cpp = (root / "src/devices/cpu/h8/h8.cpp").read_text(encoding="utf-8")
        result = compile_and_run(make_shim(h83003_cpp, h8_cpp), cxx)
    except (OSError, TestFailure) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

    print(result)
    print("PASS: production map/save/access/IRQ charging and Prophecy board wiring")
    print("PASS: public ROM-free source-level timing contract")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
