#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""ROM-free regression for the V55 UART baud-generator model."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


class TestFailure(RuntimeError):
    """A publication contract was not satisfied."""


def extract_method(source: str, qualified_name: str) -> str:
    match = re.search(
        rf"(?m)^[^\n]*\b{re.escape(qualified_name)}\b[^\n]*\n?\s*\{{",
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


def compact(source: str) -> str:
    return re.sub(r"\s+", "", source)


def check_static_wiring(source: str, header: str, driver: str, manifest: str) -> None:
    compact_header = compact(header)
    expected_wrappers = (
        "uart0_tx_bit_period() const { return uart_bit_period(0, true); }",
        "uart0_rx_bit_period() const { return uart_bit_period(0, false); }",
        "uart1_tx_bit_period() const { return uart_bit_period(1, true); }",
        "uart1_rx_bit_period() const { return uart_bit_period(1, false); }",
    )
    for wrapper in expected_wrappers:
        if compact(wrapper) not in compact_header:
            raise TestFailure(f"missing direction-specific period wrapper: {wrapper}")

    expected_source_counts = {
        "uart0_tx_bit_period()": 3,
        "uart0_rx_bit_period()": 3,
        "uart1_tx_bit_period()": 3,
        "uart1_rx_bit_period()": 3,
    }
    for token, expected in expected_source_counts.items():
        actual = source.count(token)
        if actual != expected:
            raise TestFailure(
                f"unexpected timer routing count for {token}: {actual}, expected {expected}"
            )

    formula = compact(extract_method(source, "v55_device::uart_bit_period"))
    for token in (
        "const unsigned base = 0x170 + channel * 8;",
        "const u8 brg = m_sfr[base + (transmit ? 0 : 1)];",
        "const u8 prs = m_sfr[base + 2];",
        "const unsigned selector = transmit ? ((prs >> 3) & 0x07) : (prs & 0x07);",
        "const u32 ticks = u32(brg + 1U) << (selector + 2);",
        "return attotime::from_ticks(ticks, clock());",
    ):
        if compact(token) not in formula:
            raise TestFailure(f"production divider formula is missing: {token}")

    combined = source + header + driver + manifest
    retired = (
        "set_uart0_bit_rate",
        "m_uart0_bit_rate",
        "set_uart0_stop_edge_lead_ticks",
        "m_uart0_stop_edge_lead_ticks",
        "KPROP_V55_UART0_STOP_EDGE_LEAD_TICKS",
    )
    remaining = [token for token in retired if token in combined]
    if remaining:
        raise TestFailure(f"retired fixed-rate/edge-lead controls remain: {remaining}")

    for token in (
        "const u32 v55_clock = (16_MHz_XTAL).value();",
        "V55(config, m_maincpu, v55_clock);",
        "m_maincpu->txd_handler_cb().set(FUNC(korgprophecy_state::v55_txd_w));",
        "m_subcpu->write_sci_tx<0>().set(FUNC(korgprophecy_state::h8_txd0_w));",
    ):
        if token not in driver:
            raise TestFailure(f"Prophecy clock/wire configuration is missing: {token}")


def build_and_run(method: str) -> None:
    compiler = shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        raise TestFailure("no C++ compiler found")

    harness = f"""
#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

using u8 = std::uint8_t;
using u32 = std::uint32_t;

struct attotime
{{
    std::uint64_t ticks;
    std::uint64_t frequency;

    static attotime from_ticks(std::uint64_t ticks, std::uint64_t frequency)
    {{
        return {{ticks, frequency}};
    }}
}};

class v55_device
{{
public:
    attotime uart_bit_period(unsigned channel, bool transmit) const;
    u32 clock() const {{ return 16'000'000; }}
    std::array<u8, 0x200> m_sfr{{}};
}};

{method}

static void require(bool condition, const char *message)
{{
    if (!condition)
    {{
        std::cerr << "FAIL " << message << '\\n';
        std::exit(1);
    }}
}}

int main()
{{
    v55_device device;

    device.m_sfr[0x170] = 3;
    device.m_sfr[0x171] = 5;
    device.m_sfr[0x172] = (2U << 3) | 4U;
    require(device.uart_bit_period(0, true).ticks == 64,
        "channel 0 TXBRG/TSK divider");
    require(device.uart_bit_period(0, false).ticks == 384,
        "channel 0 RXBRG/RSK divider");

    device.m_sfr[0x170] = 0x7f;
    device.m_sfr[0x171] = 0x7c;
    device.m_sfr[0x172] = 0;
    require(device.uart_bit_period(0, true).ticks == 512,
        "Prophecy V55-to-H8 31.25 kbaud divider");
    require(device.uart_bit_period(0, false).ticks == 500,
        "Prophecy H8-to-V55 32 kbaud divider");

    device.m_sfr[0x178] = 0xff;
    device.m_sfr[0x179] = 0x01;
    device.m_sfr[0x17a] = (7U << 3) | 1U;
    require(device.uart_bit_period(1, true).ticks == 131'072,
        "channel 1 maximum TX divider");
    require(device.uart_bit_period(1, false).ticks == 16,
        "channel 1 independent RX divider");
    require(device.uart_bit_period(1, true).frequency == 16'000'000,
        "V55 device clock source");

    std::cout << "PASS v55_uart_brg_timing" << '\\n';
    return 0;
}}
"""

    with tempfile.TemporaryDirectory(prefix="v55-uart-brg-") as directory:
        root = Path(directory)
        source = root / "test.cpp"
        binary = root / "test"
        source.write_text(harness, encoding="utf-8")
        compile_result = subprocess.run(
            [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(binary)],
            text=True,
            capture_output=True,
        )
        if compile_result.returncode:
            raise TestFailure(f"harness compilation failed:\n{compile_result.stderr}")
        run_result = subprocess.run([str(binary)], text=True, capture_output=True)
        if run_result.returncode:
            raise TestFailure(
                f"harness failed:\n{run_result.stdout}{run_result.stderr}"
            )
        print(run_result.stdout.strip())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--worktree",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="MAME worktree containing the production sources",
    )
    args = parser.parse_args()
    root = args.worktree.resolve()

    source = (root / "src/devices/cpu/nec/v5x.cpp").read_text(encoding="utf-8")
    header = (root / "src/devices/cpu/nec/v5x.h").read_text(encoding="utf-8")
    driver = (root / "src/mame/korg/korgprophecy.cpp").read_text(encoding="utf-8")
    manifest = (root / "tests/korgprophecy/public_controls_v1.json").read_text(
        encoding="utf-8"
    )

    check_static_wiring(source, header, driver, manifest)
    build_and_run(extract_method(source, "v55_device::uart_bit_period"))
    print("PASS v55_uart_brg_static_wiring")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, TestFailure) as error:
        print(f"FAIL {error}", file=sys.stderr)
        sys.exit(1)
