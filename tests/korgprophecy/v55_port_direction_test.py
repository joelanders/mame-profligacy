#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""ROM-free regression for V55 P2/P3 GPIO output qualification.

The runner extracts the exact production C++ port-output helpers and port write
method, compiles them in a minimal type-compatible shim, and executes focused
direction/control vectors.  Static checks cover the SFR write and reset wiring
that cannot be isolated without constructing a complete V55 device.

This proves GPIO callback qualification at the byte-write boundary, including
the per-pin output mask consumed by the Prophecy P33/IRQ1 route.  It does not
model alternate-function output waveforms or establish the electrical level of
an input/high-impedance pin.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


class TestFailure(RuntimeError):
    """A regression contract was not satisfied."""


def extract_method(source: str, qualified_name: str) -> str:
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


def require_tokens(source: str, description: str, tokens: list[str]) -> None:
    compact_source = compact(source)
    missing = [token for token in tokens if compact(token) not in compact_source]
    if missing:
        rendered = "\n  ".join(missing)
        raise TestFailure(f"{description} is incomplete; missing:\n  {rendered}")


def check_static_wiring(source: str, header: str, driver: str) -> None:
    require_tokens(
        header,
        "V55 qualified port-output API",
        [
            "u8 port_output_mask(unsigned port) const;",
            "void update_port_output(unsigned port);",
        ],
    )

    port_write = extract_method(source, "v55_device::port_w")
    require_tokens(
        port_write,
        "P2/P3 data-latch output qualification",
        [
            "if (port == 2 || port == 3)",
            "update_port_output(port);",
        ],
    )

    sfr_write = extract_method(source, "v55_device::sfr_w")
    require_tokens(
        sfr_write,
        "P2/P3 mode-register output refresh",
        [
            "case 0x112:",
            "case 0x122:",
            "update_port_output(2);",
            "case 0x113:",
            "case 0x123:",
            "update_port_output(3);",
        ],
    )

    reset = extract_method(source, "v55_device::device_reset")
    require_tokens(
        reset,
        "documented V55 P2/P3 reset directions",
        [
            "m_sfr[0x112] = 0xff;",
            "m_sfr[0x113] = 0xff;",
        ],
    )

    p3_write = extract_method(driver, "korgprophecy_state::v55_p3_w")
    require_tokens(
        p3_write,
        "Prophecy P33 output-mask consumption",
        [
            "data = u8((old & ~mem_mask) | (data & mem_mask));",
            "if ((mem_mask & 0x08) && BIT(old, 3) != BIT(data, 3))",
            "FUNC(korgprophecy_state::v55_p33_sync)",
        ],
    )

    for retired in ("pulse_h8_irq1", "update_h8_cts_from_port8"):
        if retired in driver:
            raise TestFailure(f"retired synthetic IRQ/CTS path remains: {retired}")


def build_and_run(root: Path, source: str) -> None:
    compiler = shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        raise TestFailure("no C++ compiler found (need clang++ or g++)")

    definitions = "\n\n".join(
        extract_method(source, name)
        for name in (
            "v55_device::port_w",
            "v55_device::port_output_mask",
            "v55_device::update_port_output",
        )
    )
    harness = f"""
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

using u8 = std::uint8_t;
using u32 = std::uint32_t;
using offs_t = std::uint32_t;

template <typename... Args>
void logerror(const char *, Args...) {{}}

struct fake_time
{{
    double as_double() const {{ return 0.0; }}
}};

struct fake_machine
{{
    fake_time time() const {{ return {{}}; }}
}};

struct fake_write8
{{
    bool unset = false;
    std::vector<u8> values;
    std::vector<u8> masks;
    bool isunset() const {{ return unset; }}
    void operator()(u8 data) {{ values.push_back(data); masks.push_back(0xff); }}
    void operator()(offs_t, u8 data, u8 mem_mask) {{ values.push_back(data); masks.push_back(mem_mask); }}
}};

class v55_device
{{
public:
    std::array<u8, 0x200> m_sfr{{}};
    std::array<fake_write8, 9> m_port_out_cb{{}};
    bool m_log_v55_port_core = false;

    void port_w(unsigned port, u8 data);
    u8 port_output_mask(unsigned port) const;
    void update_port_output(unsigned port);
    fake_machine machine() const {{ return {{}}; }}
    u32 pc() const {{ return 0; }}
}};

{definitions}

static int failures = 0;

static void check(bool condition, const char *message)
{{
    if (!condition)
    {{
        std::fprintf(stderr, "FAIL: %s\\n", message);
        ++failures;
    }}
}}

int main()
{{
    v55_device device;

    // NEC reset contract: both ports start in GPIO input mode.  A write may
    // update the latch, but it must not drive the board callback.
    device.m_sfr[0x112] = 0xff;
    device.m_sfr[0x113] = 0xff;
    device.port_w(3, 0x59);
    check(device.m_sfr[0x103] == 0x59, "input-mode P3 write must update latch");
    check(device.m_port_out_cb[3].values.empty(), "input-mode P3 must not drive callback");

    // Prophecy v1.7's observed setup: PM3=B3, PMC3=33 makes P32/P33/P36
    // ordinary GPIO outputs, including the active-low P33 interrupt line.
    device.m_sfr[0x113] = 0xb3;
    device.m_sfr[0x123] = 0x33;
    check(device.port_output_mask(3) == 0x4c, "Prophecy PM3/PMC3 output mask must be exact");
    device.update_port_output(3);
    check(device.m_port_out_cb[3].values.size() == 1, "direction enable must publish P3 latch");
    check(device.m_port_out_cb[3].values.back() == 0x59, "published P3 latch must remain exact");
    check(device.m_port_out_cb[3].masks.back() == 0x4c, "published P3 mask must identify P32/P33/P36");

    device.port_w(3, 0x51);
    check(device.m_port_out_cb[3].values.size() == 2, "enabled P3 write must drive callback");
    check(device.m_port_out_cb[3].values.back() == 0x51, "enabled P3 callback value must be exact");

    // Isolate P33 so each direction/control bit can suppress the callback.
    device.m_sfr[0x113] = 0xf7;
    device.m_sfr[0x123] = 0x00;
    check(device.port_output_mask(3) == 0x08, "PM33=0 and PMC33=0 must enable only P33 GPIO output");
    device.m_sfr[0x113] = 0xff;
    check(device.port_output_mask(3) == 0x00, "PM33=1 must disable isolated P33 output");
    device.m_sfr[0x113] = 0xf7;
    device.m_sfr[0x123] = 0x08;
    check(device.port_output_mask(3) == 0x00, "PMC33=1 must remove isolated P33 from GPIO callback");

    // A different output on the same byte must not make an input-configured
    // P33 appear driven.  The devcb mask carries per-pin high-Z information.
    device.m_sfr[0x113] = 0xfb;
    device.m_sfr[0x123] = 0x00;
    device.port_w(3, 0x08);
    check(device.m_port_out_cb[3].masks.back() == 0x04, "P32 output must not mark input P33 as driven");
    check(!(device.m_port_out_cb[3].masks.back() & 0x08), "input P33 must remain absent from callback mask");

    // P3 is seven bits: the nonexistent bit 7 cannot enable a callback.
    device.m_sfr[0x113] = 0x7f;
    device.m_sfr[0x123] = 0x00;
    check(device.port_output_mask(3) == 0x00, "P37 must not exist in the seven-bit port mask");

    // Preserve the pre-existing six-bit P2 contract through the shared helper.
    device.m_sfr[0x112] = 0xbf;
    device.m_sfr[0x122] = 0x00;
    check(device.port_output_mask(2) == 0x00, "P26/P27 must not exist in the six-bit port mask");
    device.m_sfr[0x112] = 0xfe;
    check(device.port_output_mask(2) == 0x01, "PM20=0 and PMC20=0 must enable only P20 GPIO output");

    check(device.port_output_mask(4) == 0x00, "unsupported port must not use P2/P3 helper");

    if (failures)
        return 1;
    std::puts("PASS: V55 P2/P3 direction/control output regression");
    return 0;
}}
"""

    with tempfile.TemporaryDirectory(prefix="v55-port-direction-") as temp_name:
        temp = Path(temp_name)
        harness_path = temp / "v55_port_direction.cpp"
        binary_path = temp / "v55_port_direction"
        harness_path.write_text(harness, encoding="utf-8")
        compile_result = subprocess.run(
            [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", str(harness_path), "-o", str(binary_path)],
            cwd=root,
            check=False,
            text=True,
            capture_output=True,
        )
        if compile_result.returncode:
            raise TestFailure(
                "failed to compile extracted V55 production methods:\n"
                + compile_result.stdout
                + compile_result.stderr
            )
        run_result = subprocess.run(
            [str(binary_path)],
            cwd=root,
            check=False,
            text=True,
            capture_output=True,
        )
        if run_result.returncode:
            raise TestFailure(
                "V55 direction/control behavior regression failed:\n"
                + run_result.stdout
                + run_result.stderr
            )
        print(run_result.stdout, end="")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="MAME source root (defaults to this script's repository)",
    )
    args = parser.parse_args()
    root = args.root.resolve()
    cpp_path = root / "src/devices/cpu/nec/v5x.cpp"
    header_path = root / "src/devices/cpu/nec/v5x.h"
    driver_path = root / "src/mame/korg/korgprophecy.cpp"
    source = cpp_path.read_text(encoding="utf-8")
    header = header_path.read_text(encoding="utf-8")
    driver = driver_path.read_text(encoding="utf-8")

    check_static_wiring(source, header, driver)
    build_and_run(root, source)
    print("PASS: V55 P3 SFR/reset integration regression")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, TestFailure) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
