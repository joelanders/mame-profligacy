# TMS57002 conformance corpus

This directory contains the repo-local, hardware-backed regression contract for
the Korg Prophecy TMS57002 work.  The corpus is intended to make DSP semantic
changes reviewable and to keep the clean publication branch aligned with the
research branch without importing the research branch's diagnostic machinery.

## What is covered

- `tms57002_oracle_replays_v1.json` contains 554 exact replay cases promoted
  from decoded physical TMS57002 captures.  The current composition is 465
  DOMH boundary/rounding/saturation cases and 89 MAC/MPY latency, store/read,
  accumulator-order, A02-tail, and LIRA cases.
- `tms57002_oracle_gaps_v1.json` preserves the original mismatch evidence and
  source hashes for 46 historical captures whose recipes underdeclared serial,
  arithmetic-pipeline, or output-latch state.  Later explicit-seed hardware
  replays supersede 42; four remain optional fully declared recaptures.  All 46
  are outside the release denominator and are not current emulator failures.
- `tms57002_known_semantics_v1.json` names ten self-contained silicon-derived
  instruction tests: AOVM `INT32_MIN`, the paired full-width MAC A,D and 24-bit
  multiplier A-port rules, raw SMLD low-port, MPY+DIS ordering, rounded MOVM
  positive-rail behavior, RDE/SACC and WRE/SMHD XRAM co-issue ordering, and the
  left-subframe DOS deadline, plus the sequential PC-wrap sample halt.
- `tms57002_legacy_assets_v1.json` inventories all 50 external dependencies of
  the older snapshot tests: 47 files and 3 directories.
- `tms57002_retired_research_tests_v1.json` accounts for eleven asset-dependent
  research diagnostics and one incomplete synthetic fixture.  They remain
  available through the explicit `legacy` mode but contribute zero tests to
  the public release denominator.

The public `TMS57TEST_ONLY=all` harness contains 23 self-contained tests and
requires 23 pass, zero fail, zero error, and zero blocked.  Missing private
assets cannot make the public gate green or red because legacy research tests
are no longer mixed into it.

## Build and run

Build the standalone harness:

```bash
JOBS=16 ./scripts/tms57002test_build.sh
```

Run the 554-case hardware replay corpus:

```bash
python3 scripts/korgprophecy_tms57002_oracle_corpus.py run \
  --binary ./tms57test --worktree . --label clean
```

Run the ten-case focused semantics manifest:

```bash
python3 scripts/korgprophecy_tms57002_corpus.py \
  --binary ./tms57test --worktree . --label clean
```

Run the complete public focused suite:

```bash
env SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  TMS57TEST_ONLY=all \
  ./tms57test tms57002test -verbose -video none -sound none -seconds_to_run 1
```

Run the retired diagnostics only when doing research archaeology with the exact
historical bundle:

```bash
env SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  TMS57TEST_LEGACY_ASSET_ROOT=/path/to/exact-bundle \
  TMS57TEST_ONLY=legacy \
  ./tms57test tms57002test -verbose -video none -sound none -seconds_to_run 1
```

The Python runners set the SDL dummy video and audio drivers themselves.  Keep
those variables on direct harness invocations as well; `-video none` alone can
still initialize the host GUI on macOS.

## Full-system lifecycle equivalence

The unit corpus does not cover scheduler-visible interactions between a DSP
frame and the H8 control strobes. Keep that boundary as a separate full-system
equivalence gate:

```bash
python3 scripts/korgprophecy_lifecycle_equivalence.py \
  --reference-bin /path/to/research/propmin \
  --candidate-bin ./propmin \
  --rompath /path/to/00-roms
```

At clean integration revision `99caaa8dec5`, the research and clean builds
produce the same 460 DSP1 serial lifecycle rows across the 0.174..0.177 second
boot/PLOAD window. The earlier four-row discrepancy was caused by the stale
legacy V55 board transport and closed when the manual-faithful UART0/INTST0 path
became the clean shipping default. This remains an equivalence gate rather than
a generated hardware golden: any future mismatch stays red.

Audit or install the legacy asset bundle:

```bash
python3 scripts/korgprophecy_tms57002_legacy_assets.py
python3 scripts/korgprophecy_tms57002_legacy_assets.py --root /path/to/bundle
```

The audit exits 2 when the bundle is absent or incomplete and exits 0 only when
every declared dependency is present, hashes match where known, and the source
and manifest inventories agree.

## Evidence boundary

The 554 green cases establish parity for their exact captured programs, initial
state, driven serial wires, and bounded alignment windows.  They are not a proof
over every possible TMS57002 program.  The separate historical ledger preserves
42 superseded underdeclared captures and four optional recaptures without
treating them as release failures.  `korgprophecy_corpus_policy.py` enforces
these denominators and verifies that every archived row links to stronger exact
hardware-replay evidence.
