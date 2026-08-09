# Korg Prophecy publication provenance

This branch is a publication-oriented Korg Prophecy emulator delta on top of
official MAME.  It was created as a physically separate clone so that private
research history, refs, and unreachable objects cannot accompany a future
push by accident.

## Base and import boundary

- Official upstream: `https://github.com/mamedev/mame.git`
- Official base: `a60f95ea04a4f9b4bd950c88068cce1dcdc04b6e`
- Base date: 2026-08-08
- Local publication branch: `profligacy-public-v1`
- Sanitized source-side checkpoint: `63d4503e3dd`

The six sanitized publication commits were rebased onto this base after its
official upstream Linux, macOS, and Windows workflows all passed on 2026-08-09.
This replaced the initial February import base without importing private refs
or ancestry.

The source-side checkpoint is an identifier for the private integration
snapshot only.  Its Git objects and ancestry were not imported.  Files were
copied through an explicit allowlist onto the official base.

The allowlist contains the Prophecy driver, the TMS57002 core and standalone
test driver, the H8 and NEC/V55 changes required by the machine, focused build
scripts, and the repository-local conformance manifests.  The large oracle
manifest contains decoded microprograms, initial state, serial stimuli, and
expected sample words promoted from physical captures; it contains no Korg
firmware or ROM image.

The following stayed outside this repository:

- firmware, ROM-derived memory images, raw capture bundles, PDFs, and audio;
- the experimental TMS57002 straight-pass include and its generator;
- the research-only Prophecy HLE/compare subtree and diagnostic hooks;
- research notes, scratch programs, logs, GUI experiments, and build outputs;
- the private repository's refs and commit ancestry.

Historical snapshot tests may refer to neutral paths beneath an external
`TMS57TEST_LEGACY_ASSET_ROOT`.  Their 50-item inventory is metadata only; the
assets are not vendored.  These tests are available only through
`TMS57TEST_ONLY=legacy` and contribute zero cases to the release denominator.

## Verified at initial import

- The standalone `tms57test` target builds from the sanitized tree.
- The initial combined harness accounted for 35 entries: 23 self-contained
  passes, eleven unavailable legacy-asset diagnostics, and one incomplete
  synthetic A02 fixture.  The latter twelve were subsequently retired from the
  public denominator and retained in a separate manifest and `legacy` mode.
- The compact silicon-derived corpus passes 10/10 cases and 65/65 assertions.
- The portable hardware-oracle corpus passes 550/550 exact replay cases.
- The `propmin` target builds and links three Prophecy driver revisions.
- `propmin -validate korgprop` passes.

The first post-import cleanup removed the default-off H8 and V55 per-PC
histograms (each allocated a 64 MiB table when enabled) and the H8 idle-loop
fast-forward experiment from the shared CPU cores.  The driver still builds and
validates after their removal.

The next publication cleanup removed the remaining default-off H8/V55 trace
controls and log bodies from the shared CPU cores, plus the driver-local DSP
firmware-stack dump.  It also baked the hardware-validated DSP serial staging
mode and retired seven lifecycle inputs that no longer had a consumer.  Of the
77 previously classified tokens, 33 were removed: eleven diagnostic controls,
eight stale/compatibility inputs, and fourteen emitted labels belonging only to
the retired diagnostics.  All 44 remaining `KPROP_*` tokens are accounted for
in `tests/korgprophecy/public_controls_v1.json`; its diagnostic and stale-input
categories are now empty.  Pooled-JIT statistics are the sole diagnostic
control retained and reclassified as a deterministic test seam: the tracked
`scripts/korgprophecy_pf4_equivalence.py` gate explicitly enables and parses
those records.  The old DSP packet logger was retired because its scoreboard
consumer is not part of the public gate set.

Those were the initial-import results.  Four subsequently measured A02 launch
cases were promoted without importing private history or assets.  The current
tracked corpus therefore contains 554 exact replay cases.  A historical ledger
preserves mismatch records and hashes for 42 underdeclared captures superseded
by later explicit-seed replays and four optional fully declared recaptures.
Those 46 rows are not current emulator failures and contribute zero release
blockers.  The current public focused gate is independently 23/23 with no
blocked tests.

These results establish parity for the covered observations, not exhaustive
proof of all TMS57002 programs or full product readiness.  Full-system,
real-time, plug-in host, signing, and notarization acceptance remain separate
release gates.  `PUBLIC_ACCEPTANCE.md` records the stable publication contract;
machine-produced, exact-source receipts determine final release status.

## Audit

Run the publication audit on committed history:

```bash
python3 scripts/korgprophecy_publication_audit.py
python3 scripts/korgprophecy_control_audit.py
python3 scripts/korgprophecy_corpus_policy.py
./scripts/korgprophecy_no_rom_gate.sh
```

Before committing, stage the proposed snapshot and run:

```bash
python3 scripts/korgprophecy_publication_audit.py --cached
```

The audit verifies the official ancestry boundary, local/remote topology,
allowlisted delta paths, absence of forbidden asset extensions and private
path markers, and the absence of merge commits above the official base.
