# Profligacy public-MAME acceptance contract

This is the publication boundary for the sanitized MAME dependency consumed by
Profligacy.  A green row proves only the named observation.  Final product
status comes from machine-produced receipts bound to the exact Profligacy tree
and dependency commits; this tracked document is not a substitute for them.

Official MAME base: `a60f95ea04a4f9b4bd950c88068cce1dcdc04b6e`

Publication branch: `profligacy-public-v1`

Supported release target: native Apple Silicon, macOS 15.0 or later.  Intel and
older macOS targets are not certified for v1.

## Asset-free core contract

| Gate | Required result |
| --- | --- |
| Publication history/path/content audit | all delta paths allowlisted; zero forbidden paths, assets, or private markers |
| `KPROP_*` disposition audit | every token classified; zero diagnostic and zero stale/compatibility controls |
| Public focused suite | 23 pass, 0 fail, 0 error, 0 blocked |
| Compact silicon-derived corpus | 10/10 cases, 65/65 assertions |
| Portable hardware-oracle corpus | 554/554 exact cases |
| Corpus classification policy | 42 historical rows `legacy_superseded`, four `recapture_optional`, all linked to stronger exact replays, zero release blockers; 12 retired research tests contribute zero cases |
| Prophecy `propmin` build/link | all three supported system revisions build and link |
| `propmin -validate korgprop` | PASS |

The asset-free gates contain no Korg firmware, ROM-derived memory image, raw
capture bundle, private research history, or generated straight-pass
implementation.  The 554 replay cases retain decoded tiny programs, initial
state, serial stimuli, and output words promoted from physical captures.  They
do not prove behavior outside those exact observations and bounded alignments.

## Product integration contract

The exact public dependency must be exercised through the final Profligacy host
and compared with the accepted research behavior where applicable:

| Gate | Required result |
| --- | --- |
| DSP hardware corpus | 554/554 exact and corpus-classification policy PASS |
| Timestamped MIDI matrix | every generated case passes its firmware invariants and strict reference parity |
| Control ABBA repeatability | every pair and both run orders pass exact audio, MIDI, NVRAM, panel, LCD, LED, ADIN, and readback comparisons |
| Product PCM matrix | research direct/host and public interpreter/JIT/host PCM are byte-identical |
| Same-process lifecycle | 10/10 start/render/stop/destroy cycles on each side, with no stale singleton/callback/thread state |
| Real-time safety | no allocation after preparation, bounded oversize behavior, and observable queue drops |
| Performance | balanced order-independent primary/control gate passes on an otherwise idle machine; scheduler-noisy evidence remains inconclusive, never green |

The v1 host contract permits one active Profligacy synth instance per process.
Additional constructed instances must remain inert and must not control or
mirror the active synth.

## Publication gates outside this MAME tree

The final Profligacy release additionally requires the repeated real editor
gate, exact-path pluginval for AU and VST3, auval, a representative DAW smoke
test, source-archive provenance, SBOM/notices review, Developer ID signing, and
Apple notarization/stapling.  Firmware and ROM images are user-supplied and are
never part of the source or binary distribution.

Any changed source, dependency commit, binary, or receipt artifact invalidates
the affected result.  Do not weaken comparisons or regenerate expected values
from the implementation under test to make a release gate pass.
