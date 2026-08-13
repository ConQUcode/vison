# Segmented Chassis Move And Three Picks

## Goal

In `APP_MODE_ARM`, run a finite sequence: drive 585 mm, pick/place point 1,
drive 500 mm, pick/place point 2, drive 500 mm, pick/place point 1, then stop.

## Phases

- [complete] Inspect chassis initialization, distance control, task ownership, and public API.
- [complete] Add a one-shot configurable-distance chassis operation without enabling the legacy 1 m/turn loop.
- [complete] Integrate chassis initialization/tasks and gate arm picking on successful 650 mm completion.
- [complete] Run static consistency and diff checks; do not compile or flash.
- [complete] Add repeatable one-shot straight segments after the first move.
- [complete] Limit the test to three pick/place operations: point 1, 2, 1.
- [complete] Reduce contact-relief attempts from five to four.

## Constraints

- Preserve unrelated dirty-worktree changes.
- Keep the existing arm pick/place behavior unchanged after chassis completion.
- Chassis faults must prevent arm picking.
- No build, flash, or hardware test.

## Errors

- `rg` patterns containing spaces/pipes were parsed by `cmd`; subsequent searches use separate `-e` patterns or simple tokens.
- PowerShell startup failed with `8009001d`; continued in native `cmd.exe`.
- Two broad patches were rejected by encoded Chinese comment anchors; no partial edits landed, and the changes were reapplied with stable symbol anchors.
