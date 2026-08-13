# Chassis 65 mm Before Pick

## Goal

In `APP_MODE_ARM`, drive the chassis forward 65 mm once, wait until it has
stopped, then start the existing alternating arm pick/place loop.

## Phases

- [in_progress] Inspect chassis initialization, distance control, task ownership, and public API.
- [pending] Add a one-shot configurable-distance chassis operation without enabling the legacy 1 m/turn loop.
- [pending] Integrate chassis initialization/tasks and gate arm picking on successful 65 mm completion.
- [pending] Run static consistency and diff checks; do not compile or flash.

## Constraints

- Preserve unrelated dirty-worktree changes.
- Keep the existing arm pick/place behavior unchanged after chassis completion.
- Chassis faults must prevent arm picking.
- No build, flash, or hardware test.

## Errors

- None.
