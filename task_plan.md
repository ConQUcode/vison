# Task Plan: Full-Damiao 3DOF Arm Refactor

## Goal

Keep the public arm framework (`ArmInit`/`ArmTask`, joint, Cartesian and
realtime commands, FK/IK and trajectory ownership) while replacing the legacy
GM6020/M3508/M2006 arm implementation with three absolute-encoder Damiao
motors in position-speed mode.

## Locked Hardware

| Joint | Motor | Motor ID | Feedback ID | TX ID |
|---|---|---:|---:|---:|
| q1 base | DM4310 | 3 | `0x13` | `0x103` |
| q2 shoulder | DM4340 | 2 | `0x12` | `0x102` |
| q3 elbow | DM4310 | 1 | `0x11` | `0x101` |

- CAN1, 1 Mbps, standard frame, DLC 8.
- Position-speed payload uses motor radians and positive rad/s speed limit.
- Firmware never sends the zero-position command.
- Geometry is `250/260/260 mm`, with zero forward/left shoulder offset.
- Safe pose is `[0,90,-90] deg`, whose FK is `(260,0,510) mm`.

## Implementation Status

### Phase 1 - Damiao module

- [completed] Added explicit `DM4340` type.
- [completed] Added validated motor ID, feedback ID, TX ID, model and mode configuration.
- [completed] Added allocation, capacity, duplicate-ID and CAN registration failures without dead loops.
- [completed] Replaced ambiguous PID references with explicit `position_ref_rad` and `velocity_limit_rad_s`.
- [completed] Split hold, enter mode, disable and clear-fault semantics.
- [completed] Decode DLC 8 absolute position, velocity, torque, temperatures, state and counters.
- [completed] Reject non-finite/out-of-protocol-range commands.
- [completed] Removed dynamic per-motor RTOS tasks and added deterministic `DMMotorControl(now_ms)`.
- [completed] Added rotating send order, per-axis TX counters and consecutive-TX-failure latching.

### Phase 2 - Bench harness

- [completed] Added `NONE/BASE/SHOULDER/ELBOW` compile-time/runtime-visible selector.
- [completed] Default is `DM_SINGLE_AXIS_TEST + ARM_DM_TEST_NONE`.
- [completed] Passive 300 ms listen, selected-axis Enter Mode fallback, feedback-first target sync.
- [completed] Added sticky relative test request limited to 2 deg initially and 5 deg maximum.
- [completed] Added `g_arm_dm_debug` with IDs, units, limits, temperatures, state and RX/TX counters.
- [pending] Perform physical direction/no-kick/unplug tests for each motor.

### Phase 3 - Arm motor adapter

- [completed] Replaced the arm's DJI motor handles with `DM_MotorInstance` adapters.
- [completed] Centralized motor-rad/joint-deg conversion for all three axes.
- [completed] Restricted protocol units to the module boundary.
- [completed] Replaced DJI current/PID debug data with Damiao torque/state/temperature/CAN data.
- [completed] Preserved public joint, Cartesian and realtime command APIs.

### Phase 4 - Startup, escape and fault handling

- [completed] Removed stall homing, endpoint scan, ratio test, software encoder reset and 3508 feedforward paths.
- [completed] Added passive feedback, Enter Mode fallback, validation and target synchronization states.
- [completed] Added elbow-shoulder-base automatic escape within the approved hard boundary.
- [completed] Added elbow-shoulder-base return to `[0,90,-90]`.
- [completed] Added 100 ms feedback timeout, CAN TX, motor-state, hard-boundary, direction and timeout faults.
- [completed] Added 70 C motion hold/refusal and 80 C disable latching.
- [completed] Added true emergency disable and normal cancel-to-current-position hold.
- [completed] Added sticky request/applied fault reset and restart from new feedback, never old trajectory state.
- [pending] Validate every startup/fault branch on supported hardware.

### Phase 5 - Kinematics and limits

- [completed] Updated geometry to `250/260/260/0/0 mm`.
- [completed] Replaced scan-derived calibration limits with static q1/q2/q3 limits.
- [completed] Updated the self-test to `[0,90,-90] -> (260,0,510) mm`.
- [completed] Retained analytic multi-branch IK and seed-nearest continuity selection.
- [completed] Verified five FK/IK round trips; worst host error was `6.10351562e-05 mm`.

### Phase 6 - Trajectory and host commands

- [completed] Removed the old hard-stop exit corridor and automatic three-point loop.
- [completed] Disabled API point replay and realtime host simulation by default.
- [completed] Retained joint, direct Cartesian, linear Cartesian and realtime target APIs.
- [completed] Applied conservative first-pass direct-drive velocity/acceleration limits.
- [completed] READY is now based on configuration, feedback, synchronization, startup READY and no fault.
- [pending] Re-enable movement stages only in the approved physical validation order.

### Phase 7 - Verification

- [completed] ARM Cortex-M4 GCC syntax checks passed with `-Wall -Wextra -Wshadow`.
- [completed] Scoped `git diff --check` passed (line-ending conversion notices only).
- [completed] Keil ARMCC 5.06u7 serialized build passed with 0 errors and 0 warnings.
- [completed] Final image: Code 46224, RO 672, RW 916, ZI 101820 bytes.
- [completed] NORMAL-mode ARMCC link also passed: 0 errors, 2 expected unused bench-helper warnings, Code 55116, RO 676, RW 916, ZI 101820 bytes; the source and final image were restored to the safe bench default afterward.
- [completed] Map retains `DMMotorInit`, `DMMotorControl` and position-speed command paths.
- [completed] Old public homing/calibration/teach-front symbols are absent from the new arm source/map.
- [pending] Flash, CAN analyzer and physical load/direction/temperature validation.

### Phase 8 - Explicit three-axis enable gate

- [completed] Make NORMAL startup send Enter Motor Mode to all three axes before target synchronization, limit handling, escape or return motion.
- [completed] Treat CAN transmit success only as an enable request; require a fresh state-1 feedback frame from every motor before continuing.
- [completed] Keep all position-speed transmission disabled until the corresponding motor has confirmed Motor Mode and its current-position target has been synchronized.
- [completed] Expose per-axis mode confirmation and mode-command counters in `g_arm_dm_debug`.
- [completed] Rebuild the NORMAL image and verify the enable-confirmation path remains linked.

### Phase 9 - Three-axis enable-only firmware

- [completed] Add a dedicated `DM_ENABLE_ONLY` boot mode whose only purpose is to enable all three motors and hold their measured startup positions.
- [completed] Do not run NORMAL validation, escape, safe-pose return, trajectory, IK or host simulation in this mode.
- [completed] Do not interpret the currently unverified feedback state code as a reason to send Reset Mode.
- [completed] After current-position synchronization, allow continuous position-speed hold frames immediately after all three Enter Mode frames are queued successfully.
- [completed] Build the final enable-only HEX and expose an unmistakable Watch state.

### Phase 10 - Correct Damiao special-command CAN IDs

- [completed] Use raw Motor ID `1/2/3` for Enter, Reset and Clear Fault special frames.
- [completed] Keep position-speed target frames on `0x101/0x102/0x103`.
- [completed] Preserve feedback filters at `0x11/0x12/0x13`.
- [completed] Add Watch-visible `last_mode_tx_id` and rebuild the enable-only image.

### Phase 11 - Break the no-feedback enable deadlock

- [completed] Do not require passive feedback before sending Enter Motor Mode.
- [completed] After 300 ms, repeatedly send raw-ID enable frames even while all motors remain red and silent.
- [completed] Wait for feedback only before seeding the position-speed hold targets.
- [completed] Rebuild and verify the enable-only image retains the corrected raw special-command IDs.

### Phase 12 - Fast three-axis commissioning after enable success

- [completed] Preserve the now-verified three-axis enable/current-position hold path.
- [completed] Upgrade bench commissioning so one firmware can select base, shoulder or elbow at runtime without reflashing between axes.
- [completed] Keep all three motors enabled and holding while allowing only the selected axis to make a sticky requested relative move.
- [completed] Retain the first-step `2 deg`, maximum `5 deg`, `5 deg/s` and normal software-limit guards.
- [completed] Build a commissioning HEX and document the exact Watch request/ack sequence.
- [pending] After physical direction confirmation, switch the final image to NORMAL for escape and sequential safe-pose return.

### Phase 13 - Connect commissioning auto motion to trajectory planning

- [completed] Reuse the existing trajectory layer in the current `DM_SINGLE_AXIS_TEST` commissioning boot path.
- [completed] Preserve the physically verified simultaneous direct initialization; enable trajectory planning only after initialization and belt compensation are complete.
- [completed] Run the automatic XYZ test point as a preflight-checked Cartesian linear trajectory instead of sending one final IK pose.
- [completed] Preserve the live geometry, q3 convention, belt compensation, target point and enable sequence.
- [completed] Reuse concise `g_arm_dm_debug` and `g_arm_motion_debug` planning state/result without adding another large debug structure.
- [completed] Perform source/diff validation only; do not run Keil compilation per user request.

### Phase 14 - Remove the forward-safe-pose hesitation

- [completed] Merge initialization-to-safe and safe-to-XYZ into one global quintic trajectory so the safe pose is not a stop point.
- [completed] Parameterize cached joint samples by joint travel time instead of raw sample index so the 1 deg staging samples and 1 mm Cartesian IK samples do not create a reference-speed step at the waypoint.
- [completed] Complete scoped source and whitespace validation without running Keil compilation.

### Phase 15 - Four-point host-command simulation

- [completed] Replace the one-shot XYZ test with the requested four-point loop.
- [completed] Send point 1 immediately after initialization, then select the next point every 3000 ms.
- [completed] Preserve the rounded composite entry path for the first point and use preflight-checked Cartesian lines between subsequent points.
- [completed] Validate state transitions, point order and source formatting without running Keil compilation.

### Phase 16 - Host-ready arm application interface

- [completed] Add a communication-independent `arm_host.h` with a unified command, command lifecycle and concise status snapshot.
- [completed] Add a single-slot non-blocking command mailbox processed only by `ArmTask()`, with emergency/cancel/reset priority.
- [completed] Keep existing joint/Cartesian/realtime APIs as compatibility wrappers around the unified interface.
- [completed] Make the commissioning startup reach `[0,90,-90]` before reporting host READY and disable the four-point loop by default.
- [completed] Perform source, API-use and whitespace validation without running Keil compilation.

### Phase 17 - USART6 Huaner LX servo driver

- [completed] Replace the internal Feetech/SCS protocol in `hsl_servo` with the Huaner LX `55 55` protocol on USART6.
- [completed] Add one DMA transaction state machine for move, stop and position-read commands with strict frame validation and timeouts.
- [completed] Extend the USART BSP with compatible TX/RX/error callback registration while preserving existing Receive-to-Idle users.
- [completed] Add concise status/debug snapshots and keep legacy Feetech APIs as non-transmitting rejected stubs.
- [completed] Initialize and service the driver from the existing application init/1 ms command path without modifying `catch.c`.
- [completed] Perform source/frame/whitespace validation only; do not run Keil compilation.

### Phase 18 - Single-owner arm boot sequence

- [completed] Move the one-shot TOOL_TIP commissioning motion out of `Test.c` and into an explicit `ArmTask()` boot sequence.
- [completed] Make `ArmTask()` the only owner of enable, auto-init, tool-init wait, stabilization, test motion and final READY publication.
- [completed] Remove the internal boot test's dependency on the public host mailbox and `g_arm_host_status.ready`.
- [completed] Keep `Test.c` as scheduling/observation only and disable its duplicate command submission path.
- [completed] Add one concise Watch-visible boot state/result structure and run source/diff validation without a Keil build.

### Phase 19 - USB host protocol and compound arm actions

- [completed] Replace the old `CmdVel` protocol with the puzzle-arm protocol (`TaskStatus`, `CartesianMotionCommand`, `MotionStatus`, ACK and heartbeat).
- [completed] Add a USB CDC copy-based TX queue so protocol frames never point at stack buffers.
- [completed] Add an `ArmUsbBridge` state machine for `MOVE`, `MAGNET_ON`, `MAGNET_OFF`, `HOME`, `STOP` and `TaskStatus END`.
- [completed] Add internal Cartesian yaw support for ID2 servo commands without changing Damiao geometry, ID1 compensation, USART6 servo transport, or CubeMX USB configuration.
- [completed] Move buzzer use to a reusable non-blocking module and use it for task-complete notification.
- [completed] Disable the current power-on tool/magnet/ID2 test for the formal USB-control firmware.
- [completed] Run source inspection and `git diff --check`; do not run Keil build unless explicitly requested.

### Phase 20 - USB command rejection and transport recovery

- [completed] Split Cartesian preflight rejection from running-motion failure so an unreachable target reports `FAILED` and immediately releases the USB business lock without submitting a redundant cancel command.
- [completed] Make `FAULT_RETRY` clear recoverable bridge state when no real arm fault or motion remains.
- [completed] Add USB CDC TX timeout/re-enumeration recovery and reserve a dedicated high-priority queue for command ACK traffic.
- [completed] Expose concise TX queue recovery diagnostics and run source/diff validation without a Keil build.

### Phase 21 - Non-blocking USART6 servo move transmission

- [completed] Keep the verified USART6 `9600`, half-duplex wiring and controller-board frame format unchanged.
- [completed] Replace blocking servo move transmission, TC polling and the blocking 2 ms delay with USART6 interrupt transmission plus task-driven completion/gap states.
- [completed] Register the existing servo callbacks through the shared USART BSP and preserve single-transaction `OK/BUSY` semantics.
- [completed] Keep both single-servo and existing dual-servo frame APIs compatible without changing arm compensation, timing or angle mapping.
- [completed] Run ARM GCC syntax checking and scoped `git diff --check`; leave Keil build, flash and hardware timing validation to the user.

### Phase 22 - Latest-target coalescing and dual-servo frames

- [completed] Add one overwriteable latest pending target slot for ID1 and ID2 instead of a historical FIFO.
- [completed] Dispatch matching-time ID1/ID2 targets through `HSLServoMove2()` and keep single-servo fallback when only one target changes or times differ.
- [completed] Preserve USART6 configuration, controller-board frames, position mapping, compensation formulas, update periods, repeat counts and motion-time parameters.
- [completed] Keep initialization from advancing until cached targets are dispatched and the USART6 transaction is complete.
- [completed] Clear unsent stale targets on tracking stop/fault and require a fully idle tool TX path before boot stabilization.
- [completed] Run ARM GCC syntax checking and scoped `git diff --check`; leave Keil build, flash and hardware validation to the user.

## Environment Notes

- PowerShell startup fails with `8009001d`; native `cmd.exe` is used.
- Host GCC requires project-local `TEMP/TMP`; otherwise it attempts to write under `C:\WINDOWS`.
- Keil is installed at `C:\Keil_v5` and builds with project-local `MDK-ARM/tmp`.
- Existing user/IDE changes in `.vscode/BROWSE.VC.DB`, `Engineer.uvguix.11737`
  and `Engineer.uvoptx` remain untouched.
- 2026-07-30: a compound `cmd.exe` `rg` inspection failed because cmd parsed the quoted search expression as commands; use `C:\w64devkit\bin\bash.exe` for compound source inspection.
- 2026-07-30: a multiline Python `-c` path simulation failed under `cmd.exe` with an unterminated string; use Bash stdin/heredoc for the numerical reproduction.
- 2026-07-30: the first Phase 18 combined patch did not apply because a mojibake comment in `arm_config.h` made the context unstable. No source changes were made; subsequent patches use stable macro/function boundaries.
- 2026-08-01: one combined Phase 20 cleanup patch did not apply after `usb.c` context changed during the same turn. No part of that patch was applied; the cleanup is split into small symbol-bound patches.
- 2026-08-01: the first optional GCC syntax-check wrapper failed before compilation because the Windows Bash invocation rejected its array syntax. No build artifacts or source files were changed; validation continues with explicit commands.
- 2026-08-01: the first Phase 22 validation wrapper was rejected by the command safety layer because it included temporary-directory deletion. No source or build artifact was changed; validation was rerun without deletion and passed.
