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

### Phase 23 - Align ID1/ID2 tracking cadence

- [completed] Keep ID1's verified 20 ms tracking cadence unchanged.
- [completed] Reduce only ID2 base/yaw compensation update period from 30 ms to 20 ms so both target generators run at 50 Hz and reach the dual-frame scheduler together more often.
- [completed] Preserve USART6 9600 baud, the 2 ms board gap, deadbands, mappings, compensation formulas, repeat sends and motion-time parameters.
- [completed] Run ARM GCC syntax checking and scoped `git diff --check`; leave Keil build, flash and hardware validation to the user.

### Phase 24 - Generate and dispatch both servo targets in one arm cycle

- [completed] Move the existing `ArmToolTask()` service point from the beginning of `ArmTask()` to after the current-cycle main-arm feedback, trajectory and ID1/ID2 compensation calculations.
- [completed] Let ID1 vertical compensation and ID2 base/yaw compensation reach the same latest-target scheduler before it decides between `HSLServoMove2()` and single-servo fallback.
- [completed] Preserve one `ArmToolTask()` call per arm cycle and retain initialization, repeat-send, fault, timeout and asynchronous USART6 ownership semantics.
- [completed] Keep the Phase 23 20 ms cadence, USART6 settings, mappings, formulas, deadbands, repeat parameters and motion times unchanged.
- [completed] Run ARM GCC syntax checking and scoped `git diff --check`; leave Keil build, flash and hardware validation to the user.

### Phase 25 - Main-arm speed tier and feedback-confirmed completion

- [completed] Raise formal USB MOVE speed/acceleration to the approved second tier while keeping HOME, magnet Z motion, startup and servo mappings unchanged.
- [completed] Make prepared trajectories enter a feedback-settling state after the reference profile finishes instead of reporting completion immediately.
- [completed] Require all three joints to remain within position and velocity thresholds for a continuous stability window before publishing HOLDING/COMPLETED to the host.
- [completed] Add a bounded settling timeout that reports the existing motion timeout fault and releases the command lifecycle cleanly.
- [completed] Expose concise settling/arrival diagnostics through the existing arm debug snapshots and verify the USB bridge only reports COMPLETED after feedback confirmation.
- [completed] Run ARM GCC source checks and scoped `git diff --check`; do not run Keil or hardware tests unless explicitly requested.

### Phase 26 - Relaxed arrival gate and non-freezing timeout recovery

- [completed] Relax the feedback-confirmed arrival gate from `1 deg / 2 deg/s` to `2 deg / 5 deg/s` while retaining the 120 ms continuous stability requirement.
- [completed] Keep the approved `700 mm/s` top speed but reduce Cartesian acceleration from `7200` to `5000 mm/s2` to reduce q2/q3 lag and end-of-path shake.
- [completed] Distinguish settling timeout from real arm/tool faults in the USB bridge.
- [completed] On settling timeout, report the existing timeout result but do not submit `CANCEL_MOTION` or overwrite the final target with the lagging feedback pose.
- [completed] Preserve the existing cancel behavior for non-timeout faults and run ARM GCC source checks plus scoped whitespace validation without a Keil build.

### Phase 27 - X-adaptive default and magnet Z mapping

- [completed] Replace the fixed USB default and magnet-action Z heights with configurable endpoint values over X=240..450 mm.
- [completed] Clamp X below/above the calibrated range and linearly interpolate both heights inside the range.
- [completed] Apply the mapped default height to normal targets, loaded-motion lift/final stages and magnet return motion.
- [completed] Apply the mapped magnet height to Task 5/6 descent while freezing the accepted action X for the complete compound action.
- [completed] Keep fixed HOME coordinates unchanged and run ARM GCC syntax plus scoped whitespace checks without a Keil build.

### Phase 28 - Generated protocol hash and STOP workflow

- [completed] Synchronize the generated protocol hash and new `STATUS_STOP=3` value without removing the firmware's USB queue, retry recovery or debug extensions.
- [completed] Add an idempotent TaskStatus STOP business flow: cancel the current action, preserve the magnet while returning HOME, then release the magnet and reliably return `CallbackStatus{task_id, STOP}`.
- [completed] Keep all packet IDs, payload layouts, ACK/de-duplication, heartbeat echo and reliable queue behavior compatible with the new generated files.
- [completed] Synchronize the project protocol document with the new hash/STOP semantics and verify source structure, syntax and whitespace without a Keil build.

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
- 2026-08-01: the first combined Phase 28 STOP patch was rejected cleanly because the live `TargetControl` formatting did not match one patch anchor. No source change from that patch was applied; implementation continued with symbol-sized patches.

### Phase 29 - Gripper tool and differential chassis refactor

- [in_progress] Replace the ID1 vertical-compensation and ID2 world-yaw tool model with an absolute-pitch axis and an OPEN/CLOSE gripper while preserving the three-Damiao arm, belt coupling and arm trajectory ownership.
- [pending] Replace the unused legacy swerve chassis with CAN2 M3508 ID1/ID2 differential drive, IMU-heading odometry, heading hold and relative/absolute turns.
- [pending] Extend the reliable USB protocol with pitch/tool and chassis messages while retaining ACK priority, duplicate suppression and retry behavior.
- [pending] Restore `INS_Init()`/1 kHz `INS_Task()`, integrate task scheduling and add arm/chassis motion interlocking.
- [pending] Run scoped GCC checks, full Keil ARMCC build, map/source symbol audit and `git diff --check`; hardware calibration remains external.
- [pending] Keep the user-owned `.vscode/BROWSE.VC.DB*` changes untouched.

### Phase 29 errors

- 2026-08-02: the first planning-record patch used a truncated Phase 28 line as context and was rejected cleanly; no file changed. Subsequent planning updates use stable EOF context.
- 2026-08-02: compound `cmd.exe` searches containing `|` were parsed as shell pipes, and one bash inspection lost Windows backslashes. Source inspection now uses literal single-pattern searches or forward-slash paths.
## Phase 30 - Chassis-only one metre test

- [x] Restore incomplete arm refactor source changes after the user narrowed scope.
- [x] Replace legacy chassis application with two-wheel CAN2 M3508 control.
- [x] Run INS at 1 kHz and use YawTotalAngle for straight-line heading hold.
- [x] Add encoder odometry and a staged 1.0 m automatic motion state machine.
- [x] Disable arm initialization and periodic arm/Damiao control in this test image.
- [x] Add motor-offline, IMU-invalid, timeout, overshoot, and stable-stop handling.
- [x] Verify with ARMCC build and source diff checks.

## Phase 31 - Huaner dual-servo feedback supervision

- [completed] Confirm CubeMX now generates USART6 asynchronous 115200 8N1 on PG14 TX and PG9 RX, RX DMA2 Stream1 Channel5, TX DMA2 Stream6 Channel5 and priority-5 IRQ handlers.
- [completed] Implement controller-board `0x15` dual-position request/reply framing and remove the incompatible direct-servo checksum parser from the active path.
- [completed] Add one-owner RX-to-idle/TX DMA transactions, strict response validation, timeout recovery and DMA-buffer reservation before frame publication.
- [completed] Implement 20 ms polling, separate target/feedback positions, measured velocity, arrival stability, stale/offline detection and Watch diagnostics.
- [completed] Make the active firmware feedback-only by default, keep arm/chassis motor control disabled and require an explicit sticky Watch request for any servo movement.
- [completed] Connect feedback snapshots to arm-tool diagnostics without re-enabling the arm runtime, and add an on-boot protocol parser self-test.
- [completed] Pass ARM GCC/ARMCC source checks, scoped whitespace validation and Keil link (`0 errors`, only existing legacy warnings); leave flash, captured reply validation and physical motion tests external.

## Phase 32 - Align the ID1 bench test with the controller-board reference

- [completed] Compare the live USART6 and `hsl_servo` implementation against the ESP32 `9600 8N1`, command `0x03` and feedback command `0x15` reference.
- [completed] Preserve asynchronous TX/RX DMA and strict feedback validation while correcting only confirmed protocol/timing mismatches.
- [completed] Keep the active image limited to the ID1 0/90 degree, five-second sweep with Watch-visible feedback.
- [completed] Run scoped syntax/build/frame checks and document hardware-only validation boundaries.

### Phase 32 errors

- 2026-08-11: `session-catchup.py` could not determine the home directory under the MSYS2 Python runtime. Existing planning files and live diffs were read directly instead; no source file was changed by the failed recovery command.
- 2026-08-11: The first UV4 build inherited an unusable system temporary directory and reported `Failed to create temporary file name` plus `Target not created`, despite a misleading zero-error summary. The retry uses the verified project-local `MDK-ARM/tmp` directory and must recreate `Engineer.axf/.hex` before the build can pass.

## Phase 33 - Concise ID1 servo Watch telemetry

- [completed] Reduce `g_huaner_servo1_debug` to initialization, online/feedback validity, target angle and measured current angle.
- [completed] Move sweep timing, command state and counters into private `Test.c` runtime state without changing the verified 0/90 degree five-second motion behavior.
- [completed] Do not expose fabricated current, voltage or temperature values because the verified controller-board `0x15` reply only contains servo ID and position.
- [completed] Run source reference checks, scoped `git diff --check` and a Keil ARMCC build with project-local temporary files.

### Phase 33 verification

- Keil ARMCC 5.06u7 build generated current AXF/HEX/MAP with `0 errors / 0 warnings`; image size is Code 42100, RO-data 628, RW-data 444 and ZI-data 97604 bytes.
- Scoped whitespace checks pass for `Test.c`, `Test.h` and planning records. Full-worktree `git diff --check` still reports whitespace in Keil-generated build logs/dependency files.

## Phase 34 - Controller-board voltage telemetry

- [completed] Add controller-board command `0x0F` request/reply support to the existing single-owner USART6 DMA transaction state machine.
- [completed] Poll board voltage at 500 ms without delaying the existing 50 ms ID1 position polling or changing the 0/90 degree five-second sweep.
- [completed] Expose only `board_voltage_valid` and `board_voltage_v` in the concise application Watch snapshot.
- [completed] Add deterministic protocol self-tests, run source/reference checks, scoped `git diff --check` and a Keil ARMCC build.
- [completed] Leave servo temperature and current telemetry unimplemented until controller-board documentation or external sensing hardware is available.

### Phase 34 verification

- Keil generated current AXF/HEX/MAP with `0 errors`; Code 42992, RO-data 636, RW-data 444 and ZI-data 97636 bytes.
- The 10 build warnings are unchanged legacy `catch.c/catch.h` warnings. `Test.c` and `hsl_servo.c` compile without warnings.
- MAP retains the voltage request, parser and validator, and the concise `g_huaner_servo1_debug` snapshot is 16 bytes.
- Scoped `git diff --check` passes. Flashing, captured `0x0F` reply validation and physical voltage comparison remain external.

### Phase 34 errors

- UV4 returned exit code 1 because the full build emitted 10 legacy `catch.c/catch.h` warnings; the build log itself confirms successful link/HEX generation with 0 errors.

## Phase 35 - Absolute-pitch tool and closed-loop gripper

- [completed] Replace the ID1 empirical vertical compensation with the 240-degree absolute-pitch mapping (`500` aligned with the small link, software range `125..875`) and the 30 mm tool-center model.
- [completed] Replace ID2 world-yaw/magnet semantics with the closed-loop gripper positions `boot=1000`, `ready=850`, `open=800`, `close=950` and software range `800..1000`.
- [completed] Add ID2-only close/boot stall detection, 10-position relief, `HELD_CONTACT` versus `CLOSED_EMPTY` classification, and feedback/timeout safety behavior.
- [completed] Integrate pitch and gripper state into arm trajectories, startup readiness, Task 5/6, STOP, host status and the reliable USB protocol.
- [completed] Restore the production arm/protocol schedule while retaining the ID1 sweep as a compile-time-disabled bench helper; leave chassis, IMU and Damiao configuration unchanged.
- [completed] Run protocol self-tests, scoped strict source checks, Keil ARMCC build, map/symbol audit and `git diff --check`; physical stall/load validation remains external.

### Phase 35 locked decisions

- Both Huaner servos use a 240-degree `0..1000` position scale through the controller board.
- ID1 relative pitch uses `position = 500 + relative_pitch_deg * 1000 / 240`; commands outside `125..875` are rejected, never clamped.
- ID2 stall detection applies only while moving toward boot `1000` or close `950`; voltage is not a stall input.
- Business close contact backs off 10 positions and completes as `HELD_CONTACT`; reaching 950 completes as `CLOSED_EMPTY`.
- Boot-to-1000 stall backs off 10, latches initialization failure and prevents READY.
- Main-arm READY triggers ID2 ready position 850; STOP opens to 800 and leaves it open.

### Phase 35 verification

- Keil ARMCC 5.06u7 full rebuild generated current AXF/HEX/MAP with `0 errors / 0 warnings`; image size is Code 90556, RO-data 764, RW-data 1008 and ZI-data 106588 bytes.
- MAP retains `ArmToolSetPitchDeg`, `ArmToolSetGripper`, `ArmToolGetCenterFromWrist`, `ArmUsbBridgeOnToolControl` and `protocol_send_tool_status`.
- Old vertical-compensation, world-yaw and magnet business symbols are absent from source/MAP. The main-arm belt coupling and Damiao geometry remain intact.
- Protocol payload capacity is 32 bytes and the new protocol hash is `0x1E7AC5B2`.
- Scoped `git diff --check` passes; the only output is Git's existing LF-to-CRLF conversion notice.
- Physical flashing, direction/range confirmation, contact relief, jam classification, feedback-unplug and temperature/load validation remain external.

### Phase 35 errors

- 2026-08-11: `session-catchup.py` again failed because the MSYS Python runtime could not determine the home directory. Live sources, `git diff` and the existing planning records are used as the recovery source; the command will not be retried.
- 2026-08-11: the first combined planning-record patch used a non-matching `findings.md` heading and was rejected atomically; no file changed. Planning records are appended separately using their live EOF anchors.
- 2026-08-11: PowerShell startup failed with host error `8009001d`; final read-only checks use native `cmd.exe`. Quoted multi-pattern `rg/findstr` commands were split by `cmd`, so final checks use literal single-pattern commands instead.

## Phase 36 - Preserve arm interfaces and restore chassis 1 m test image

- [completed] Add concise Chinese comments to the public arm-tool, arm-host and tool-protocol interfaces without changing their ABI or runtime behavior.
- [completed] Add an explicit compile-time chassis-test selector and make it mutually exclusive with the isolated ID1 sweep image.
- [completed] Restore the chassis-only initialization path: USB/protocol/buzzer, one-time `INS_Init()` and CAN2 M3508 ID1/ID2 `ChassisInit()`; do not initialize the arm or USART6 tool driver.
- [completed] Restore the periodic chassis path: 1 kHz `INS_Task()`, 5 ms internal `ChassisTask()` control/odometry, and 1 kHz `DJIMotorControl()`; do not call `ArmTask()` or `DMMotorControl()`.
- [completed] Keep the 1.0 m auto-start distance, IMU heading correction, odometry, Watch snapshot and all Phase 30 safety limits unchanged.
- [completed] Run scoped source checks, Keil ARMCC rebuild, map-symbol audit and `git diff --check`; flashing and wheels-off-ground validation remain external.

### Phase 36 locked decisions

- The final image is the chassis 1 m test image; the production arm implementation remains compiled and documented but is not initialized or periodically serviced.
- `HUANER_SERVO_ID1_SWEEP_TEST_ONLY` remains available but defaults off and cannot be enabled together with the chassis test.
- Chassis test mode does not initialize `HSLServo`, `ArmInit`, `ArmUsbBridgeInit` or any Damiao motor instance.
- Chassis test mode does not parse USB protocol frames; its USB task only services the buzzer and `DaemonTask()` motor-online supervision.
- Existing chassis geometry, direction signs, 95 mm wheel diameter, 19.2032 reduction ratio, temporary 320 mm track width and IMU yaw sign are not retuned in this phase.

### Phase 36 errors

- 2026-08-11: several read-only `rg` commands containing quoted spaces or alternation were split by native `cmd.exe`, and the first INS source lookup assumed the wrong directory. No file changed; subsequent inspection uses `rg --files` plus one literal symbol per command.
- 2026-08-11: the first combined MAP region search repeated the known quoted-alternation issue and returned no result. It was replaced by literal `Grand.Totals`, `Load.Region` and `Execution.Region` searches; no build artifact was affected.

### Phase 36 verification

- Keil ARMCC 5.06u7 final full rebuild generated current AXF/HEX/MAP with `0 errors / 0 warnings`; image size is Code 64976, RO-data 2984, RW-data 1360 and ZI-data 98568 bytes.
- MAP retains `ChassisInit`, `ChassisTask`, `ChassisNotifyImuUpdate`, `INS_Init`, `INS_Task`, `DJIMotorControl` and `g_chassis_debug` in the final image.
- MAP explicitly removes `ArmInit`, `ArmTask`, `DMMotorInit`, `DMMotorControl`, `HSLServoInit`, `USB_ProcessTask`, `protocol_tick`, arm protocol callbacks and `ArmUsbBridgeTask`, proving the active image cannot run or indirectly submit the arm/tool chain.
- Flash load region is `0x10ec8 / 0x100000`; SRAM1 execution region is `0x18330 / 0x1c000`; SRAM2 is `0x0328 / 0x4000`. No region overflows.
- Scoped `git diff --check` passes with existing LF-to-CRLF notices only. Physical flashing, wheel-direction confirmation and 1 m floor test remain external.

## Phase 37 - Reverse chassis IMU feedback sign

- [completed] Change only the centralized `CHASSIS_IMU_YAW_SIGN` from `+1.0f` to `-1.0f` after the first hardware observation showed the heading correction direction was reversed.
- [completed] Rebuild the chassis-only image and verify the same sign is applied to both `YawTotalAngle` and IMU Z-axis angular velocity; leave wheel command/feedback signs and PID gains unchanged.

### Phase 37 verification

- Keil ARMCC 5.06u7 full rebuild passed with `0 errors / 0 warnings`; final image remains Code 64976, RO-data 2984, RW-data 1360 and ZI-data 98568 bytes.
- Source audit confirms `CHASSIS_IMU_YAW_SIGN=-1.0f` is applied to zero capture, odometry yaw, previous-yaw tracking and gyro-Z derivative feedback.
- MAP retains `ChassisTask` and removes `ArmTask` plus `DMMotorControl`; the chassis-only runtime selection is unchanged.
- Scoped `git diff --check` passes with existing line-ending notices only. Physical correction-direction confirmation remains pending.

## Phase 38 - Switch from chassis test to production arm test

- [completed] Set `CHASSIS_ONE_METER_TEST_ONLY=0` while keeping `HUANER_SERVO_ID1_SWEEP_TEST_ONLY=0`, selecting the existing production-arm branch through `APPLICATION_ARM_RUN_ENABLE`.
- [completed] Rebuild and verify `ArmInit`, `ArmTask`, `DMMotorControl`, `DJIMotorControl`, USB protocol processing and `ArmUsbBridgeTask` are linked, while `INS_Init`, `INS_Task`, `ChassisInit` and application `ChassisTask` are removed.
- [completed] Preserve the chassis implementation and reversed IMU sign in source for later mechanical repair; do not retune arm, gripper, chassis or IMU parameters.

### Phase 38 verification

- Keil ARMCC 5.06u7 full rebuild passed with `0 errors / 0 warnings`; final arm image is Code 90556, RO-data 764, RW-data 1008 and ZI-data 106588 bytes.
- MAP retains `ArmInit`, `ArmTask`, `DMMotorControl`, `DJIMotorControl`, `HSLServoInit`, `USB_ProcessTask`, `protocol_tick`, arm protocol callbacks and `ArmUsbBridgeTask`.
- MAP removes `INS_Init`, `INS_Task`, `ChassisInit` and application `ChassisTask`; the misleading FreeRTOS wrapper name `ChassisTask_f` remains only as the 1 kHz `all_cmd_Task()` thread.
- Scoped `git diff --check` passes with existing line-ending notices only. Flashing and physical arm/gripper validation remain external.

## Phase 39 - Full arm initialization with ID1-axis Cartesian endpoint

- [completed] Keep the verified combined tool/Damiao initialization sequence active and give its boot mode an explicit full-initialization name.
- [completed] Change HOME to the ID1 pitch-servo axis target `(225.1666, 0, 192.0) mm`, corresponding to `q=[0,90,-60] deg` and shoulder physical angle `-90 deg`.
- [completed] Resolve HOME through the existing 3DOF IK, software limits and automatic-safe-region checks before issuing any motor target.
- [completed] Make direct, linear and realtime Cartesian commands consistently interpret their endpoint as the ID1 pitch-servo axis, while retaining the 30 mm gripper-center geometry for diagnostics only.
- [completed] Update host/protocol comments and protocol hash for the endpoint-semantic change without changing packet layout.
- [completed] Run numerical FK/IK checks, Keil ARMCC rebuild, MAP audit and scoped `git diff --check`; flashing and physical motion remain external.

### Phase 39 locked decisions

- The controlled Cartesian endpoint is the ID1 pitch-servo rotation axis, not the gripper center 30 mm beyond it.
- The selected HOME pose is `q1=0 deg`, `q2=90 deg`, `q3=-60 deg`; with the live `62/260/260 mm` geometry its axis position is `(225.1666, 0, 192.0) mm`.
- The real gripper center remains available in the internal tool state for geometry inspection, but it is not subtracted from host Cartesian targets.
- The existing startup order remains: obtain both servo feedbacks, command ID1=500 and ID2=1000, enable/synchronize the three Damiao motors, move to HOME, command ID2=850, then publish READY.

### Phase 39 verification

- Live kinematics code gives FK `[0,90,-60] = (225.166580,0,192.000000) mm`; error against the configured HOME is `0.000015 mm`.
- IK of the configured HOME from startup seed `[0,180,-90]` returns `[0,90,-60]`; software limits and automatic-safe-region checks both pass.
- Keil ARMCC 5.06u7 full rebuild generated current AXF/HEX/MAP with `0 errors / 0 warnings`; totals are Code 90776, RO-data 764, RW-data 1016 and ZI-data 106620 bytes.
- MAP retains `ArmInit`, `ArmTask`, `ArmResolveHomePose -> ArmInverseKinematics3DOF`, `DMMotorControl`, `HSLServoInit`, `ArmUsbBridgeTask` and the 20-byte `g_arm_servo_angle_debug`.
- MAP removes application `ChassisInit` and `INS_Init`, so the final image remains the arm/tool test image.
- Scoped `git diff --check` passes with only existing LF-to-CRLF notices. Flashing and physical initialization remain external.

## Phase 40 - FruitDetection USB observer and positive-X base calibration

- [completed] Replace the active arm task protocol with the supplied `FruitDetection` wire contract while preserving the USB CDC copy queues, parser recovery and high-priority system replies.
- [completed] Add strict handshake and 3000 ms heartbeat timeout state without coupling standalone arm HOME to host connection state.
- [completed] Add a standalone fruit USB observer bridge with validated latest-result state and concise Watch diagnostics; it must never submit an arm or gripper command.
- [completed] Remove the legacy arm USB bridge from the active Keil target and 1 ms schedule while preserving its source files in the dirty worktree.
- [completed] Keep the complete arm/Damiao/Huaner initialization and HOME `[0,90,-60]`, with chassis and INS disabled.
- [completed] Add Chinese calibration comments that define physical positive X through the Damiao base saved zero; firmware must not send a zero command or apply a 180-degree software offset.
- [completed] Run host protocol tests, strict source checks, Keil ARMCC rebuild, MAP audit and scoped `git diff --check`.

### Phase 40 locked decisions

- USB CDC remains the host transport; the generated document's 115200 baud field does not configure USB.
- The only active wire packets are Ack `0xFD`, Heartbeat `0xFE`, Handshake `0xFF` and non-reliable FruitDetection `0x10` with hash `0x923FFDD9`.
- Fruit results are observation-only in this image. Ripe and unripe packets update Watch state but cannot cause arm motion.
- The base is physically aimed toward the requested positive-X side and saved as Damiao zero using the vendor host tool before the motion test.
- A heartbeat timeout invalidates host data but does not open the gripper, return HOME or cancel the standalone boot/HOME sequence.

### Phase 40 errors

- 2026-08-11: the first MinGW host-test compile failed before source compilation because `TEMP/TMP` resolved to protected `C:\WINDOWS`. The retry uses an explicit project-local temporary directory.
- 2026-08-11: the MSYS2 GCC retry still exited without a useful source diagnostic after using the project-local temporary directory. The independent `C:\w64devkit\bin\gcc.exe` toolchain compiled and ran the same host harness successfully with warnings treated as errors.
- 2026-08-11: the first complete `git diff --check` found trailing whitespace only in Keil-generated `Engineer.build_log.htm` and duplicate-CR line endings in `Engineer_Engineer.dep`. Mechanical line-ending/trailing-space normalization of those two text artifacts resolved the errors without changing source or image content.

### Phase 40 verification

- ARM GCC strict syntax checks passed for `protocol.c` and `fruit_usb_bridge.c` with `-Wall -Wextra -Wshadow -Wconversion -Werror`.
- The host protocol harness passed handshake gating and mismatch, FruitDetection no-ACK and duplicate-refresh behavior, no-target and invalid-value handling, length/CRC rejection, heartbeat echo and exact 3000 ms expiry checks.
- Kinematics verification gives HOME FK `(225.166580,0.000000,192.000000) mm`; IK returns `(0.000000,90.000008,-60.000000) deg` and satisfies the configured limits.
- Keil ARMCC 5.06u7 full rebuild generated current AXF/HEX/MAP with `0 errors / 0 warnings`; totals are Code 84080, RO-data 764, RW-data 972 and ZI-data 106384 bytes.
- MAP retains `FruitUsbBridgeInit`, `FruitUsbBridgeTask`, `on_receive_FruitDetection`, `g_fruit_usb_debug`, `ArmInit`, `ArmTask`, `DMMotorControl`, `HSLServoInit` and `protocol_tick`; `ArmUsbBridgeTask` is absent from the active image.
- MAP removes application `ChassisInit` and `INS_Init`. Flash load is `0x14F38 / 0x100000`, SRAM1 is `0x191B0 / 0x1C000`, and SRAM2 is `0x11AC / 0x4000`; no region overflows.
- Complete `git diff --check` passes; its remaining output is Git's LF-to-CRLF conversion warning only. All Phase 40 temporary test sources, executables, logs and temporary directories were removed.
- Physical base-zero calibration, flashing, automatic HOME movement and USB host/hardware validation remain external.

## Phase 41 - Diagnose Huaner feedback gate blocking Damiao enable

- [completed] Trace the live `ArmInit -> ArmToolInit -> HSLServoInit` return path and identify the Watch fields that distinguish UART registration failure from missing or invalid feedback.
- [completed] Verify USART6 baud, DMA channels/IRQs, controller-board query frame and expected two-servo reply layout against the active source, CubeMX output and ESP32 reference.
- [completed] Change the boot-state ownership so the three Damiao motors enable and hold current position before waiting for both tool-servo feedbacks; automatic HOME remains tool-gated.
- [completed] Rebuild with Keil, audit the MAP and run `git diff --check` without changing chassis/INS or the new FruitDetection protocol.
- [pending] Flash the new image and capture the concise `g_hsl_servo_debug` fields needed to classify the remaining hardware feedback failure.

### Phase 41 hardware observation

- 2026-08-11: after flashing the Phase 40 image, both tool servos appear offline in Watch and the three Damiao motors are not enabled. This is consistent with the current tool-first boot gate but the exact USART6/feedback failure layer is not yet identified.

### Phase 41 verification

- The new boot entry state is `ARM_BOOT_WAIT_MOTORS`; `ArmProcessEnableOnly()` enables and synchronizes all three Damiao motors before the state machine waits for tool feedback. While waiting, the motors retain their measured-position targets and automatic HOME is not started.
- The ESP32 reference confirms the same two-ID request `55 55 05 15 02 01 02` and an 11-byte response for two IDs. USART6 remains 9600 8N1 with RX DMA2 Stream1 and TX DMA2 Stream6.
- Keil ARMCC 5.06u7 full rebuild passed with `0 errors / 0 warnings`; image totals are Code 84104, RO-data 764, RW-data 972 and ZI-data 106384 bytes.
- MAP retains `ArmProcessBootSequence -> ArmProcessEnableOnly`, `DMMotorControl`, `HSLServoInit`, `USARTRegisterAsyncCallbacks`, `HSLServoTask` and `FruitUsbBridgeTask`; chassis and INS application entry points remain removed.
- Complete `git diff --check` passes with line-ending conversion warnings only. Hardware flashing and the actual USART6 failure classification remain pending.

## Phase 42 - Restore full arm runtime and complete controller-board servo communication

- [completed] Inspect all six pages of the supplied controller-board protocol and compare every arm-used command/reply with the live USART6 driver.
- [completed] Disable the dual-servo feedback-only selector and restore the complete arm/Damiao/Huaner runtime while keeping chassis and INS disabled.
- [completed] Complete the controller-board transport for exact frame sizing, two-servo position polling, board-voltage polling, parser resynchronization, timeout recovery and per-servo freshness without adding unrelated action-group behavior.
- [completed] Update deterministic driver self-tests and concise Chinese interface comments for the protocol-defined frames and health semantics.
- [completed] Run strict source checks, Keil ARMCC full rebuild, MAP/runtime audit and scoped `git diff --check`; flashing and physical HOME/gripper motion remain external.

### Phase 42 locked decisions

- USART6 remains controller-board serial communication at `9600 8N1`; the PDF is the wire-level source of truth.
- ID1 and ID2 must remain independently observable even when one servo is absent; normal full-arm polling may aggregate requests only if partial replies are handled safely.
- The active gripper calibration remains default/open `550`, close `630`, with the software range `550..630`.
- FruitDetection remains observation-only. Chassis, M3508 and INS/IMU initialization remain disabled.

### Phase 42 errors

- 2026-08-11: PowerShell startup again failed with host error `8009001d`; all further inspection and build steps use native `cmd.exe` and project-local Keil temporary files.
- 2026-08-11: the first PDF-render attempt used the MSYS Python installation, which does not include PyMuPDF. The existing Scoop Python installation rendered all six pages successfully; no dependency was installed.
- 2026-08-11: the first host GCC `-Werror` check stopped in CMSIS because a 32-bit Cortex-M vector address is cast to a 64-bit host pointer. The retry exempts only `-Wint-to-pointer-cast`; all project-source warnings remain errors and the servo module passes.
- 2026-08-11: text-based PDF/build intermediates were removed, but the execution policy rejected deletion of binary scratch PNG/stream files under untracked `tmp/pdfs`; they are outside all Keil/project references and do not enter the firmware.

### Phase 42 verification

- Strict host syntax checking passes for `hsl_servo.c` with project warnings treated as errors, apart from the documented host-width CMSIS vector cast.
- Keil ARMCC 5.06u7 full rebuild passes with `0 errors / 0 warnings`: Code 84400, RO-data 796, RW-data 972 and ZI-data 106384 bytes.
- MAP retains `ArmInit`, `ArmTask`, `DMMotorControl`, `HSLServoBuildMoveFrame`, `HSLServoServicePolling`, `FruitUsbBridgeTask` and `g_arm_servo_angle_debug`.
- MAP explicitly removes `ChassisInit` and `INS_Init`, so the final image is complete-arm plus observation-only FruitDetection, without chassis or IMU runtime.
- Hardware flashing, automatic HOME movement, ID2 default/open 550, close 630 and physical stall behavior remain external tests.
