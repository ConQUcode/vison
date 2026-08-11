# Progress

## 2026-08-10 - Phase 31 started

- User approved execution of the Huaner feedback-closure plan after manually changing CubeMX to asynchronous 115200 baud.
- Verified the generated RX/TX DMA handles, stream IRQ handlers and USART6 GPIO configuration.
- Scoped the first runnable image to feedback-only polling of IDs 1 and 2. Arm and chassis motion remain disabled; no automatic servo movement will be introduced.
- Replaced the active mixed protocol with controller-board `0x03` move and `0x15` multi-position request/reply framing. Added RX-to-idle plus TX DMA ownership, strict dual-ID parsing, measured velocity, freshness, arrival and timeout supervision.
- Added a feedback-only test mode. It initializes and polls IDs 1/2 at 50 Hz, disables chassis/arm motor service, and requires both `motion_unlocked=1` and a new sticky `request_seq` before moving a servo.
- The first GCC syntax wrapper failed before compilation because `cmd.exe` interpreted the quoted executable path literally. Retrying the same compiler through its space-free absolute path succeeded with `-Wall -Wextra -Wshadow` and no diagnostics.

## 2026-08-01 - Phase 28 completed

- Compared the desktop auto-generated `protocol.h/.c/PROTOCOL_DOC.md` against the live firmware protocol. Packet IDs, payload fields, sizes, CRC, ACK sequence, heartbeat echo and retry timing are unchanged; the wire hash changed to `0x8845D84A` and `Status` gained `STOP=3`.

## 2026-08-02 - Phase 29 started

- User approved the complete ID1-pitch/ID2-gripper and CAN2 two-M3508 differential chassis implementation plan.
- Confirmed the source baseline is clean apart from user-owned VS Code browse database files. The three-Damiao arm, 62/260/260 mm geometry and shoulder-elbow belt coupling remain in scope as preserved behavior.
- Selected native Windows command/build tooling because PowerShell startup fails on this host. No source file has been changed yet.
- Preserved the firmware's existing USB copy queue, high-priority ACK path, reliable queue rotation, timeout recovery and debug extensions instead of overwriting them with the smaller generated transport implementation.
- Implemented TaskStatus STOP as an idempotent business state machine: cancel any active motion, preserve the magnet while returning to fixed HOME, turn the magnet off only after HOME completes, clear task state and reliably enqueue `CallbackStatus{task_id, STOP}`.
- STOP HOME failure retains the magnet state and returns `FAULT_RETRY`; repeated STOP after the safe state does not repeat motion and reissues the STOP callback.
- Updated protocol documentation for the new hash, STOP semantics and the current X-adaptive Z behavior. ARM GCC syntax checks passed for `protocol.c` and `arm_usb_bridge.c`; scoped whitespace checking passed with line-ending notices only. No Keil build, flash or hardware test was run.

## 2026-08-01 - Phase 27 completed

- Replaced fixed USB `33 mm / 21 mm` target heights with one clamped linear X calibration: at X=240 mm the default/magnet heights are 32/20 mm, and at X=450 mm they are 35/23 mm.
- Normal TargetControl, loaded-motion lift/final movement, Task 5/6 descent and Task 5/6 return now use the same centralized mapping functions. Compound magnet actions retain their accepted X for both descent and return.
- HOME remains at its fixed configured XYZ and is not affected by the X-height calibration.
- ARM GCC syntax checking passed for `arm_usb_bridge.c` with `-Wall -Wextra -Wshadow`. Scoped whitespace checking passed with line-ending notices only. No Keil build, flash or hardware test was run.

## 2026-08-01 - Phase 26 completed

- Reviewed `debug(1).md`: three long radial moves stopped about 7.5-8.1 mm short, timed out, then completed after the same target was resent. HOME showed the same first-attempt failure/retry-success pattern, while lateral Y moves and magnet tasks generally completed.
- Relaxed actual-arrival thresholds to `2 deg` position error and `5 deg/s` logical speed while retaining the 120 ms continuous stability window and 2000 ms diagnostic timeout.
- Kept formal MOVE speed at `700 mm/s` and reduced Cartesian acceleration from `7200` to `5000 mm/s2`.
- Settling timeout now reports failure and immediately releases the USB business state without submitting cancel; the Damiao motors retain the original final target. Non-timeout faults still use the existing cancel/hold safety path.
- ARM GCC syntax checks passed with `-Wall -Wextra -Wshadow` for `arm_usb_bridge.c`, `arm_trajectory.c` and `arm.c`. No Keil build, flash or hardware test was run.

## 2026-08-01 - Phase 25 started

- User approved implementing both speed tiers in one pass and adding actual-arrival confirmation before upper-computer completion reporting.
- Rechecked the live trajectory, host lifecycle and USB bridge call paths. No source behavior has been changed yet in this phase.

## 2026-08-01 - Phase 25 completed

- Raised formal USB/default linear motion to `700 mm/s`, Cartesian acceleration to `7200 mm/s2`, and joint acceleration limits to `3000/2500/3000 deg/s2`; joint speed limits, HOME, magnet Z motion, boot motion and all servo settings were preserved.
- Added feedback-confirmed settling for linear, staged and direct joint/Cartesian commands. The host remains BUSY/RUNNING until three-axis feedback is within `1 deg` and `2 deg/s` for `120 ms`.
- Added a `2000 ms` settling timeout and propagated it through the arm command result into USB `FAILED/TIMEOUT`, including compound HOME/magnet failure mapping.
- Added settling Watch diagnostics and reran ARM GCC syntax-only checking for the changed execution path plus `Test.c`; all passed. Scoped whitespace checking passed with only LF/CRLF notices. Keil, flash and physical testing were intentionally not run.

## 2026-08-01 - Phase 24 completed

- Moved the single existing `ArmToolTask(now_ms)` call from the beginning of `ArmTask()` to the common finish path after ID1 and ID2 compensation targets are calculated.
- Normal motion now presents both current-cycle targets to the latest-target scheduler before it selects a dual-servo or single-servo frame.
- Verified that early fault/temperature branches converge on the same finish label, so asynchronous USART6 servicing is not skipped.
- Preserved all established servo configuration and tracking parameters. ARM GCC syntax checks and scoped whitespace checks passed; no Keil build, flash or hardware test was run.

## 2026-08-01 - Phase 23 completed

- Changed only `ARM_TOOL_SERVO2_TRACK_UPDATE_PERIOD_MS` from 30 ms to 20 ms, matching the existing ID1 compensation cadence.
- The expected result is faster ID2 base/yaw response and more opportunities for ID1/ID2 to use one `HSLServoMove2()` frame.
- Kept the verified asynchronous USART6 transport, latest-target overwrite scheduler, mappings, compensation formulas, deadbands and motion-time settings unchanged.
- ARM GCC syntax-only checking passed and scoped whitespace checking passed with only the existing LF/CRLF notice. No Keil build, firmware flash or hardware test was run.

## 2026-08-01 - Phase 22 completed

- Added overwriteable latest-target slots for ID1/ID2 in `arm_tool`, including Watch-visible pending positions, overwrite counts and single/dual-frame counters.
- Added automatic `HSLServoMove2()` dispatch when both targets share one motion time, with fair single-frame fallback for one target or mismatched times.
- Routed ID2 repeat sends and direct position requests through the same scheduler, and kept the dynamic ID1/ID2 compensation parameters unchanged.
- Added pending-command cleanup for stop/fault paths and a TX-idle boot gate so asynchronous tool frames cannot leak across a stop or be mistaken for completed initialization.
- ARM GCC syntax-only checks passed for `arm_tool.c`, `arm.c` and `hsl_servo.c`. Scoped whitespace checking passed with only LF/CRLF notices. No Keil build, firmware flash or hardware test was run.

## 2026-08-01 - Phase 21 completed

- Converted the USART6 controller-board move path to interrupt-driven non-blocking transmission without changing CubeMX configuration.
- Added shared BSP callback registration, a non-blocking 2 ms post-TX gap state, finite 50 ms TX timeout and compatible status handling for the existing two-servo frame API.
- Fixed the generic transaction queue to retain its `expects_response` argument so the position-read path remains logically intact.
- ARM GCC syntax-only check passed with no diagnostics; scoped `git diff --check` passed with only existing LF/CRLF notices. No Keil build or hardware test was run.

## 2026-08-01 - Phase 20 started

- Compared the host-side `debug.md` report with the live protocol, USB queue and arm bridge code.
- Confirmed two independent recovery gaps: rejected arm commands can retain the bridge business lock, while a stuck/full USB TX queue can prevent even `ACK id=4` from leaving the MCU.
- Scoped the implementation to bridge-state recovery and USB transport recovery only; arm geometry, HOME coordinates, motor limits, speed and hardware parameters remain untouched.
- A combined late-completion and task-record cleanup patch failed context verification without changing source; continuing with smaller patches.
- The first GCC syntax-check wrapper failed at shell parsing (`unexpected (`) before invoking GCC; switching to explicit per-file commands.
- Completed the rejection-state split, recoverable `FAULT_RETRY`, dedicated ACK queue, USB TX timeout/reconnect recovery, late-callback guard and Watch diagnostics.
- Explicit ARM GCC syntax-only check passed for all four changed C translation units.
- Scoped `git diff --check` passed; only existing LF/CRLF conversion notices were emitted. Keil compilation and hardware testing remain manual.

- 2026-07-30: Started Phase 18 after hardware showed the automatic point task remained in `wait_ready` and never submitted. Chose a single-owner `ArmTask()` boot sequence instead of adding more host-ready conditions to `Test.c`.
- 2026-07-30: First combined Phase 18 patch was rejected cleanly due to an unstable mojibake comment anchor. Switched to smaller patches keyed by symbols and function boundaries.

- 2026-07-30: Started the combined arm-interface completion and USART6 Huaner LX driver implementation. Confirmed the CubeMX USART6 DMA/IRQ path is complete, `catch.c` remains out of scope, and the existing SCS legacy calls must be retained without transmitting.
- 2026-07-30: Completed the host-ready arm interface. Added the unified command mailbox, priority ESTOP/cancel/stop-realtime/fault-reset handling, interruption records, compatibility wrappers and an always-refreshed status snapshot. Startup reaches `[0,180,-90]`, then smoothly reaches `[0,90,-90]`, and only then reports READY; all automatic point/simulator defaults are off.
- 2026-07-30: Completed the USART6 Huaner LX rewrite with move/stop/position-read APIs, strict `55 55` validation, one DMA transaction state machine, pre-armed RX DMA for fast reply/echo tolerance, per-ID status and `g_hsl_servo_debug`. Central BSP callback dispatch remains the only HAL callback owner.
- 2026-07-30: All unchanged legacy Feetech APIs are non-transmitting unsupported stubs, so current `catch.c` calls cannot put old 875..1500 parameters onto USART6.
- 2026-07-30: ARM GCC source-only syntax checks passed for `hsl_servo.c`, `bsp_usart.c`, `arm.c`, `arm_trajectory.c` and `Test.c` with `-Wall -Wextra -Wshadow`; scoped `git diff --check` passed with line-ending notices only. No Keil build, flash, logic-analyzer capture or hardware validation was performed.

- 2026-07-30: Began connecting the current automatic initialization and automatic XYZ test point to the existing quintic joint/Cartesian-linear trajectory layer. Scope is source-only with no Keil build, while preserving current live geometry, q3 convention and belt coupling.
- 2026-07-30: Completed trajectory integration. Auto initialization now uses synchronized quintic joint interpolation with a real Watch-tunable joint speed cap; the auto XYZ point now uses full Cartesian-linear preflight and online IK execution. Scoped `git diff --check` passed; no Keil build was run by request.
- 2026-07-30: Hardware reported that initialization no longer moved. Restored the verified simultaneous direct initialization because normal joint-trajectory references reject startup feedback outside soft limits. Kept Cartesian straight-line planning for the post-initialization XYZ move.
- 2026-07-30: Diagnosed the post-initialization no-motion result as a valid Cartesian preflight rejection: the initialization wrist point is behind/below the base and cannot connect directly to the forward target within limits. Added a quintic joint transition to `[0,90,-90]` before the validated Cartesian straight-line segment.
- 2026-07-30: Increased the post-initialization staged trajectory to the fast commissioning tier: 300 mm/s Cartesian command, 1800 mm/s2 Cartesian acceleration, 220-240 deg/s joint limits, and 700-800 deg/s2 joint acceleration. Initialization itself remains at the separately configured 100 deg/s.
- 2026-07-30: Applied the user's all-phase speed increase and denser path request. Initialization is now 240 deg/s, staged joints 380-420 deg/s with 1500-1800 deg/s2, Cartesian motion 450 mm/s with 3600 mm/s2, arrival stability 120 ms, preflight spacing 1 mm, and online IK every 2 ms.
- 2026-07-30: Removed the stop/restart at the forward safe pose by adding one continuous joint-then-Cartesian cached trajectory. The safe pose is now only a waypoint inside one global quintic profile; only the final XYZ target decelerates to zero.
- 2026-07-30: Removed the remaining sample-density speed discontinuity at the safe waypoint. Cached joint samples now carry normalized progress based on each segment's q1/q2/q3 minimum travel time instead of treating every 1 deg and 1 mm sample as equal-duration; duration limiting uses the same nonuniform progress. Source inspection only, with no Keil build per user request.
- 2026-07-30: Added a local cubic Bezier corner blend around the forward safe region (12 staging intervals before and 12 Cartesian intervals after). This changes the former exact sharp waypoint into a tangent-continuous safe-region pass so the shoulder-to-elbow motion transition no longer requires an instantaneous direction change.
- 2026-07-30: Replaced the post-initialization one-point test with a continuous four-point upper-computer simulation: `(250,50,120) -> (250,50,150) -> (250,-50,150) -> (250,-50,120) -> repeat`, with a 3000 ms command interval. The first move uses the rounded composite entry and later moves use Cartesian-linear preflight.
- 2026-07-30: Started the formal host-control application boundary. Added `arm_host.h` as the communication-independent public surface for a unified command envelope and concise status snapshot; motor/CAN/debug internals remain in `arm.h`.

- 2026-07-30: User confirmed all three Damiao motors now enable successfully. Started the fast commissioning phase: retain three-axis enable/hold, add runtime Watch axis selection and one-axis-at-a-time `2 deg` direction tests in a single firmware before enabling NORMAL automatic return.
- 2026-07-30: Completed the fast commissioning firmware. Default boot is now `DM_SINGLE_AXIS_TEST`, but it first executes the physically verified three-axis enable/current-position hold sequence. Watch can select base/shoulder/elbow at runtime without reflashing; selection alone never moves a motor.
- 2026-07-30: Added sticky test diagnostics: applied axis, start logical angle, target logical angle and fixed test speed. Requests remain limited to `2 deg` initially, `5 deg` maximum and `5 deg/s`, with normal software-limit preflight.
- 2026-07-30: Added a no-stacking guard: another Watch request is acknowledged but returns `BUSY` until all axes are within `1 deg` of their hold target and below `2 deg/s`.
- 2026-07-30: Final Keil ARMCC build `dm_arm_fast_commission_build.txt` passed with Code 46744, RO 672, RW 916, ZI 101872 bytes, 0 errors and 0 warnings. Map retains the three-axis enable/hold and single-axis test paths and removes NORMAL `ArmProcessStartup`. Current output is `Engineer/MDK-ARM/Engineer/Engineer.hex`; it was not flashed by Codex.

- 2026-07-30: Began the explicit three-axis enable-gate correction after hardware showed only the elbow with apparent holding force. The intended NORMAL order is now: request Enter Mode on all axes, wait for fresh state-1 feedback from all three, synchronize current targets, then validate limits/escape/return.
- 2026-07-30: Completed the explicit three-axis enable gate. CAN TX success no longer marks a Damiao motor enabled; each axis requires a post-request state-1 feedback frame. Added Watch-visible `mode_request_pending`, `mode_confirmed` and `mode_command_count` per axis.
- 2026-07-30: Rebuilt the final NORMAL image: Code 55300, RO 676, RW 916, ZI 101844 bytes, 0 errors and 11 warnings. Two warnings are NORMAL-unused bench helpers; the remaining nine are existing legacy declaration/newline warnings from `nac.h`, `chassis.h` and `catch.h`. No flash or CAN-analyzer confirmation was performed.
- 2026-07-30: Hardware LEDs confirmed all three Enter Mode commands briefly work, after which firmware returns all motors to red/disabled. Began a dedicated enable-only image that synchronizes startup positions, enables all three and continuously holds those positions without entering any arm motion state machine.
- 2026-07-30: Completed and linked `DM_ENABLE_ONLY`. Final image is Code 44708, RO 672, RW 916, ZI 101844 bytes with 0 errors and 11 existing/expected warnings. The map removes NORMAL startup/return and all linked `DMMotorDisable` use; no flash or physical LED validation was performed by Codex.
- 2026-07-30: Corrected the decisive CAN-ID bug using the user's upper-computer capture: special mode payloads now transmit on raw IDs 3/2/1, while position-speed targets remain on 0x103/0x102/0x101. Rebuilt final enable-only image: Code 44792, RO 672, RW 920, ZI 101856, 0 errors and 11 existing/expected warnings. Added per-axis `last_mode_tx_id`.
- 2026-07-30: Found the corrected-ID image could still send no Enter frames because enable-only startup waited indefinitely for passive feedback from red/disabled motors. Changed the sequence to send and refresh Enter first, then wait for feedback before current-position hold synchronization.
- 2026-07-30: Rebuilt the no-feedback-deadlock enable-only image: Code 44936, RO 672, RW 920, ZI 101856 bytes, 0 errors and 2 expected unused bench-helper warnings. Map retains raw-ID mode send and removes NORMAL startup.

- 2026-07-30: Completed the full-Damiao source refactor. Replaced the arm's GM6020/M3508/M2006 implementation, stall homing, scan calibration, ratio mapping and current/PID/feedforward dependencies with three absolute-position Damiao motor adapters.
- 2026-07-30: Reworked `dmmotor` into feedback-first position-speed APIs, explicit hold/disable/fault operations, validated IDs/models, absolute feedback counters and one unified 2 ms transmit service called from the existing 1 ms task.
- 2026-07-30: Added the safe bench default `DM_SINGLE_AXIS_TEST + ARM_DM_TEST_NONE` and `g_arm_dm_debug`; automatic host simulation and old point replay remain disabled.
- 2026-07-30: Implemented NORMAL startup, approved 20-degree escape corridor, elbow-shoulder-base sequencing, safe-pose return, 70/80 C protection, emergency disable and sticky explicit fault reset.
- 2026-07-30: Updated static geometry/limits and removed the trajectory hard-stop exit corridor and old automatic three-point cycle.
- 2026-07-30: ARM Cortex-M4 GCC syntax checks passed for dmmotor, arm, kinematics, trajectory, wrist and Test with `-Wall -Wextra -Wshadow`.
- 2026-07-30: Host numeric test produced safe FK `(260,0,510) mm`; five FK/IK/FK round trips passed with worst error `6.10351562e-05 mm`.
- 2026-07-30: First host numeric build failed because GCC attempted to create temp files under `C:\WINDOWS`; project-local `TEMP/TMP` fixed the environment issue.
- 2026-07-30: Final serialized Keil ARMCC 5.06u7 build passed: Code 46224, RO 672, RW 916, ZI 101820 bytes, 0 errors and 0 warnings.
- 2026-07-30: Temporarily selected NORMAL and completed a full ARMCC link of the escape/return/fault path: Code 55116, RO 676, RW 916, ZI 101820, 0 errors and 2 expected unused bench-helper warnings. Restored `DM_SINGLE_AXIS_TEST + ARM_DM_TEST_NONE` and rebuilt the final safe image with 0 errors/0 warnings.
- 2026-07-30: No flash, physical motion or CAN-analyzer validation was performed. Existing `.vscode` and Keil user-option dirty files were preserved.

- 2026-07-30: User approved implementation of the decision-complete full-Damiao plan. Confirmed q1/q2/q3 mappings and limits, 20-degree escape bounds, safe pose `[0,90,-90]`, elbow-shoulder-base sequencing, explicit sticky fault reset, CAN1/1 Mbps, and unified 1 ms DM service.
- 2026-07-30: Rechecked the live worktree; preserved user/IDE changes in `.vscode/BROWSE.VC.DB`, `Engineer.uvguix.11737`, and `Engineer.uvoptx`. Confirmed trajectory code depends on a small internal arm interface, allowing the legacy DJI-heavy `arm.c` implementation to be replaced without changing the public trajectory framework.

- 2026-07-30: Began the full-Damiao arm refactor audit in analysis-only mode. Locked the proposed base DM4310 ID3/Master 0x13, shoulder DM4340 ID2/Master 0x12, elbow DM4310 ID1/Master 0x11, all in speed-position mode with absolute encoders and no stall homing.
- 2026-07-30: Restored the prior GM6020/M3508/M2006 FK/IK and trajectory context; selected embedded project routing, driver review, CAN protocol review, and Windows environment-guard workflows. PowerShell failed with 8009001d, so inspection continues through cmd.exe.
- 2026-07-30: Located the active project under `Engineer/` and the relevant `APPLICATION/arm`, `MODULE/motor/DMmotor`, `DJImotor`, and shared motor-task paths. A cmd `findstr` pattern attempt failed and was replaced with direct `rg` filtering.
- 2026-07-30: Confirmed the DM wrapper implements position-speed frames (`0x100 + motor ID`, float radians + rad/s) but found feedback-unit, direction, stop, task-lifetime, shared-counter, and task-name hazards that require module-layer cleanup before the arm can safely depend on it.
- 2026-07-30: Verified the live firmware has no DM runtime path: `ArmInit()` runs before the RTOS scheduler, no `DMMotorControlInit()` call exists, and the linker removes the DM APIs. Defined a feedback-first, current-position-seeded startup sequence for the plan.
- 2026-07-30: Completed the analysis-only source audit and wrote a seven-phase full-Damiao refactor plan covering module hardening, one-axis bench bring-up, arm motor adaptation, removal of stall homing, new 250/260/260 mm kinematics, trajectory reconnection, and build/hardware acceptance. Runtime source remains unchanged pending joint-convention confirmations.

- 2026-07-27: Started the complete 3DOF FK/IK and Cartesian linear-motion implementation from the confirmed `(-29,-7.6,34) mm`, `L1=150 mm`, `L2=179 mm` model.
- 2026-07-27: Confirmed the existing motor tuning and homing code will remain unchanged; identified repeated PID clearing in `ArmSetJointTargetDeg()` as the main control-interface issue to split.
- 2026-07-27: Added the pure `arm_kinematics` module with the confirmed offset geometry, reference-pose self-test, soft-limit check, conservative automatic region, four-branch analytic IK, continuity scoring, and FK round-trip error.
- 2026-07-27: Added `arm_trajectory` with the three-stage hard-limit exit corridor, quintic joint staging, 2 mm Cartesian preflight, 5 ms online IK, 1 ms reference interpolation, settling, hold, continuous P1-P2-P3 cycling, and fault shutdown.
- 2026-07-27: Split movement startup from continuous reference updates so PID state is cleared only once at the beginning of a new move.
- 2026-07-27: Prevented the legacy READY hold path from writing references or clearing the base controller while the trajectory layer owns the three motors.
- 2026-07-27: Added runtime timeout coverage for staging/running, fixed the final zero-duration IK-segment edge case, and populated staging Cartesian debug targets.
- 2026-07-27: Added both new source files explicitly to `Engineer.uvprojx`; a full Keil Clean Build passed with 0 errors. It reports 49 pre-existing warnings in unrelated legacy modules; the three arm source files add no warnings. The final incremental build is code 59892 bytes with 0 errors and 0 warnings.
- 2026-07-27: Added MATLAB closed-loop path validation. Independent parity validation passed 53 samples with maximum FK error `1.10e-13 mm` and maximum joint step `[3.066,1.034,0.570] deg`.

- 2026-07-24: Started the temporary stall-homing implementation.
- 2026-07-24: Selected embedded project routing and driver-review workflows; classified the critical layer as DJI motor feedback/control semantics.
- 2026-07-24: Reworked the arm application into independent M3508/M2006 stall-homing state machines; GM6020 remains registered and stopped, and PWM startup is inactive.
- 2026-07-24: Reset PID runtime and timing state before each homing attempt to prevent stale integral or delta-time effects.
- 2026-07-24: Added `g_arm_homing_abort` as a persistent Watch emergency stop; an explicit restart command is still required afterward.
- 2026-07-24: Made abort discard any pending restart request so clearing abort cannot resume motion by itself.
- 2026-07-24: Corrected abort recovery by adding a terminal `ARM_HOMING_ABORTED` state; only `g_arm_homing_restart=1` leaves it.
- 2026-07-24: `arm.c` passed GCC syntax checking with `-Wall -Wextra -Wshadow -Wconversion` and no warnings; scoped `git diff --check` passed.
- 2026-07-24: Confirmed the Keil project still contains the arm source entry and `APPLICATION/arm` include path. MDK CLI build was unavailable because Keil tools are not on PATH.
- 2026-07-24: Moved the registered-but-disabled GM6020 ID1 from CAN2 to CAN1; no CubeMX changes were required.
- 2026-07-24: Made `DJIMotorReset()` explicitly clear `measure.total_angle` after updating the software zero offset.
- 2026-07-24: Added independent shoulder/elbow homing entry points and per-motor Watch diagnostics for stall condition, completion, zero execution, trigger measurements, and reset count.
- 2026-07-24: Split M3508 and M2006 homing speed/current settings into four independently named Watch globals.
- 2026-07-24: Disabled automatic homing timeout by default after identifying it as the likely source of stop-without-zero behavior; timeout remains opt-in with a nonzero value.
- 2026-07-24: Added per-motor peak absolute current telemetry for homing threshold tuning.
- 2026-07-24: Started replacing the temporary one-sided homing flow with sequential two-ended calibration, teaching, and non-driving 3DOF kinematics.
- 2026-07-24: Replaced the old auto-start homing logic with explicit sequential two-ended shoulder/elbow calibration, RAM teaching, GM6020 front teaching, wrist-center FK, and non-driving IK.
- 2026-07-24: Tightened maximum-end travel protection to measure from the minimum-end software zero and added runtime guards for invalid Watch-tuned ratios, speeds, currents, margins, and IK inputs.
- 2026-07-24: Made a new calibration attempt clear stale joint limits, spans, directions, and teaching offsets while preserving an already taught GM6020 front zero.
- 2026-07-24: Confirmed `arm.c` and `Test.c` pass ARM GCC syntax checking with `-Wall -Wextra -Wshadow -Wconversion`; scoped `git diff --check` passes apart from an existing line-ending notice in `DJI_motor.c`.
- 2026-07-24: Completed a Keil ARMCC 5.06u7 build with project-local `TEMP/TMP`: code 48340 bytes, RO data 1492 bytes, RW data 1020 bytes, ZI data 96068 bytes, 0 errors and 0 warnings.
- 2026-07-24: Verified FK reference poses and four-branch IK round trips independently; the general-pose reconstruction error was below `1e-12 mm` before joint-limit filtering.
- 2026-07-24: Confirmed the MATLAB source uses `baseHeight=80 mm`, `L1=150 mm`, and `L2=179 mm`; firmware intentionally stops at the wrist center and excludes MATLAB's `L3/q4` tool-tip extension.
- 2026-07-24: Replaced the Watch-triggered start with one-shot power-on automatic calibration after M3508/M2006 feedback remains online for 500 ms.
- 2026-07-24: Removed Watch request globals for calibration start, pose teaching, GM6020 teaching, and IK solving; retained only public state/result structures and optional `g_arm_homing_abort` emergency stop.
- 2026-07-24: Verified an abort or calibration failure cannot resume automatically after clearing the abort; another attempt requires reset or an explicit API call.
- 2026-07-24: Rebuilt with Keil after the automatic-start simplification: code 45072 bytes, RO data 1492 bytes, RW data 928 bytes, ZI data 96040 bytes, 0 errors and 0 warnings.
- 2026-07-24: Reordered automatic calibration so M2006 completes both endpoints first, then M3508 completes both endpoints, before entering `ARM_CAL_WAIT_KNOWN_POSE`.
- 2026-07-24: Reversed only the M2006 calibration direction: its first-end scan now uses negative speed, while release and second-end scan use positive speed; M3508 direction is unchanged.
- 2026-07-24: Reversed the M2006 calibration direction again at bench request: its first-end scan now uses positive speed, while release and second-end scan use negative speed; M3508 remains unchanged.
- 2026-07-25: Split boot behavior into fast `ArmHomingStart()` single-reference homing and documented `ArmCalibrationStart()` maintenance full scanning, sharing one protected state machine.
- 2026-07-25: Added a mandatory 2000 ms zero-torque settle state after each confirmed reference-stop stall before clearing `measure.total_angle`.
- 2026-07-25: Compiled measured endpoints q2 `-9/188 deg`, q3 `132/-101 deg`, and measured motor spans `-5493.33936/-8517.4375 deg`; full scans recalculate RAM scale values from current spans.
- 2026-07-25: Final GCC strict syntax check and Keil ARMCC build passed; image size code 45784 bytes, RO data 1492 bytes, RW data 928 bytes, ZI data 96048 bytes, 0 errors and 0 warnings.
- 2026-07-25: Added a temporary isolated M3508 ratio test. It bypasses normal homing, keeps GM6020/M2006 disabled, settles and zeros M3508, then commands 1710 motor degrees at 90 motor-deg/s to test the 19:1 hypothesis.
- 2026-07-25: Added ratio-test online, reverse-motion, overrun, 30 s timeout, and 200 ms sustained-stall shutdowns plus Watch-visible state/angle fields. Keil build passed with code 46636 bytes and 0 errors/0 warnings.
- 2026-07-25: Increased the temporary M3508 ratio-test command from 90 to 342 motor-deg/s, giving a nominal 1710-degree test duration of 5.0 seconds and an assumed 19:1 output speed of 18 deg/s.
- 2026-07-30: Completed Phase 18 large application-layer refactor. Replaced the cross-task `Test.c -> Host mailbox -> ArmTask` startup test with a single-owner `ArmTask()` boot state machine covering three-motor enable, simultaneous initialization, safe-pose entry, tool initialization, stabilization, internal TOOL_TIP trajectory and final READY.
- 2026-07-30: Removed the obsolete API point test task/debug data and old automatic-point symbols. Added terminal handling for tool-init error/timeout and boot-test failure, with current-pose hold and no automatic retry.
- 2026-07-30: Scoped source and whitespace inspection passed; only CRLF conversion notices were emitted. Per user request, no Keil compile, firmware flash or hardware motion test was run.
- 2026-07-30: Corrected the ID1 end-tool vertical compensation direction by restoring `ARM_TOOL_SERVO1_DIRECTION=+1.0f`. Main-arm motor directions, belt coupling and kinematics were left unchanged; no Keil build or hardware test was run.
- 2026-07-31: Started Phase 19 USB host protocol integration. Confirmed live code still has the old `CmdVel` protocol, USB CDC RX ring buffer is present but `USB_ProcessTask()` is commented out, `arm_host.h` lacks ID2 yaw fields, and `ARM_BOOT_TOOL_TEST_ENABLE` is still enabled. Existing user changes are `.vscode/BROWSE.VC.DB` and `Engineer/APPLICATION/arm/arm_config.h`.
- 2026-07-31: Completed Phase 19 source implementation. Replaced the protocol module, added USB copy TX queue and CDC TX completion handling, added `arm_usb_bridge` compound action state machine, added ID2 yaw fields to arm host commands, enabled `USB_ProcessTask/protocol_tick/ArmUsbBridgeTask/BuzzerTask`, added a reusable non-blocking TIM4 buzzer module, and disabled the power-on tool test. `git diff --check` passed with CRLF conversion notices only. No Keil build, flash, USB capture, or hardware validation was run.
## 2026-08-02 - Differential chassis 1 m test implemented

- User narrowed the active goal to chassis-only bring-up; the incomplete arm-tool refactor was fully restored before continuing.
- Replaced the inactive legacy swerve application with CAN2 M3508 ID1/ID2 differential-drive test control.
- Added encoder distance odometry, continuous IMU yaw heading hold, staged auto-start, acceleration limiting, distance deceleration, and latched motor/IMU/timeout safety stops.
- Disabled ArmInit, ArmTask, DMMotorControl, and the arm USB bridge task in the active test schedule.
- Restored 1 kHz INS_Task execution and initialized INS before the scheduler.
- Keil ARMCC 5.06 directly recompiled every changed C file, then the generated Engineer.lnp/scatter configuration was linked with armlink: 0 errors, 0 warnings; Code=67044, RO=2984, RW=1340, ZI=98124.

## 2026-08-11 - Huaner closed-loop feedback bench image completed

- Replaced the mixed direct-servo feedback parser with controller-board `0x15` dual-ID request/reply handling and a strict on-boot protocol self-test.
- Added RX-to-idle DMA, TX DMA, one-owner transaction state, response/TX timeouts, UART recovery, 20 ms polling and per-servo target/feedback/velocity/arrival/offline diagnostics.
- Fixed the shared TX-buffer race by reserving the transaction before copying any caller frame into the DMA-owned buffer.
- Added the feedback-only Watch harness. Arm and chassis motor control remain disabled; `motion_unlocked` defaults to zero and no automatic move is issued.
- Integrated feedback health into `arm_tool` diagnostics for later use without calling the mechanical-arm runtime in this image.
- ARM GCC strict syntax checks, direct ARMCC compilation and scoped `git diff --check` passed for the changed path.
- Final Keil link generated `Engineer.axf`, `Engineer.hex` and `Engineer.map`: Code 66480, RO 2992, RW 896, ZI 99076, 0 errors and 10 legacy catch warnings.
- Hardware reply capture, actual servo position range, flash and physical motion validation remain pending.

## 2026-08-11 - ESP32 controller-board reference alignment in progress

- Confirmed the user-updated CubeMX output already uses USART6 asynchronous 9600 8N1 with separate TX/RX and DMA.
- Kept the existing controller-board `0x03`/`0x15` packet implementation and strict DMA feedback parser rather than replacing it with the less defensive ESP32 parser.
- Added UART configuration validation and Watch-visible baud/expected reply length fields.
- Changed controller-board feedback polling from 20 ms to 50 ms to leave timing margin at 9600 baud while preserving the ID1 0/90 degree five-second sweep.
- The first UV4 invocation failed before compilation because it could not create temporary files; its zero-error footer is invalid because the same log says `Target not created`. A project-local TEMP/TMP retry is required.
- Rebuilt with project-local TEMP/TMP. Final `Engineer.axf/.hex/.map` were generated with 0 errors; 10 existing warnings come only from `catch.c/catch.h`. Map output retains the UART configuration guard, controller-board polling and ID1 sweep sender.
- Scoped `git diff --check` passed with line-ending notices only. Hardware reply capture, board alarm diagnosis, flashing and physical movement remain external.

## 2026-08-11 - Concise ID1 servo Watch telemetry completed

- Reduced `g_huaner_servo1_debug` from the large protocol/state snapshot to five fields: `initialized`, `online`, `feedback_valid`, `target_angle_deg` and `current_angle_deg`.
- Moved sweep state, next target, five-second timing, move count and last driver result into private `Test.c` runtime state; ID1 motion and feedback polling behavior are unchanged.
- Did not add current, voltage or temperature fields because the verified controller-board `0x15` position reply does not carry those measurements.
- Keil ARMCC build passed with 0 errors and 0 warnings; Code=42100, RO=628, RW=444 and ZI=97604 bytes. Scoped `git diff --check` passed.

## 2026-08-11 - Controller-board voltage telemetry completed

- Added controller command `0x0F` request `55 55 02 0F` and strict reply parsing for `55 55 04 0F VL VH`, interpreted as little-endian millivolts in the 3000-15000 mV range.
- Added a separate board-voltage transaction, 500 ms polling and 1500 ms freshness supervision to the existing USART6 DMA owner. Position polling remains first priority and voltage failures do not alter servo online state.
- Added protocol self-tests for the voltage request, valid 7400 mV reply, wrong command, invalid length, truncation and out-of-range voltage.
- Added only `board_voltage_valid` and `board_voltage_v` to `g_huaner_servo1_debug`; the Watch structure remains compact at 16 bytes.
- Keil produced AXF/HEX/MAP with 0 errors and 10 unchanged legacy catch warnings; Code=42992, RO=636, RW=444 and ZI=97636 bytes. Scoped whitespace checks passed.

## 2026-08-11 - Phase 35 started

- Reopened the live runtime, tool, trajectory, USB bridge and protocol sources instead of relying on the earlier broad refactor plan.
- Confirmed the controller-board transport can support the new feature without CubeMX or frame-format changes.
- Locked ID1 `125/500/875`, ID2 `800/850/950/1000`, ID2-only close/boot stall detection and 10-position relief semantics.
- Preserved all unrelated chassis, CubeMX, IDE database and generated build changes.

## 2026-08-11 - Phase 35 completed

- Replaced ID1 empirical vertical compensation with absolute-pitch control, `125..875` rejection limits and the 30 mm gripper-center coordinate model.
- Added complete path preflight and 20 ms ID1 tracking for joint, direct Cartesian, linear Cartesian and realtime trajectories; unsupported tool yaw is rejected.
- Replaced ID2 world-yaw/magnet behavior with feedback-confirmed `BOOT/READY/OPEN/CLOSE`, close/boot stall classification and one-time 10-position relief.
- Migrated startup READY gating, Task 5, Task 6, STOP, host snapshots and the reliable USB bridge to the new pitch/gripper semantics.
- Updated protocol IDs `0x04..0x07`, raised the payload limit to 32 bytes, set `PROTOCOL_HASH=0x1E7AC5B2` and updated `PROTOCOL_DOC.md`.
- Restored the production arm schedule and left the old five-second ID1 sweep compile-time disabled. Chassis, IMU, Damiao parameters and USART6 CubeMX settings were not changed by this phase.
- Keil ARMCC 5.06u7 full rebuild passed with `0 errors / 0 warnings`: Code 90556, RO-data 764, RW-data 1008 and ZI-data 106588 bytes. AXF, HEX and MAP were regenerated on 2026-08-11.
- Source/MAP audit confirms the new pitch/gripper symbols are linked and old vertical-compensation, world-yaw and magnet business symbols are gone. Final hardware flashing and mechanical validation remain pending.
- Final scoped `git diff --check` passed with line-ending conversion notices only. The temporary `Engineer/MDK-ARM/phase35_build.log` was removed after its result was recorded; formal AXF/HEX/MAP outputs remain in place.

## 2026-08-11 - Phase 36 started

- Routed the STM32F407/FreeRTOS/Keil task to application-schedule inspection plus Keil rebuild/map verification; no UART or CAN wire-format change is required.
- Confirmed the chassis 1 m controller, IMU/encoder odometry and Watch snapshot still exist and only the active `Test.c` runtime selection must be restored.
- Locked the final image to chassis-only mode while retaining the completed arm/tool implementation and public APIs in source.
- Confirmed the intended scheduler ownership: `INS_Init()` before the scheduler, `INS_Task()` plus `ChassisNotifyImuUpdate()` at 1 kHz, and `ChassisTask()` plus `DJIMotorControl()` in the 1 kHz combined-control task.
- Confirmed `RobotCMDInit()` is only the DWT clock initialization required by INS and is safe to retain in the chassis-only image.
- Added Chinese semantic comments to the arm tool API, host command/status fields, tool protocol packets, chassis Watch snapshot and chassis public functions without reordering ABI-visible data.
- Added mutually exclusive `CHASSIS_ONE_METER_TEST_ONLY`, `HUANER_SERVO_ID1_SWEEP_TEST_ONLY` and derived production-arm mode selection; the chassis test is now the default.
- Restored one-time INS/chassis initialization and periodic 1 kHz INS notification plus chassis/DJI control. The active branch contains no arm, Damiao or USART6-tool initialization/service call.
- Scoped `git diff --check` passes with LF-to-CRLF notices only; Keil rebuild and MAP verification are next.
- The first Phase 36 rebuild passed, then final USB/protocol isolation required a second full rebuild. The final Keil ARMCC 5.06u7 image passes with `0 errors / 0 warnings`: Code 64976, RO-data 2984, RW-data 1360 and ZI-data 98568 bytes.
- MAP confirms the chassis/INS/DJI runtime is linked and the arm, Damiao and USART6-tool initialization/control entry points are removed from the final image.
- Flash, SRAM1 and SRAM2 regions all fit. Final scoped whitespace validation passed; flashing and physical 1 m testing were not performed.
- Reopened final verification after confirming `USB_ProcessTask()` directly dispatches protocol callbacks. Gated USB RX/TX parsing and `protocol_tick()` to production-arm mode so the chassis image cannot indirectly submit arm commands.
- Final MAP and region audit confirms the USB parser/arm callbacks are removed, chassis/INS/DJI control is retained, and all flash/SRAM regions fit.

## 2026-08-11 - Phase 37 IMU direction correction

- Hardware observation indicates the straight-line IMU correction direction is reversed. Changed the single centralized `CHASSIS_IMU_YAW_SIGN` from `+1.0f` to `-1.0f` so both continuous yaw and gyro-Z feedback use the corrected logical sign.
- Left/right command signs, encoder feedback signs, differential-drive equations and heading PID gains remain unchanged.
- Final Keil rebuild passed with `0 errors / 0 warnings`; Code 64976, RO-data 2984, RW-data 1360 and ZI-data 98568 bytes. MAP confirms the chassis runtime remains active and arm/Damiao periodic control remains removed.

## 2026-08-11 - Phase 38 arm test mode selected

- Disabled the chassis 1 m test selector and kept the isolated ID1 sweep selector disabled, which activates the existing production arm/USB/tool branch.
- No arm, gripper, chassis, IMU or protocol control parameter was changed; this phase only switches the active application runtime.
- Keil full rebuild passed with `0 errors / 0 warnings`: Code 90556, RO-data 764, RW-data 1008 and ZI-data 106588 bytes.
- MAP confirms the production arm, Damiao, Huaner tool and USB protocol paths are linked, while INS and application-layer chassis control are removed from the final image.

## 2026-08-11 - Phase 39 started

- Confirmed the current production-arm selector initializes and services both Huaner servos, all three Damiao motors and the USB arm bridge.
- Confirmed the requested HOME calculation: `[0,90,-60] deg` maps to ID1 pitch-axis position `(225.1666,0,192.0) mm` with the live geometry and passes the configured joint limits.
- Scoped the implementation to arm configuration, HOME IK resolution, Cartesian endpoint conversion, host/protocol semantics and build artifacts; unrelated dirty files remain untouched.
- PowerShell startup failed with the known host error `8009001d`; all subsequent inspection uses native `cmd.exe`. Several quoted MSYS `sed` expressions were rejected by `cmd`; unquoted address ranges succeeded and no source file was changed by those failed reads.
- Updated the active boot-mode naming, HOME joint/Cartesian constants, coordinate-path helpers, host status semantics and protocol documentation/hash while preserving the combined initialization state machine.
- A host build of the live `arm_kinematics.c` verified FK `[0,90,-60] -> (225.166580,0,192.000000) mm` with `0.000015 mm` error. IK from seed `[0,180,-90]` returned exactly `[0,90,-60]`, and both software-limit and automatic-safe checks returned true.
- The first two host GCC attempts failed before compilation because `TEMP` pointed to protected `C:\WINDOWS`; an explicit user-local TEMP/TMP retry passed. The temporary test source and executable were removed after the successful result.
- Keil ARMCC 5.06u7 full rebuild completed with `0 errors / 0 warnings`; current AXF, HEX, MAP and build log were regenerated at 07:05. Image totals are Code 90776, RO-data 764, RW-data 1016 and ZI-data 106620 bytes.
- MAP confirms the final runtime contains `ArmInit`, `ArmTask`, HOME IK, `DMMotorControl`, `HSLServoInit`, `ArmUsbBridgeTask` and `g_arm_servo_angle_debug`, while application `ChassisInit` and `INS_Init` are removed.

## 2026-08-11 - Phase 40 started

- Reopened the live protocol, scheduler, application selector, arm calibration constants and Keil project membership before editing.
- Locked the implementation to USB CDC, strict handshake/heartbeat, observation-only FruitDetection, full standalone arm HOME, and no chassis/INS runtime.
- Confirmed the base positive-X correction will be performed by resaving the physical base zero with the Damiao host tool; firmware mapping and joint limits remain centered at q1=0.
- PowerShell remains unusable with host error `8009001d`; native `cmd.exe` and Windows Keil tools are used for inspection and build validation.
- Added the new protocol state machine, generic reliable FIFO, strict heartbeat state, observation-only fruit bridge, scheduler switch, Keil project membership and positive-X saved-zero guard.
- ARM GCC strict syntax checks pass for the new protocol and fruit bridge. The first MinGW host-test link did not compile because its temporary directory resolved to protected `C:\WINDOWS`; a project-local TEMP/TMP retry is next.
- The independent `w64devkit` GCC built the host protocol harness with all warnings as errors, and every runtime assertion passed: handshake gating/mismatch, no-ACK FruitDetection, duplicate refresh, no-target/invalid handling, length/CRC rejection, heartbeat echo and exact 3000 ms expiry.

## 2026-08-11 - Phase 40 completed

- Replaced the active legacy arm business protocol with the supplied observation-only FruitDetection contract and hash `0x923FFDD9`, while preserving USB CDC queue ownership, parser resynchronization, high-priority system replies and generic reliable-message infrastructure.
- Added strict handshake and heartbeat session handling plus the standalone `fruit_usb_bridge`; `g_fruit_usb_debug` exposes only connection, latest result and key packet counters. Fruit packets cannot move the arm or gripper.
- Removed `ArmUsbBridgeInit/Task` from the active initialization, schedule and Keil target without deleting the existing dirty bridge sources. The full arm, three Damiao motors and two Huaner servos remain active; chassis and INS remain excluded.
- Preserved HOME `[0,90,-60]` and added the base saved-zero startup guard and Chinese calibration comments. Logical positive X is defined by the physical Damiao zero saved with the vendor host tool, not by an FK sign inversion or 180-degree software offset.
- ARM GCC strict checks and the host protocol harness passed. HOME FK/IK verification returned `(225.166580,0,192.000000) mm` and `(0,90.000008,-60) deg`.
- Keil ARMCC 5.06u7 rebuilt AXF/HEX/MAP with `0 errors / 0 warnings`: Code 84080, RO-data 764, RW-data 972 and ZI-data 106384 bytes. MAP retains Fruit/arm/Damiao/Huaner runtime symbols, excludes `ArmUsbBridgeTask`, and removes application chassis/INS initialization.
- Final flash/SRAM regions fit. After normalizing trailing whitespace in two Keil-generated text artifacts, complete `git diff --check` passes with line-ending conversion warnings only; all Phase 40 temporary files were removed.
- Firmware flashing, physical base-zero saving, automatic HOME motion and hardware USB validation were not performed in this session.

## 2026-08-11 - Phase 41 started

- Hardware reports both Huaner servos offline and no Damiao enable after the Phase 40 image starts.
- The current boot design intentionally waits for successful tool-controller initialization and fresh ID1/ID2 position feedback before entering the three-Damiao enable sequence, so the missing motor enable is a downstream symptom.
- Investigation is scoped to the USART6 controller-board init/DMA/query/reply path and the tool boot gate; chassis, INS, FruitDetection and arm geometry remain unchanged until the exact failure is isolated.
- Changed the full-arm boot order so all three Damiao motors enable and hold their measured startup positions first. Tool polling continues in parallel, but automatic HOME remains blocked until both Huaner servos have fresh valid feedback and complete their boot positions.
- Keil rebuilt the new boot order with `0 errors / 0 warnings`: Code 84104, RO-data 764, RW-data 972 and ZI-data 106384 bytes. MAP confirms both the Damiao hold path and USART6 feedback path are linked, while chassis/INS remain removed.
- Full `git diff --check` passes after normalizing the two Keil-generated text artifacts. The remaining servo fault requires one new hardware run and the selected `g_hsl_servo_debug` counters; no source-side evidence supports callback-registration exhaustion or a dual-ID length mismatch.

## 2026-08-11 - Phase 42 started

- Hardware now confirms both controller-board servos respond correctly in the isolated feedback test.
- The requested runtime target is the complete arm/Damiao/Huaner initialization and HOME sequence, with chassis and INS still disabled.
- The supplied six-page controller-board protocol will be used to harden only the position/move/voltage functions required by the arm; action-group features remain out of scope.
- PowerShell failed with the known `8009001d` startup error, so native `cmd.exe` remains the command shell.
- Rendered and visually inspected all six PDF pages. The arm-used wire formats are controller-board move `0x03`, position read `0x15` and board voltage `0x0F`; no CRC or per-servo current/temperature reply is defined in this document.
- Restored the complete arm selector, retained chassis/INS disablement, unified `0x03` move-frame generation, and changed configured multi-ID feedback polling to independent alternating single-ID `0x15` requests.
- Driver lifecycle review confirms static DMA-buffer ownership and ISR/task separation are sound on STM32F407; no BSP or CubeMX change is required.
- Strict host syntax checking passes for `hsl_servo.c` with `-Wall -Wextra -Wshadow -Werror`; only the explicitly non-fatal 32-bit CMSIS vector-address cast appears under the 64-bit host compiler.

## 2026-08-11 - Phase 42 completed

- Restored the complete arm runtime while keeping chassis and INS disabled. Startup again initializes both Huaner servos, the three Damiao arm motors, standalone HOME and the observation-only FruitDetection bridge.
- Controller-board move, position and voltage formats now follow the supplied PDF through centralized builders/parsers and deterministic self-tests. Configured two-servo polling is split into alternating single-ID requests so one missing reply does not invalidate the other servo.
- Keil ARMCC rebuilt the final AXF/HEX/MAP with `0 errors / 0 warnings`: Code 84400, RO-data 796, RW-data 972 and ZI-data 106384 bytes. MAP retains arm/Damiao/Huaner/Fruit paths and removes chassis/INS entry points.
- Flashing and hardware HOME/gripper validation were not performed. Untracked binary PDF-render scratch files remain under `tmp/pdfs` because deletion is blocked by the current execution policy; they are not referenced by the project.
