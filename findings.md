# Findings

## 2026-08-11 - Controller-board servo protocol alignment

- The ESP32 reference and live STM32 driver use the same controller-board protocol: `55 55 | Length | Command | Count | ...`, move command `0x03`, multi-position-read command `0x15`, no direct-servo checksum.
- The live CubeMX source and `.ioc` already contain `USART6 9600 8N1`, asynchronous PG14 TX/PG9 RX, and RX/TX DMA, so generated peripheral configuration did not need another edit.
- At 9600 baud, a one-servo read request plus reply occupies about 14.6 ms on the wire; a two-servo exchange occupies about 18.8 ms before controller turnaround. The prior 20 ms polling period left little or no margin, so the controller-board default was changed to 50 ms.
- The existing STM32 parser is intentionally stricter than the ESP32 reference: it checks frame length, command, count, IDs, duplicates and the 0..1000 position range while retaining raw TX/RX Watch frames.
- Added startup validation for `9600/8N1/TX_RX/no-flow-control` and exposed baud/config/expected-reply-length values through the existing debug structures.
- Keil ARMCC rebuilt the final image successfully: Code 42300, RO-data 628, RW-data 444 and ZI-data 97732 bytes. The 10 warnings are pre-existing declarations/type warnings in inactive `catch.c/catch.h`; the changed servo path produced no warnings.

## 2026-08-01 - Main-arm speed and completion audit

- Formal USB `TargetControl` motion currently commands `450 mm/s`, while the trajectory layer allows `700 mm/s`; the duration solver is also bounded by `3600 mm/s2` Cartesian acceleration and per-joint velocity/acceleration limits.
- Prepared trajectories currently switch directly from RUNNING to HOLDING when the reference duration expires. Host command completion then sees `!ArmTrajectoryIsBusy() + HOLDING + progress=1` and can report COMPLETED before physical feedback has settled.
- `ARM_MOTION_SETTLING` and arrival thresholds already exist in the codebase, but the prepared Cartesian trajectory path does not currently use them. Reusing that state is the narrowest change and keeps the USB protocol unchanged.
- The approved second speed tier will target `700 mm/s` formal MOVE and `7200 mm/s2` Cartesian acceleration. HOME, magnet vertical motion, startup speed and fixed business delays remain separate and unchanged.
- Implemented the second tier: formal/default linear speed is `700 mm/s`, Cartesian acceleration is `7200 mm/s2`, and q1/q2/q3 acceleration limits are `3000/2500/3000 deg/s2`; the existing `420/380/420 deg/s` joint speed caps remain unchanged.
- Prepared and direct joint/Cartesian commands now enter `ARM_MOTION_SETTLING` at the final reference. Completion requires all three feedback angles within `1 deg`, all three logical joint speeds within `2 deg/s`, continuously for `120 ms`.
- Settling is bounded to `2000 ms`. A timeout becomes `ARM_MOTION_ERROR_TIMEOUT`, the active host command finishes as `ARM_COMMAND_TIMEOUT`, and the USB TargetControl path reports `MotionStatus FAILED/TIMEOUT` before its recovery cancel releases the business state.
- Added Watch fields `arrival_within_tolerance`, `arrival_stable_ms` and `settling_timeout_ms` to the existing `g_arm_control_debug`; existing joint error, speed, trajectory and tracking-error fields remain available.
- ARM GCC syntax-only checks passed for `arm_trajectory.c`, `arm.c`, `arm_usb_bridge.c` and `Test.c`; scoped `git diff --check` passed with line-ending notices only. No Keil build, flash or hardware validation was run.

## 2026-08-01 - Same-cycle ID1/ID2 target generation

- Previously `ArmToolTask()` ran at the start of `ArmTask()`. ID1's latest vertical target was converted into a pending frame there, but the current-cycle ID2 base/yaw target was not calculated until the common finish path, so the scheduler could dispatch ID1 before ID2 reached its slot.
- `ArmToolTask()` now runs once in the common finish path, immediately after current feedback, trajectory work and both compensation target calculations. It first advances the previous USART6 transaction, then converts ID1's current target and dispatches against the ID2 target produced in the same arm cycle.
- All `goto arm_task_finish` paths still reach the tool service. Initialization and fault states therefore continue to advance once per arm cycle; no ISR business logic, extra queue or second tool-task call was introduced.
- The change affects scheduling order only. The 20 ms cadence, latest-target overwrite behavior, frame bytes, UART settings, mappings, directions, compensation formulas, deadbands, repeats and motion times are unchanged.
- ARM GCC syntax checking passed for `arm.c` and `arm_tool.c`; scoped `git diff --check` passed with line-ending notices only. No Keil build, flash or hardware test was run.

## 2026-08-01 - ID1/ID2 50 Hz cadence alignment

- ID1 compensation already generates a latest target every 20 ms, while ID2 base/yaw tracking was limited to 30 ms. The mismatched cadence reduced opportunities for the latest-target scheduler to see both slots together.
- ID2 tracking is now also 20 ms. This raises its maximum tracking refresh from about 33 Hz to 50 Hz and aligns it with ID1 without changing either compensation calculation.
- At 9600 baud, a 13-byte two-servo frame occupies about 13.54 ms on an 8N1 UART. Including the existing 2 ms board gap gives about 15.54 ms, so a 20 ms combined-frame period retains roughly 4.46 ms scheduling margin.
- No deadband, mapping, direction, scale, repeat count, motion time, UART or CubeMX setting changed. ARM GCC syntax checking and scoped `git diff --check` passed; no Keil build or hardware test was run.

## 2026-08-01 - Latest-target servo coalescing

- `arm_tool` now owns one pending slot per servo. When USART6 is busy, a newer target replaces that servo's unsent target, so the bus does not replay obsolete intermediate compensation angles.
- When both slots are valid and use the same `time_ms`, the scheduler emits the existing controller-board two-servo frame through `HSLServoMove2()`; one changed servo or mismatched motion times use the existing single-servo frame.
- Initialization waits for both the relevant pending slot and `g_hsl_servo_debug.busy` to clear. Boot tool stabilization additionally requires `ArmToolTxIdle()`, preventing READY sequencing from outrunning an asynchronous initialization or compensation frame.
- Stopping ID1 tracking clears its unsent ID1 target. Fault/estop paths clear both pending targets and cancel ID2 repeat sends, preventing a stale command from being dispatched after motion has been stopped.
- No USART6, mapping, compensation, update-period, repeat-count or motion-time macro was changed. ARM GCC syntax-only checking passed for `arm_tool.c`, `arm.c` and `hsl_servo.c`; scoped `git diff --check` passed with line-ending notices only. No Keil build, flash or hardware validation was run.

## 2026-08-01 - Non-blocking USART6 servo move TX

- Live CubeMX USART6 has the global USART interrupt but no USART6 TX DMA. At 9600 baud with 10/13-byte controller-board frames, `HAL_UART_Transmit_IT()` removes task blocking without requiring a CubeMX DMA change.
- `HSLServoMove()` and `HSLServoMove2()` now copy complete frames into the driver-owned persistent buffer and return `OK`; while that transaction is active, another command still returns `BUSY`.
- USART6 TX completion is delivered through the existing shared BSP callback dispatcher. The ISR only sets `tx_complete`; `HSLServoTask()` updates counters/status, waits the existing 2 ms board gap non-blockingly, then releases `busy`.
- The old move path's blocking `HAL_UART_Transmit()`, TC polling loop and `HAL_Delay(2)` are gone. USART6 baud, half-duplex mode, frame bytes, servo position mapping and arm compensation parameters are unchanged.
- ARM GCC syntax-only checking with `-Wall -Wextra -Wshadow` and scoped `git diff --check` passed. Keil link, flashing and hardware timing tests were not run.

## 2026-08-01 USB recovery failure analysis

- A rejected Cartesian target is accepted into the arm mailbox first and later finishes as `ARM_COMMAND_STATE_REJECTED`; the USB bridge currently groups that state with `CANCELLED/FAULTED`, submits an unnecessary cancel command, and remains in `ARM_USB_ACTION_FAILED` until `host.busy` becomes zero.
- A later `TargetControl` is rejected at the bridge entry whenever either `action_state` or `active_business` is not idle.
- Protocol ACK generation occurs before `ArmUsbBridgeOnTargetControl()`, so a trace with no `ACK id=4` cannot be explained solely by the arm business lock.
- The USB copy TX queue can remain permanently at `tx_busy=1` if the CDC completion callback is lost or the host reconnects during a transfer. There is no timeout, CDC init/deinit reset hook, or queue-state debug API; restarting only the host therefore does not necessarily recover it.
- ACK, heartbeat, callback and motion-status frames share the same eight-slot FIFO. Low-priority reliable status traffic can fill every slot and prevent a newly received command ACK from being queued.

## 2026-08-01 USB recovery implementation

- `ARM_COMMAND_STATE_REJECTED` now uses an immediate reject path: it sends the appropriate failed status/callback, clears active command/yaw/phase tracking, and restores `active_business/action_state` to idle in the same bridge cycle. It does not submit `CANCEL_MOTION`.
- Synchronous submission failures for target, HOME, magnet descend/raise and ID2 reset use the same immediate reject path because no new physical command started.
- `STATUS_FAULT_RETRY` now clears recoverable bridge state when the arm has no latched fault; if the arm is genuinely busy it submits one cancel request and waits for `host.busy` to clear.
- USB protocol ACK frames now use an independent four-slot high-priority copy queue. Reliable motion/callback traffic remains on the normal eight-slot queue and cannot consume ACK capacity.
- USB TX records start time and recovers after 100 ms without completion by dropping the stale in-flight copy, clearing CDC `TxState`, flushing the CDC IN endpoint and waiting a 20 ms guard period. Pointer matching prevents a late completion callback from completing a newer frame.
- CDC init/deinit clears both application TX queues and requests a protocol-session reset in USB task context, so reconnecting the host clears duplicate-sequence state and stale reliable queues.
- Task IDs 1..4 are recorded without retaining `TASK_START_RECORD` as an active movement-business lock, and each new non-duplicate TargetControl resets status deduplication so it receives its own failure response.
- ARM GCC `-fsyntax-only` passed for `arm_usb_bridge.c`, `usb.c`, `usbd_cdc_if.c` and `Test.c`; scoped `git diff --check` passed with line-ending notices only. No Keil link, flash or hardware test was run.

## 2026-07-31 USB host protocol integration audit

- The active protocol under `Engineer/MODULE/protocol` is still the old `Handshake/Heartbeat/CmdVel` version. The desktop source `C:/Users/11737/Desktop/protocol.h/.c` contains the requested puzzle-arm messages and keeps `Packet_CartesianMotionCommand` at 16 bytes.
- USB CDC RX already follows the correct low-level shape: `CDC_Receive_FS()` calls `USB_RxHandler()`, and `USB_ProcessTask()` feeds `protocol_fsm_feed()` byte by byte. However `Usb_f()` currently comments out `USB_ProcessTask()`, so no host frames are consumed.
- `USB_Transmit()` currently forwards caller buffers directly to `CDC_Transmit_FS()`. Since CDC transmit is asynchronous, protocol stack frames must be copied into a persistent USB TX queue before calling the CDC driver.
- The arm application has `ArmSubmitCommand()` and `ArmGetHostStatus()`, but Cartesian commands currently do not expose `tool_yaw_valid/tool_yaw_deg`; `ARM_TOOL_ACTION_SERVO2_ANGLE` is still rejected in `ArmExecuteToolCommand()`.
- `arm_tool` already owns ID1/ID2 Huaner servo commands and PB12 magnet control. Current live calibration values are retained: ID1 neutral pos 520, ID2 neutral pos 615, ID1 compensation scale 0.80, magnet offset 54 mm.
- Current boot config still enables the power-on internal tool test, magnet hold, ID2 135/45 test and boot buzzer path. Formal USB control requires disabling that test so READY waits for host commands.

## 2026-07-30 ID1 vertical-down compensation direction

- Restored `ARM_TOOL_SERVO1_DIRECTION` from `-1.0f` to the previously correct `+1.0f` physical convention.
- The vertical-down target remains `servo1_deg = 90 deg + small_link_pitch_deg / direction`; with `direction=+1`, a positive small-link pitch commands an ID1 angle above 90 deg to counter-rotate the end tool.
- This change affects only the ID1 tool-servo compensation direction; Damiao joint directions, elbow belt coupling compensation, FK/IK geometry and motor initialization are unchanged.

## 2026-07-30 Single-owner boot refactor completion

- `ArmTask()` now exclusively owns the commissioning sequence: motor enable/feedback synchronization, simultaneous mechanical initialization, safe-pose trajectory, tool initialization wait, stabilization, internal TOOL_TIP test and final READY publication.
- The internal test no longer uses `ArmSubmitCartesianCommand()`, the single-slot Host mailbox or `g_arm_host_status.ready`; those paths are reserved for actual external commands after boot.
- `Test.c` no longer contains `ArmApiPointTestTask()` or `g_arm_api_test_debug`; it only schedules `ArmTask()`, optional disabled simulators and motor control services.
- Tool initialization failure or an 8 s initialization timeout now enters a terminal boot fault instead of waiting forever.
- Boot test failure cancels the trajectory, holds the current pose through the existing cancel path, sets `ARM_MODE_FAULT/ARM_START_FAULT`, never republishes READY and never automatically retries the test.
- The concise bring-up Watch surface is `g_arm_boot_debug`; old automatic-point and API-point debug structures have been removed from source.
- Scoped `git diff --check` passed with line-ending conversion notices only. No Keil build, flash or hardware validation was performed.

## 2026-07-30 Arm boot/test ownership refactor

- Hardware Watch shows `g_arm_api_test_debug.wait_ready=1`, `submitted=0`, `rejected=0`; the test command never reaches the mailbox, so this is not an IK rejection.
- The current one-shot boot test lives in `Test.c` and waits on public `g_arm_host_status.ready`, while that status is produced at the end of `ArmTask()`. This creates an unnecessary cross-task readiness loop for an internal commissioning action.
- The trajectory layer already accepts internal motion while the commissioning boot mode is `ARM_MODE_DM_SINGLE_AXIS_TEST`, so the TOOL_TIP test can be started directly by the arm boot sequence before publishing public READY.
- USART6 tool initialization and CAN motor initialization can remain concurrent; only the transition into the TOOL_TIP test must wait for both auto-init completion and tool init completion.

## 2026-07-30 USART6 Huaner LX integration

- USART6 is now CubeMX-generated as 115200 8N1 full-duplex on PG14/PG9, with RX DMA2 Stream1 Channel5, TX DMA2 Stream6 Channel5 and USART6 IRQ priority 5.
- The external controller board owns the two-wire UART to single-wire servo-bus conversion, so firmware must not add a direction GPIO or switch the MCU UART to single-wire mode.
- The existing `hsl_servo` is Feetech/SCS protocol (`FF FF`, register-address writes) on huart1 and cannot be converted by changing the UART handle alone; Huaner LX uses `55 55`, command-specific parameters and position range 0..1000.
- `catch.c` still calls legacy `WritePosEx2()` with values up to 1500. It must remain unmodified in this phase, so all legacy SCS APIs will be retained as non-transmitting rejected stubs to prevent unsafe reinterpretation.
- The existing USART BSP owns `HAL_UARTEx_RxEventCallback` and `HAL_UART_ErrorCallback` but has no DMA TX/RX-complete dispatch. The new driver therefore needs a compatible callback-registration extension rather than defining competing HAL global callbacks.
- The STM32F4 HAL normal-mode TX DMA callback is raised only after the UART TC interrupt confirms the final stop bit has left the peripheral, not merely when DMA empties its memory buffer.
- Waiting for a later 1 ms task call to start RX can miss a short LX response. Position reads now arm normal RX DMA before TX, then `HSLServoTask()` inspects the DMA remaining count and searches the 16-byte buffer for a valid 8-byte reply. This also tolerates controller-board TX echo before the reply without treating the echo's idle gap as the complete transaction.
- Independently verified frames: `Move(1,500,1000)` is `55 55 01 07 01 F4 01 E8 03 10`; `Stop(1)` is `55 55 01 03 0C EF`; `PositionRead(1)` is `55 55 01 03 1C DF`.
- Per-ID TX/RX/timeout/checksum/frame counters are independent; global totals remain in `g_hsl_servo_debug`.
- Legacy `WritePosEx2()` and related Feetech symbols contain no UART transmit call and only report unsupported/increment `legacy_reject_count`.

## 2026-07-30 Commissioning trajectory integration

- The live geometry and elbow model differ from the original plan snapshot: keep the current source values and do not restore the obsolete `250/260/260 mm` geometry.
- Current `DM_SINGLE_AXIS_TEST` auto initialization and auto XYZ point bypass `ArmTrajectoryTask()` and call `ArmCommandPose()` with the final pose directly.
- The existing trajectory layer already supports synchronized quintic joint interpolation and preflight-checked Cartesian linear motion; integration should reuse it instead of adding another planner.
- One compound `cmd.exe` source-search command failed due quoting/parsing; subsequent compound inspections use w64devkit Bash.
- `DM_SINGLE_AXIS_TEST` now executes `ArmTrajectoryTask()` after the proven three-axis enable/hold reaches READY. Internal trajectory readiness accepts this commissioning mode, while public host command checks remain unchanged.
- Automatic initialization now calls `ArmTrajectoryMoveJointAtSpeed()`: all three joints share one quintic progress variable and the Watch `auto_init.speed_deg_s` actually limits trajectory duration.
- Automatic XYZ motion now calls `ArmMoveLinear()`: the complete straight path is sampled every 2 mm for IK/limit/continuity preflight, then executed with 5 ms online IK and 1 ms reference interpolation.
- Automatic point speed is now explicitly `speed_mm_s`; the default is `150 mm/s`. Existing `g_arm_motion_debug` exposes progress, duration, sample count, acceptance and preflight result.
- Hardware showed no initialization movement after routing initialization through the normal joint trajectory. That entry requires every interpolated reference, including the initial feedback pose, to be inside normal soft limits; a startup pose outside soft limits is rejected and the trajectory time is frozen. Initialization is therefore restored to the previously validated simultaneous direct command, while Cartesian planning remains active only after initialization and belt coupling activation.
- After initialization was restored, the Cartesian motion still correctly rejected its path. FK places `[0,180,-90]` at approximately `(-260,0,292) mm`; a straight line to `(250,0,120) mm` crosses the base-center/branch-change region. Firmware-equivalent 2 mm sampling found no limited IK solution at the second sample.
- A staged route was numerically validated: joint quintic `[0,180,-90] -> [0,90,-90]`, followed by a Cartesian line from `(230,0,322)` to `(250,0,120)`. The line passes all 103 samples and ends near `[0,65.93,-62.82] deg`.
- The first staged trajectory felt slow because the 90 deg safety transition was acceleration-limited at `200 deg/s2`, while the 203 mm Cartesian segment was limited to `150 mm/s` and `600 mm/s2`. Fast commissioning values are now `220-240 deg/s`, `700-800 deg/s2`, `300 mm/s`, and `1800 mm/s2`; motor reference frames use a matching `240 deg/s` velocity cap.
- The path was not five points: `5 ms` was the online-IK update period. The 203 mm straight segment previously had about 103 preflight samples at 2 mm spacing plus 1 ms reference interpolation. It now uses 1 mm preflight spacing (about 204 samples), 2 ms online IK, and higher limits for every phase: initialization 240 deg/s, staged joints 380-420 deg/s with 1500-1800 deg/s2, Cartesian 450 mm/s with 3600 mm/s2, and 120 ms arrival stability.
- The visible pause at `[0,90,-90]` was caused by two independent quintic trajectories: the first explicitly reached zero velocity and waited for arrival stability, then the Cartesian segment restarted from zero. The auto route is now one cached composite trajectory with roughly 91 joint-transition samples plus 204 Cartesian IK samples under one global quintic time base. The safe pose remains a mandatory waypoint but is no longer a HOLDING state or stop point.

## 2026-07-30 Enable success and fast commissioning decision

- The user has now physically confirmed that all three Damiao motors can be enabled. This closes the raw-ID Enter Motor Mode and no-feedback startup-deadlock bring-up stage.
- The next unsafe boundary is no longer CAN enable; it is unverified physical direction and absolute-angle mapping under commanded motion.
- Switching directly from enable-only to NORMAL would immediately allow automatic elbow/shoulder/base motion. A faster controlled route is one commissioning image that keeps all three motors enabled and holding while selecting one moving axis at runtime through Watch.
- The existing single-axis harness already has sticky request/applied fields and `2 deg`/`5 deg` limits, but its selected axis is initialized from a compile-time macro and its startup path does not reuse the proven three-axis enable-only sequence.
- The commissioning firmware now defaults to `ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST`, but its startup calls the proven `ArmProcessEnableOnly()` sequence first. All three axes are enabled and synchronized before any test request can be accepted.
- `selected_test_axis=0` is a safe no-motion selection and no longer disables the motors. Runtime selections are `1=base`, `2=shoulder`, `3=elbow`; changing selection alone does not move anything.
- A move occurs only when `test_request` differs from `test_applied`. The firmware copies every processed request to `test_applied`, records the applied axis/start/target, and returns a result code even when the request is rejected.
- Non-selected axes keep their existing position targets and remain enabled. The command uses current feedback plus the requested relative logical angle, stays inside the normal software limit, defaults to `2 deg`, is capped at `5 deg`, and uses `5 deg/s`.
- A new request is rejected as `ARM_COMMAND_BUSY` while any prior axis still has more than `1 deg` tracking error or more than `2 deg/s` logical speed, preventing accidental Watch-side command stacking.
- Final commissioning Keil image built incrementally with 0 errors and 0 warnings. Size is Code 46744, RO 672, RW 916, ZI 101872 bytes. The map retains `ArmProcessSingleAxisTest` and `DMMotorEnterModeAndHoldOpenLoop`; `ArmProcessStartup` is removed, so NORMAL automatic escape/return cannot run in this image.

## 2026-07-30 Full-Damiao Implementation Results

- Host-control packaging decision: communication modules will include `arm_host.h`, submit one `Arm_Command_s`, and read one `Arm_Host_Status_s`. Damiao instances, CAN counters, trajectory caches and bench debug structures remain outside this stable interface.
- User selected startup option 1: the arm must not report host READY at the rear initialization pose `[0,180,-90]`; it must continue to the forward safe pose `[0,90,-90]` and satisfy the arrival stability check first.

- The automatic host simulation now cycles through `(250,50,120)`, `(250,50,150)`, `(250,-50,150)`, and `(250,-50,120) mm`. The first command retains the safe-region rounded composite entry; subsequent commands use the normal Cartesian-linear preflight path.
- The 3000 ms interval is measured from command acceptance. If a trajectory takes longer than 3000 ms, firmware waits for it to finish and dispatches the next point immediately instead of overwriting an active trajectory.

- The first continuous joint-then-linear implementation removed the explicit stop at `[0,90,-90]`, but it still parameterized cached samples by array index. Joint staging samples are about 1 deg apart while Cartesian IK samples are about 1 mm apart, so the shoulder reference speed could still change sharply at the waypoint even though the motion state remained RUNNING.
- Cached joint paths now assign progress from each segment's minimum q1/q2/q3 travel time under the configured joint speed limits. Dense Cartesian IK samples therefore consume proportionally less global progress than the 1 deg staging samples, removing the artificial sample-density speed collapse without changing geometry, belt compensation, motor direction or the target point.
- The exact safe-pose join is also a geometric corner: the incoming path is shoulder-dominant while the outgoing linear IK path is elbow-dominant. A 12 joint-interval / 12 Cartesian-interval cubic Bezier blend now rounds only this local join, stays inside the same joint safety limits, and preserves tangent continuity into and out of the blend.

- Physical bring-up reported that only the elbow appeared enabled. The current module sets `mode_entered/control_enabled` from CAN transmit success, although that only proves the STM32 queued a frame and does not prove the motor entered Motor Mode.
- NORMAL currently sends Enter Mode during `ARM_START_SYNC_TARGETS`, then immediately begins escape/return without waiting for a fresh state-confirming feedback frame. Startup must instead gate all later behavior on explicit three-axis enable confirmation.
- The current position-speed-mode CAN identifier remains `0x100 + Motor ID`. No CAN-analyzer or vendor-document evidence is present in the workspace to justify changing special-command IDs separately, so this correction is limited to ordering and feedback confirmation.
- `DMMotorEnterMode()` now records the current RX count and leaves `mode_entered/control_enabled` cleared. A later feedback frame confirms Motor Mode only when it is newer than the request and reports `DM_STATE_MOTOR_MODE (1)`.
- NORMAL startup now executes `WAIT_PASSIVE_FEEDBACK -> ENTER_ALL_MODES -> WAIT_ENABLE_CONFIRM -> SYNC_TARGETS -> VALIDATE_POSITION`; target synchronization and all automatic movement are unreachable until every axis confirms Motor Mode.
- The final NORMAL map retains `ArmAllModesConfirmed`, `DMMotorModeConfirmed`, `DMMotorEnterMode` and `ArmProcessStartup`.
- Hardware LEDs now show all three motors briefly green and then red. This proves the Enter Mode frame reaches all three motors; the firmware subsequently disables them. The immediate task is therefore an isolated enable-and-hold firmware, not further NORMAL startup refinement.
- Feedback `state==1` has not been validated against the actual position-speed-mode state semantics. It must not be used to force Reset Mode during the enable-only test.
- The final enable-only image synchronizes all three absolute positions before Enter Mode, enables each axis, starts the normal 2 ms position-speed hold service immediately, and refreshes Enter Mode every 100 ms.
- The enable-only image's linker map removes `ArmProcessStartup` and `ArmBeginReturnSequence`; `DMMotorDisable` is also absent from the final map. Therefore this build has no linked automatic Reset Mode path from the arm application.
- User-provided upper-computer capture proves the elbow Enter Motor Mode frame is standard data frame ID `0x001`, DLC 8, payload `FF FF FF FF FF FF FF FC`, with feedback on `0x011`. The previous firmware incorrectly sent the same payload on position-speed ID `0x101`.
- `DMMotorSendModeCommand()` now temporarily changes the CAN header to the raw Motor ID for the synchronous HAL mailbox enqueue, then immediately restores the position-speed command ID. Expected special-command IDs are base `0x003`, shoulder `0x002`, elbow `0x001`; normal position-speed IDs remain `0x103/0x102/0x101`.
- The prior note saying `DMMotorDisable` was absent from the final map became stale after later public-symbol retention changed link reachability. The active enable-only `ArmTask` path still does not call it, and `ArmEmergencyStop` is removed from the current image.
- A second startup bug explained the all-red result after correcting CAN IDs: `ArmProcessEnableOnly()` required all three passive feedback streams before it sent Enter Mode. Motors that remain silent while disabled therefore caused an infinite `WAIT_PASSIVE_FEEDBACK` deadlock, so no corrected enable frame was actually transmitted.
- The corrected enable-only sequence now waits only 300 ms, sends raw-ID Enter frames regardless of feedback, refreshes them every 100 ms, and waits for feedback solely before enabling the position-speed current-position hold service.

- The Damiao module now treats position-speed commands and feedback entirely in raw motor radians/rad/s; installation direction is owned only by the arm adapter.
- Position-speed velocity is a positive speed limit, not a signed velocity command.
- `DMMotorInit()` registers CAN only. Position frames require valid feedback, a synchronized target, entered motor mode, enabled control and no latched module fault.
- The unified `DMMotorControl(HAL_GetTick())` call is in the existing 1 ms command task; its internal 2 ms gate sends each enabled Damiao motor at approximately 500 Hz and rotates the first axis.
- Default firmware is intentionally `DM_SINGLE_AXIS_TEST + ARM_DM_TEST_NONE`, so no motor is selected or automatically enabled after build.
- Normal startup is implemented separately and compiles when `ARM_BOOT_MODE_NORMAL` is selected: listen, optional Enter Mode, validate, sync, escape elbow/shoulder/base, return elbow/shoulder/base, READY.
- Fault reset invalidates cached feedback after Clear Fault, then waits for a new frame before target synchronization. Old trajectory state is aborted and cannot resume.
- Static limits are q1 `[-90,90]`, q2 `[40,180]`, q3 `[-140,10]`; escape bounds are each widened by 20 deg.
- Safe pose FK is exactly `(260,0,510) mm`. Five representative FK/IK round trips passed with worst host error `6.10351562e-05 mm`.
- Final ARMCC build passed with 0 errors/0 warnings and image sizes Code 46224, RO 672, RW 916, ZI 101820 bytes.
- A temporary NORMAL-mode ARMCC link retained the full escape/return state machine and passed with 0 errors and only 2 compile-time unused bench-helper warnings; size was Code 55116, RO 676, RW 916, ZI 101820. The default was then restored and rebuilt as `DM_SINGLE_AXIS_TEST + NONE`.
- The map retains `DMMotorInit`, `DMMotorControl`, `DMMotorSetPositionSpeed`, `ArmInit` and the single-axis harness. Old arm homing/calibration public symbols are absent.
- Actual motor feedback state-code meaning, passive-feedback behavior, direction, no-kick behavior, temperature under load and CAN analyzer timing remain hardware validation items.
- Host GCC initially failed because its temp directory resolved to `C:\WINDOWS`; using project-local `TEMP/TMP` resolved it.

## 2026-07-30 Full-Damiao Refactor Audit

- The workspace root is a wrapper; the active embedded project is under `Engineer/` and contains `Engineer.ioc`, a Keil project, `MODULE/motor/DMmotor`, `MODULE/motor/DJImotor`, and the shared `motor_task` layer.
- Current user-owned/irrelevant dirty state includes `.vscode/BROWSE.VC.DB`; it must not be reverted or modified as part of the arm refactor.
- The first `findstr` multi-pattern filter failed because cmd treated later terms as filenames. Use `rg` regex filtering directly for Unicode-safe, predictable inspection.
- Cmd also interpreted quoted regex alternation as shell syntax, and inline Python quoting failed. Avoid compound quoted regex and inline scripts; use simple literal `rg` patterns or a helper file if needed.
- The new hardware facts supplied by the user are not yet verified against runtime code: base DM4310 ID3/Master 0x13, shoulder DM4340 ID2/Master 0x12, elbow DM4310 ID1/Master 0x11; all use speed-position mode and absolute encoders.
- Geometry target for the new model is shoulder-axis height 250 mm, upper arm 260 mm, forearm 260 mm, with no stated Y-axis offset among the three axes.
- `DMMotorConfigModel()` supports position-speed mode by changing the transmit identifier from motor CAN ID `n` to `0x100+n`; the configured receive ID is not changed there and is therefore the likely place for each `Master ID`.
- Position-speed commands are sent as two native 32-bit floats: position in radians and velocity in radians per second. `DMMotorSetRef()` and `DMMotorSetSpeedRef()` therefore cannot receive the existing arm degree units directly.
- The wrapper's reported feedback exposes `measure.position` in radians, `measure.velocity` in rad/s, and a custom `total_angle` in degrees. The current `total_angle` reconstruction assumes wrapping over a `4*pi` threshold and scales position by `position/(4*pi)*360`; this needs verification against the actual DM4310/DM4340 feedback protocol and should not be trusted as the new absolute-joint source yet.
- The DM wrapper has several runtime hazards relevant to a three-motor arm: feedback reverse settings are not applied, position-speed mode reverses position but not speed, `DMMotorStop()` sends zero velocity while retaining the last position target, a shared global `time` is incremented by all motor tasks, an error branch uses `return` and permanently exits that motor task, and `DMMotorControlInit()` builds task names in an undersized/mutating buffer.
- `MotorControlTask()` only calls `DJIMotorControl()`; DM motors instead depend on per-motor RTOS tasks created by `DMMotorControlInit()`. The initialization ordering and actual call site must be verified before considering the wrapper usable.
- The live linker map removes all DM initialization/control sections as unused. The current application calls `ArmInit()` before `MX_FREERTOS_Init()` and scheduler start, and its 1 ms command task calls only `DJIMotorControl()`, so no DM control task exists in the running firmware.
- For the proposed position-speed configuration, expected standard CAN transmit IDs are base `0x103`, shoulder `0x102`, elbow `0x101`; the likely feedback filters are the provided master IDs `0x13`, `0x12`, `0x11` respectively.
- Safe startup cannot reuse the wrapper's current default behavior. It should register motors without commanding a new position, wait for valid absolute feedback after scheduling starts, seed each command with its current measured position, then enable one axis at a time under a low speed limit.
- Current live arm configuration is `baseHeight=93 mm`, `L1=150 mm`, `L2=141 mm`, not the older planning snapshot. The new `250/260/260 mm` geometry must be propagated through FK/IK, reach checks, test points, and trajectory preflight from the live code.
- The existing IK is gated by legacy `base_calibrated/joint_calibrated` flags and uses scan-derived `g_arm_calibration` soft limits. Those semantics must become configuration/absolute-feedback-validity semantics rather than being left as fake homing completion.

## 2026-07-27 Linear Motion Implementation

- The live firmware already has verified one-sided homing, measured motor-to-joint mapping, angle-loop control, soft limits, and a 1 ms `ArmTask()` call followed by `DJIMotorControl()`.
- Existing `ArmSetJointTargetDeg()` clears all three PID controllers every call, so continuous trajectory references require a separate no-reset update path.
- Boot homing ends at q2=180 and q3=-180, outside the 5-degree software limits. Automatic motion therefore needs a one-time hard-limit exit corridor before normal trajectory checks apply.
- The conservative loop points are numerically reachable with continuous IK branches: P1 `(27.13,-7.60,120.04)`, P2 `(49.31,14.61,117.82)`, P3 `(26.13,-20.57,99.34)` mm.
- The repository is dirty from the complete arm bring-up and Keil outputs. Changes must remain additive and must not replace the tuned motor registration structures.
- The closed P1-P2-P3-P1 path passes the firmware-equivalent 2 mm preflight over 53 samples. Maximum adjacent joint changes are `3.066/1.034/0.570 deg`, below the configured `5/2/2 deg` thresholds.
- The reference FK self-test returns `(0,-7.6,34) mm` for `[0,180,-180] deg`, and all three target-point IK solutions round-trip to numerical precision.
- MATLAB command-line batch mode still crashes on this host with exit `0xc0000409`; the script now contains the validation function, but automated MATLAB GUI execution remains unverified.

- Current task is limited to the three DJI motors; PWM wrist behavior must remain inactive.
- The repository is already dirty from CubeMX, Keil output, and the previous arm refactor, so edits must remain narrowly scoped.
- `DJIMotorInit()` enables a motor by default, so GM6020 must be stopped immediately after registration and on every arm task iteration.
- `speed_aps` is degrees per second and `real_current` is filtered raw current feedback.
- `DJIMotorReset()` updates `zero_offset`; writing `total_angle = 0` immediately afterward makes Watch reflect the new software zero before the next CAN feedback frame.
- Homing should use a speed outer loop with output limiting so the commanded speed and current ceiling are explicit.
- A Watch-controlled abort is necessary for bench work; clearing abort alone must not resume motion, and restart remains an explicit separate action.
- Abort must move both homing channels into a terminal `ABORTED` state; clearing the abort flag alone otherwise resumes the previous seek state.
- GM6020 was moved to CAN1 ID1. It does not conflict with M3508 ID2 or M2006 ID3 because their receive IDs are 0x202/0x203 while GM6020 ID1 receives on 0x205; their command frames are also split between 0x200 and 0x1FF.
- `DJIMotorReset()` now updates `zero_offset` and explicitly sets `measure.total_angle` to zero; the application no longer duplicates the direct field write.
- The user's Watch screenshot (`zero_offset=0`, `total_angle=2275.09`) proves the shoulder reset branch had not executed. Independent per-motor diagnostics were added to distinguish threshold failure, timeout/offline stop, and successful zeroing.
- Homing speed and stall-current thresholds now use separate named globals for M3508 and M2006 rather than indexed arrays, so each motor's parameters are unambiguous in Watch.
- The screenshot's roughly six motor turns at 180 deg/s is consistent with the former 15-second timeout. The default timeout is now disabled (`0`) so a motor stops only on its own confirmed stall or explicit abort.
- Added independent peak absolute current tracking for M3508/M2006 to support threshold tuning from live Watch data.
# Final Implementation Findings

## 2026-08-02 Gripper and differential chassis baseline

- The live arm has three Damiao main joints; USART6 servo ID1 currently owns vertical-down compensation and ID2 owns base/world-yaw compensation. Only these two tool behaviors are being replaced.
- `ChassisTask_f` runs the combined `all_cmd_Task()` and the legacy `ChassisInit/ChassisTask` are not called. The replacement chassis must be explicitly initialized and serviced.
- The BMI088/INS implementation exists, but `ImuTask_f` currently only delays. The new chassis must call `INS_Init()` once before the scheduler and `INS_Task()` at 1 kHz.
- CAN2 M3508 ID1/ID2 use feedback IDs 0x201/0x202 and the existing shared 0x200 current frame slots. The unused catch mechanism is on CAN1 and remains disabled.
- Existing DJI feedback is `total_angle` and `speed_aps` in motor-side degrees and degrees/second. Differential-drive conversion must apply gear ratio and explicit feedback signs; the motor reverse flag alone does not normalize feedback direction.

## 2026-08-01 Generated protocol STOP update

- The latest generated protocol changes only two wire-visible definitions relative to the live firmware: `PROTOCOL_HASH` is now `0x8845D84A`, and `Status` adds `STATUS_STOP=3`. Packet IDs, field layouts, packed sizes, CRC8, transparent reliable sequence byte, ACK behavior and retry settings remain compatible.
- The generated `protocol.c` is a baseline transport and lacks the live firmware's USB-copy ownership, priority ACK queue, richer diagnostics and queue recovery. The implementation therefore synchronized the wire definitions without replacing those proven firmware extensions.
- STOP now owns two explicit states: cancel completion and HOME completion. The magnet is intentionally preserved through cancellation and HOME motion, and is switched off only after feedback-confirmed HOME completion.
- A successful STOP clears the active task and returns reliable `CallbackStatus{original_task_id, STATUS_STOP}`. A repeated new-sequence STOP in the already safe state only reissues that callback; a duplicate reliable sequence remains handled by protocol ACK/de-duplication before business delivery.
- STOP failure does not claim completion or prematurely drop a held object: HOME/cancel faults return `FAULT_RETRY`, and HOME failure leaves the magnet unchanged.

## 2026-08-01 X-adaptive Z calibration

- USB motion coordinates are in millimetres, so the requested X endpoints `24/45` are represented as `240/450 mm` in firmware.
- Default Z is now `32 + clamp((X-240)/210, 0, 1) * 3 mm`; magnet descent Z is `20 + clamp((X-240)/210, 0, 1) * 3 mm`.
- Thus X=345 mm produces default Z=33.5 mm and magnet Z=21.5 mm; X outside 240..450 mm uses the nearest endpoint height.
- The mapping is centralized in `arm_usb_bridge.c`; normal moves use the target X, loaded pre-lift uses the current X, and Task 5/6 descent/return use the X frozen when the task is accepted. Fixed HOME Z is intentionally unchanged.
- ARM GCC syntax-only checking passed for the changed bridge. Keil compilation and physical height verification remain manual.

## 2026-08-01 Arrival-timeout follow-up

- The upper-computer log showed repeatable first-pass radial undershoot of about 7.5-8.1 mm, followed by success when the identical point was resent. This was not a USB ACK or parser failure.
- The feedback-settling timer correctly starts after the prepared trajectory reaches its final reference; it was not accidentally counting the trajectory runtime itself.
- The former timeout path submitted `CANCEL_MOTION`, which replaced the original final target with the current lagging feedback pose. That froze the undershoot and made a second upper-computer command necessary.
- The revised timeout path preserves the original final target, relaxes arrival to `2 deg / 5 deg/s / 120 ms`, and reduces Cartesian acceleration to `5000 mm/s2`. Real faults still retain cancel behavior.

- Normal boot now performs only single-reference homing after both M3508 and M2006 remain online for 500 ms; M2006 homes first and M3508 homes second.
- M3508 and M2006 scan independently in sequence. Only the active joint is enabled; GM6020 remains stopped throughout.
- At each reference stop, torque is removed immediately, both joints remain disabled for 2000 ms, and only then does the interrupt-protected `DJIMotorReset()` update `zero_offset` and clear `measure.total_angle`.
- The optional maintenance full scan continues from the reference zero to the opposite stop, records the signed motor span, and refreshes the RAM motor-to-joint scale without resetting at the opposite stop.
- The retained bench defaults are shoulder `350 deg/s, 1300 current units` and elbow `800 deg/s, 1400 current units`; low-speed threshold is `20 deg/s`, startup mask `500 ms`, and stall confirmation `8 ms`.
- `g_arm_homing_abort` is the only Watch-write variable retained, and is only needed for emergency stop. After an abort, clearing it does not restart motion automatically.
- Measured joint endpoints are compiled as q2 `-9 to 188 deg` and q3 `-101 to 132 deg`; normal homing immediately makes q2/q3 feedback valid. IK still waits for optional GM6020 front teaching.
- Firmware FK/IK uses the wrist joint center with `baseHeight=80 mm`, `L1=150 mm`, and `L2=179 mm`. MATLAB uses the same first two links but extends to the tool tip with `L3/q4`.
- IK remains a callable C API that writes only the caller-provided result structure; it does not enable a motor or change a motor reference.
- `ARM_BOOT_FULL_SCAN=0` selects normal single-reference boot. Set it to `1` only for maintenance, then copy the resulting spans from `g_arm_calibration` back to persistent constants before returning it to `0`.
- Temporary M3508 ratio-test firmware currently has internal `arm_3508_ratio_test_enable=1`. It commands 1710 motor degrees at 90 motor-deg/s; restore the value to `0` after the test to re-enable normal arm homing.
- Final Keil ARMCC build completed with zero errors and zero warnings. MATLAB batch execution is unavailable on this host due process exit `0xc0000409`, so formula parity was checked from source and independent numerical round trips.
## 2026-08-02 Chassis-only 1 m test findings

- The active test initializes only USB/protocol/buzzer, INS, and the two CAN2 M3508 chassis motors; no arm motor or tool-servo instance is registered.
- M3508 driver feedback units are motor-side degrees and degrees per second. Distance conversion therefore divides total angle by 19.2032 and multiplies by the 47.5 mm wheel radius.
- The right wheel uses explicit command and feedback signs of -1; the DJI driver's motor reverse setting remains normal so application semantics are visible in chassis_config.h.
- DaemonIsOnline is used as the feedback gate. The motors remain stopped until both feedback streams and the IMU are healthy, then the test locks encoder and YawTotalAngle zero references.
- The first hardware run must be wheels-off-ground because command, feedback, and IMU yaw signs still require physical confirmation.
## 2026-08-10 - USART6 feedback-closure baseline

- CubeMX now generates USART6 full-duplex at 115200 8N1 on PG14 TX and PG9 RX, with normal-mode RX DMA2 Stream1 Channel5 and TX DMA2 Stream6 Channel5. USART6 and both DMA IRQs use priority 5.
- The live move APIs emit Huaner controller-board frames (`55 55 LEN 03 ...`) without a checksum, while the live position request/parser still uses an incompatible direct-servo ID/length/checksum layout. That mixed protocol is the first functional blocker.
- The existing move completion path copies the commanded position into status and marks it valid without a reply. Closed-loop supervision must keep commanded and measured positions separate.
- The current chassis-only firmware does not initialize or service `hsl_servo`. The safe first hardware image should poll ID1/ID2 only, keep arm/chassis motor control disabled and require a sticky Watch unlock/request before any move.

## 2026-08-11 - Huaner feedback-supervision implementation

- The active driver now consistently uses the Huaner controller-board protocol: move command `0x03`, dual-ID position query `55 55 05 15 02 01 02`, and strict two-ID reply length/count validation.
- The reply layout is still a hardware assumption until a real controller-board capture confirms `55 55 09 15 02 ID POS_L POS_H ...`; the parser intentionally rejects wrong commands, duplicates, missing IDs, truncated frames and positions outside `0..1000`.
- `HSLServoInit()` runs the same frame builder/parser through a deterministic self-test. `g_hsl_servo_debug.protocol_self_test_passed` must be `1` before USART6 is accepted as initialized.
- Public move/query APIs build into local stack frames. The driver reserves the single transaction, allocates status slots, copies the frame into the DMA-owned buffer and publishes the state while interrupts are masked, so a BUSY caller cannot overwrite an active TX DMA buffer.
- RX-to-idle DMA is armed before a query TX. HAL callbacks only publish completion/error flags; parsing, timeout recovery, health supervision and polling remain in the 1 ms task.
- Per-servo status separates target and feedback positions and exposes velocity, arrival stability, stale/offline state, motion timeout, timestamps and counters. A successful TX no longer claims that a servo is online or at target.
- The active test image has `HUANER_SERVO_CLOSED_LOOP_TEST_ENABLE=1` and `CHASSIS_ONE_METER_TEST_ENABLE=0`. It does not call `ArmInit`, `ArmTask`, Damiao control, `ChassisInit`, `ChassisTask` or `DJIMotorControl`.
- Motion is locked at boot. Watch must set `g_huaner_servo_test_debug.motion_unlocked=1`, fill `request_id/request_position/request_time_ms`, then change `request_seq`; no automatic sweep or boot movement exists.
- Final Keil build generated current `axf/hex/map`: Code 66480, RO-data 2992, RW-data 896, ZI-data 99076, `0 errors / 10 warnings`. All 10 final incremental warnings are from legacy `catch.c/catch.h`; the changed servo, test, arm-tool and USART files compile without warnings.
- No firmware flash, logic-analyzer capture or physical servo motion was performed. The first bench run must validate the reply bytes and `0..1000` range before Watch motion is unlocked.

## 2026-08-11 - ID1 Watch telemetry scope

- The active controller-board position reply contains only servo ID and position, so measured angle is the only requested physical telemetry available from the verified frame.
- Servo current, individual servo voltage and temperature are not present in that reply. The firmware must not populate Watch fields for them with zero, constants or values borrowed from the incompatible direct-servo protocol.
- The public `g_huaner_servo1_debug` snapshot can be reduced independently; sweep scheduling and error handling belong in private `Test.c` runtime state.

## 2026-08-11 - Controller-board voltage command scope

- The photographed Hiwonder 32-bit ARM bus-servo controller uses the active `9600` host-UART framing and advertises board low-voltage alarm plus servo position readback, but not servo temperature readback.
- First-pass voltage telemetry is therefore explicitly board supply voltage, using controller command `0x0F`: request `55 55 02 0F`, expected reply `55 55 04 0F VL VH`, little-endian millivolts.
- Voltage polling must remain independent from per-servo online state. A voltage timeout or malformed voltage frame must invalidate only board voltage and must not mark ID1 offline or interrupt the sweep.

## 2026-08-11 - Phase 35 live baseline

- Active `all_init_Task()`/`all_cmd_Task()` currently own only the ID1 0/90-degree sweep; `ArmInit()` is commented and the arm USB bridge is not in the active schedule.
- `hsl_servo` already provides one-owner DMA transactions, dual-ID 50 ms position polling, board-voltage polling, target/feedback error, measured position velocity, arrival stability and timeout supervision.
- `arm_tool` still contains ID1 `0.80` vertical compensation, the old 60.3/54 mm magnet geometry, ID2 world-yaw/base compensation, repeat sends and PB12 magnet state.
- `arm.c` and `arm_trajectory.c` still reject nonzero `tool_pitch_valid`; the USB `TargetControl` and `MotionStatus` third float are still named `yaw_deg`.
- The generic UART driver must remain communication-only. Gripper contact/jam semantics belong in `arm_tool`, where target intent and feedback history are both available.

## 2026-08-11 - Phase 35 completed implementation

- ID1 now implements absolute tool pitch from the small-link reference with `125/500/875 = -90/0/+90 deg`; every public, trajectory-preflight and final-send path rejects out-of-range values rather than clamping them.
- Cartesian commands now target the gripper center using a 30 mm pitch-axis offset. Direct, linear, realtime and joint trajectories lock or accept an absolute pitch, preflight the complete ID1 path and update the servo target every 20 ms.
- ID2 now owns only gripper boot, ready, open and close behavior at `1000/850/800/950`. One-shot closed-loop commands advance from fresh dual-servo position feedback instead of repeated target transmission.
- Close/boot stall detection uses the locked 300 ms ignore/window timing, error/span/travel thresholds and a 10-position relief. A travelled close obstruction becomes `HELD_CONTACT`; insufficient travel becomes `JAMMED`; boot obstruction prevents READY; reaching 950 becomes `CLOSED_EMPTY`.
- Task 5, Task 6 and STOP now use feedback-confirmed gripper actions. STOP keeps the current grasp while returning HOME, then opens to 800 and leaves the gripper open.
- USB IDs `0x04/0x05` now carry pitch, IDs `0x06/0x07` provide reliable tool command/status, maximum payload is 32 bytes and `PROTOCOL_HASH` is `0x1E7AC5B2`.
- The production `ArmInit`/`ArmTask`/USB bridge schedule is active again. The five-second ID1 sweep remains available only behind `HUANER_SERVO_ID1_SWEEP_TEST_ONLY=0` by default; chassis and IMU are not activated by this phase.
- Live USART6 configuration remains 9600 baud in `usart.c`, `Engineer.ioc` and `HSL_SERVO_CONTROLLER_BAUD_RATE`; this phase intentionally did not change CubeMX/UART configuration.

## 2026-08-11 - Phase 36 live switching baseline

- The differential chassis implementation and `CHASSIS_AUTO_FORWARD_TEST_ENABLE=1` remain intact in `chassis.c/chassis_config.h`; the active runtime currently selects the production arm only because `Test.c` initializes and services `ArmInit/ArmTask`.
- `chassis.h` already exposes the stable runtime interface `ChassisInit(attitude_t *)`, `ChassisTask(now_ms)`, `ChassisNotifyImuUpdate(now_ms)` and `ChassisEmergencyStop()`, plus the Watch-visible `g_chassis_debug` odometry and safety snapshot.
- The arm tool and USB packet ABIs are already complete. The requested interface cleanup should add Chinese semantic comments only and must not reorder enums, fields or packed packet layouts.
- The safest mode switch is one explicit compile-time selector in `Test.h`, with a compile-time mutual-exclusion check against the isolated ID1 sweep selector.
- `RobotCMDInit()` only calls `DWT_Init(168)` and does not register motors or remote-control instances; it must remain enabled in chassis mode because INS timing uses DWT delta time.
- `INS_Init()` explicitly documents that it must run outside the realtime task and returns the shared `attitude_t` snapshot. `INS_Task()` is the 1 kHz owner of BMI088 reads and EKF updates, so the chassis task must only consume that snapshot.
- The existing USB task can remain active for transport/diagnostics in chassis mode, but `ArmUsbBridgeTask()` must be gated to production-arm mode so no arm business state machine is serviced.

## 2026-08-11 - Phase 36 completed chassis image

- `CHASSIS_ONE_METER_TEST_ONLY=1`, `HUANER_SERVO_ID1_SWEEP_TEST_ONLY=0` and the derived production-arm mode is false. A preprocessor error prevents enabling both bench modes together.
- Startup now performs `USB_Init -> protocol_init -> BuzzerInit -> INS_Init -> ChassisInit`; it does not call `ArmUsbBridgeInit`, `ArmInit` or `HSLServoInit`.
- The IMU task now performs `INS_Task()` and then `ChassisNotifyImuUpdate()` every 1 ms. The combined-control task calls `ChassisTask()` and `DJIMotorControl()` every 1 ms; `ChassisTask()` retains its internal 5 ms gate.
- Existing 1.0 m distance, 0.20 m/s maximum speed, acceleration/deceleration, encoder conversion, IMU heading hold, motor/IMU timeout and direction safety settings were not changed.
- The final MAP removes all arm/Damiao/tool initialization and periodic-control entry points while retaining the chassis, INS and DJI control symbols.
- The final isolated image uses Code 64976, RO 2984, RW 1360 and ZI 98568 bytes. SRAM1 uses `0x18330 / 0x1c000`, leaving about 15 KiB headroom.
- A final isolation audit found that `USB_ProcessTask()` feeds bytes directly into `protocol_fsm_feed()`, whose strong arm-bridge callbacks can accept arm commands even when `ArmUsbBridgeTask()` is gated off. The chassis mode therefore also gates USB RX/TX parsing and protocol tick; the USB task retains only buzzer and daemon service.
- The rebuilt MAP confirms `USB_ProcessTask`, `protocol_tick`, `on_receive_TargetControl`, `ArmUsbBridgeTask` and all arm/Damiao/tool runtime entry points are removed, while chassis/INS/DJI entry points remain linked.

## 2026-08-11 - Phase 39 endpoint and HOME baseline

- The active arm image already runs the combined servo/Damiao initialization path under the historically named `ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST`; switching to the old `NORMAL` branch would select a different sequential escape/return state machine.
- `ArmResolveHomePose()` currently bypasses IK in that active mode and directly uses `ARM_SAFE_Q*`; normal tool-enabled HOME instead treats `ARM_USB_HOME_*` as the gripper center and subtracts the 30 mm offset.
- Cartesian direct, linear and realtime paths still convert `ARM_CONTROL_POINT_TOOL_TIP` through the 30 mm gripper-center offset, while lower-level `ArmSetCartesianTarget()` already treats its input as the wrist/ID1-axis point.
- Host status currently reports the calculated gripper center in `position_mm/tool_tip_mm`; the USB bridge uses `tool_tip_mm` as the current Cartesian point, so status must change together with command semantics.
- With `BASE_HEIGHT=62 mm`, `L1=L2=260 mm` and `q3_math=-180-q3`, pose `[0,90,-60] deg` gives `q23=-30 deg` and axis position `(225.1666,0,192.0) mm`.
- `q2=90 deg` and `q3=-60 deg` are inside the live software/automatic limits (`q2=35..180`, `q3=-190..-35`) and away from their boundaries.
- The final implementation keeps `g_arm_state.tool_tip` as the physical 30 mm-offset gripper center for diagnostics, but reports `g_arm_state.wrist_center` through host `position_mm/tool_tip_mm` and treats all compatible TOOL_TIP Cartesian commands as the same ID1-axis endpoint.
- The protocol layout is unchanged, but its hash is now `0x90A149D1` because X/Y semantics changed; an old host build must not silently control the new endpoint definition.
- The final MAP shows `ArmResolveHomePose` directly references `ArmInverseKinematics3DOF`, and the active image includes the full arm/tool/USB chain while removing chassis/INS initialization.

## 2026-08-11 - Phase 40 FruitDetection protocol baseline

- The supplied generated protocol is a replacement business contract, not an additive update: only Ack, Heartbeat, Handshake and `FruitDetection{fruit_id,status}` remain, with hash `0x923FFDD9`.
- `FruitDetection` has a two-byte payload and no appended reliable sequence byte, so the MCU must not ACK it or interpret repeated packets as repeated mechanical actions.
- The live project protocol still has the old Task/Target/Tool packet types and three typed reliable status FIFOs. Its USB copy queue, priority system queue, parser resynchronization and disconnect recovery remain useful and must be preserved.
- The live protocol declares handshake and heartbeat switches but currently uses optional handshake and no strict timeout; strict 3000 ms session expiry must therefore be implemented, not only configured.
- The complete arm test path currently calls `ArmUsbBridgeInit/Task`; the replacement observer requires a new bridge and Keil project membership change while leaving the old dirty bridge sources untouched.
- The live base mapping already has logical zero 0, direction +1 and q1 limits -90..90. The requested physical positive-X reversal is a Damiao saved-zero calibration step, not an FK sign change or a software 180-degree offset.

## 2026-08-11 - Phase 40 completed implementation

- The active USB business protocol now exposes only `FruitDetection 0x10`, `Ack 0xFD`, `Heartbeat 0xFE` and `Handshake 0xFF`, with hash `0x923FFDD9` and a 32-byte parser payload ceiling.
- A matching handshake is mandatory before application delivery. A wrong hash is counted and receives no echo; after a valid handshake, heartbeat count is echoed and an exact 3000 ms gap invalidates the session and latest fruit result.
- `FruitDetection` remains a fixed two-byte non-reliable observation packet. It never receives an ACK, and repeated valid frames only refresh `g_fruit_usb_debug` without submitting any arm, trajectory or gripper command.
- `fruit_id=1..6,status=0..1` is valid; `{0,0}` is no target; all other zero-ID/status or out-of-range combinations are rejected. Disconnect and heartbeat expiry retain raw diagnostic values while forcing `valid=0`.
- The active schedule initializes USB, protocol, fruit bridge, buzzer and the complete arm/Damiao/Huaner chain. It does not initialize chassis or INS, and the legacy arm USB bridge remains in the worktree but is absent from the active Keil target and schedule.
- Positive X remains a physical calibration responsibility: point the base toward the desired positive-X side and save that position as zero with the Damiao host tool. Firmware keeps zero offset, positive direction and q1 limits centered on zero and never issues a motor-zero command.
- Host checks passed the complete protocol state matrix and verified HOME FK `(225.166580,0,192.000000) mm` plus IK `(0,90.000008,-60) deg`. The final Keil image builds with 0 errors and 0 warnings and fits all flash/SRAM regions.

## 2026-08-11 - Phase 41 initial hardware symptom

- The active full-arm boot sequence is tool-first: lack of valid USART6 ID1/ID2 feedback prevents the tool state from becoming ready, and the boot state does not proceed to the three-Damiao enable stage.
- The observed `servo offline + Damiao not enabled` combination therefore points first to Huaner initialization, query transmission, response reception or parsing rather than to CAN/Damiao enable logic.
- Live static inspection confirms USART6 is initialized as 9600 8N1 TX/RX before `ArmInit`, both DMA handles are linked, all three USART6/DMA IRQ handlers call HAL, and only `HSLServoInit` occupies an asynchronous callback slot. Callback-table exhaustion is therefore not a credible cause in the current image.
- `g_hsl_servo_debug.initialized=0` and `online=0` represent different layers: initialization zero means config/self-test/DMA/callback registration failed, while initialized one plus online zero means no fresh valid position reply was parsed.
- The current full-arm path uses a two-ID position request, unlike the earlier isolated ID1 sweep. The exact dual-ID controller-board response remains the highest-value protocol comparison if initialization is already one.
- The provided ESP32 reference emits the same dual-ID request and expects `num * 3 + 5 = 11` response bytes for two servos, so the current request/length model is consistent with that known reference. Runtime counters are still required to distinguish no RX from malformed RX.

## 2026-08-11 - Phase 42 current hardware baseline

- Both physical servos are now confirmed online with separate ID1/ID2 position queries, proving the USART6 controller-board wiring, 9600 baud and basic `0x15` request/reply path work on hardware.
- The current image remains intentionally feedback-only because `HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY=1`; restoring the arm requires changing only the application selector, not re-registering a second UART driver.
- ID2 calibration is now default/open `550`, close `630`, and its send-path software range is `550..630`.
- The complete six-page PDF confirms `55 55` framing, 9600 baud, no CRC, and `Length = parameter_count + 2`; every 16-bit time, position and voltage value is little-endian.
- `CMD_SERVO_MOVE=0x03` uses parameters `count,time_l,time_h,(id,pos_l,pos_h)*count`, so its length field is `count*3+5` and total frame bytes are `count*3+7`.
- `CMD_GET_BATTERY_VOLTAGE=0x0F` is a four-byte request `55 55 02 0F`; its six-byte reply is `55 55 04 0F mv_l mv_h`.
- `CMD_MULT_SERVO_POS_READ=0x15` uses request length `count+3`; its reply uses length `count*3+3` and entries `(id,pos_l,pos_h)`.
- Controller-originated `0x06/0x07/0x08` packets are action-group status reports. The arm does not use action groups, so these packets must not be treated as position/voltage replies and must not desynchronize later `55 55` frames.
- The USART6 transport is DMA-driven with static module-owned TX/RX buffers. ISR callbacks only publish `volatile` completion/error flags and the received length; all parsing, abort/recovery and transaction changes remain in the 1 kHz task context. STM32F407 has no data-cache maintenance requirement for these buffers.
- Full-arm polling now alternates protocol-valid single-ID `0x15` requests at half of the configured two-servo period, so each ID still refreshes near 50 ms while timeout/frame errors remain scoped to only the requested servo.
- Single- and dual-servo `0x03` commands now share one count-driven frame builder. Its deterministic self-test uses the PDF's exact one-servo 1000 ms and two-servo 800 ms examples.
- Final Keil MAP proves the runtime switch is effective: arm initialization/task, Damiao control, Huaner frame builder/poller and Fruit observer are linked, while chassis and INS initialization sections are linker-removed.
