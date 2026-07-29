# Findings

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
