# Findings

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
