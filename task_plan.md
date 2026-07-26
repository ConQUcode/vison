# Task Plan: Three-Motor Arm Calibration And 3DOF Kinematics

## Goal

Use fast power-on single-reference homing for M2006/M3508, retain a documented maintenance full scan for mechanical changes, and apply the measured q2/q3 endpoint mapping to the existing 3DOF FK/IK.

## Phases

- [complete] Reconcile the existing homing implementation with the new calibration and kinematics contracts.
- [complete] Implement sequential power-on single-reference homing with stall, travel, timeout, offline, abort, and delayed-zero handling.
- [complete] Retain maintenance full scanning and measured endpoint mapping; keep optional GM6020 front-direction teaching.
- [complete] Implement wrist-center 3DOF FK/IK and Watch request/result surfaces without motor motion.
- [complete] Run GCC and Keil builds, inspect warnings, and verify all Watch-visible state paths.

## Constraints

- Preserve unrelated dirty worktree changes.
- GM6020 stays registered on CAN1 and disabled; it is only read for absolute-angle front teaching.
- After M3508 and M2006 feedback remain online for 500 ms, normal homing automatically finds the M2006 reference stop and then the M3508 reference stop.
- A hard-stop stall requires current, low speed, online feedback, startup masking, and persistence.
- Measured endpoint constants are compiled into firmware; a maintenance full scan refreshes scale values in RAM and exposes new spans for source/Flash persistence later.
- IK only computes results and never enables or commands a motor.
- No PWM servo operation in the active path.
- Normal calibration requires no Watch writes. Only `g_arm_state`, `g_arm_calibration`, and optional emergency stop `g_arm_homing_abort` remain externally visible.

## Errors Encountered

- `cmd` parsing rejected several multi-pattern `rg/findstr` commands with quoted spaces; repeated the inspections with simple literal patterns.
- Keil is not on PATH and PowerShell startup fails with `8009001d`; the final build used `cmd`, the explicit `C:\Keil_v5\UV4\UV4.exe` path, and a project-local `TEMP/TMP` directory.
- The first Keil invocation returned exit code zero despite a temporary-file failure. Final verification therefore used the generated build log, which reports actual compilation, linking, hex generation, and zero errors/warnings.
- MATLAB `-batch` exited with host status `0xc0000409`; formula parity was instead checked from the script source and independent FK/IK round-trip calculations.
