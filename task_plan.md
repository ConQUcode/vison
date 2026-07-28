# Task Plan: 3DOF Arm Linear Motion

## Goal

Complete the existing GM6020/M3508/M2006 arm with separated FK/IK, safe path preflight, quintic joint staging, Cartesian linear interpolation, automatic three-point cycling, Watch telemetry, and fail-safe shutdown.

## Phases

- [completed] Extract the confirmed geometry and analytic FK/IK into `arm_kinematics` without changing motor initialization or tuning.
- [completed] Add `arm_trajectory` with boot delay, hard-stop exit corridor, path preflight, linear motion, settling, holding, cycling, and fault states.
- [completed] Split one-shot PID reset from per-cycle reference updates and integrate the trajectory task into `ArmTask()`.
- [completed] Add the new sources to Keil and synchronize the MATLAB model/test path.
- [completed] Run source checks, Keil build, numerical path checks, and document the Watch acceptance fields.

## Locked Decisions

- Geometry: shoulder `(-29,-7.6,34) mm`, `L1=150 mm`, `L2=179 mm`.
- Reference joints: `[0,180,-180] deg`.
- Automatic region: q1 `[-30,30]`, q2 `[125,150]`, q3 `[-155,-135]` deg.
- Boot delay 3 s; staging corridor S0-S1-S2-S3; continuous P1-P2-P3-P1 cycle.
- Cartesian peak speed 20 mm/s, 2 mm preflight spacing, 5 ms IK update, 1 ms motor reference update.
- Existing motor registration, CAN IDs, PID values, homing speeds, stall thresholds, and current-loop settings are not retuned.

## Errors Encountered

- PowerShell startup fails on this host; use `cmd.exe` for inspection and builds.
- Keil requires project-local `TEMP/TMP`; verify the build log rather than relying only on process exit status.
- MATLAB `-batch` previously exited with `0xc0000409`; use source checks and independent numeric checks if it remains unavailable.
