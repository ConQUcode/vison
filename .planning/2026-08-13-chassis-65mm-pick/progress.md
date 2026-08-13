# Progress

- 2026-08-13: Inspected application mode gating and found chassis is not active
  in `APP_MODE_ARM`.
- 2026-08-13: Confirmed a dedicated one-shot chassis API is required before
  integrating the 65 mm pre-pick step.
- 2026-08-13: Chosen design: initialize chassis in one-shot mode with a runtime
  distance and tolerance, finish in a latched DONE state, then gate the arm
  scheduler on that state.
- 2026-08-13: Added one-shot straight initialization, runtime distance and
  tolerance, completion/fault getters, and a latched DONE state while retaining
  the legacy chassis-only loop.
- 2026-08-13: Arm mode now initializes INS plus a 0.065 m one-shot chassis move,
  runs the existing IMU/chassis tasks, and blocks the first pick until the
  chassis has stopped successfully. Chassis faults block all picking.
- 2026-08-13: Static verification passed: `git diff --check` returned clean;
  symbol searches confirmed the 65 mm target, 3 mm tolerance, latched DONE,
  fault gate, arm-mode task gates, FreeRTOS callers, and retained legacy loop.
  Per request, no compile, flash, or hardware test was run.
- 2026-08-13: Corrected the requested pre-pick travel from 65 mm to 650 mm;
  `APP_ARM_PRE_PICK_CHASSIS_DISTANCE_M` is now `0.650f`. The existing 15 s
  timeout and 1.3 m excess-distance guard remain compatible.
- 2026-08-13: Corrected physical forward direction by reversing both wheel
  command signs and their matching feedback signs. This preserves positive
  odometry and the straight-motion direction guard for the 650 mm move.
- 2026-08-13: Corrected the resulting reversed IMU heading feedback by changing
  `CHASSIS_IMU_YAW_SIGN` from `+1.0f` to `-1.0f`. This applies consistently to
  YawTotalAngle and Gyro Z without changing heading PID gains.
- 2026-08-13: Changed the five-retreat contact outcome to the dedicated
  `FORCED_HELD` state, accepted as a completed grip; Watch retains the current
  state plus cumulative `gripper_forced_held_count`, and the scheduler proceeds
  to post-grip dwell and placement instead of locking the flow FAILED.
- 2026-08-13: Added repeatable one-shot chassis segments and changed the test
  scheduler to the finite sequence 585 mm/point 1, 500 mm/point 2,
  500 mm/point 1, then DONE. Each segment still requires stop confirmation.
- 2026-08-13: Reduced maximum contact-relief attempts from five to four while
  retaining `FORCED_HELD` completion and its cumulative Watch record.
- 2026-08-13: Final static checks passed: no stale five-attempt descriptions,
  no conflict markers, and `git diff --check` was clean. No build, flash, or
  hardware test was performed.
