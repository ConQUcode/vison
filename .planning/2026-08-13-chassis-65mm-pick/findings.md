# Findings

- `APP_MODE_ARM` currently initializes only the arm path; chassis headers,
  `ChassisInit`, `INS_Init`, `INS_Task`, and `ChassisTask` are gated behind
  `APP_MODE_CHASSIS_ONE_METER`.
- The current chassis module automatically runs its legacy 1 m / right-turn
  loop after initialization, so it cannot be enabled unchanged in arm mode.
- The chassis already has encoder odometry and closed-loop straight driving,
  but the target distance was originally a compile-time constant.
- `ChassisRunStraight` originally stopped when remaining distance was within
  the global 10 mm tolerance; the one-shot pre-pick move now uses a dedicated
  3 mm tolerance.
- The FreeRTOS IMU and chassis threads already call `AppImuTask` and
  `AppChassisTask` at 1 kHz in every app mode; only application-level compile
  gates currently turn those calls into no-ops.
- `AppMotorControlTask` already calls `DJIMotorControl()` in arm mode, so no
  motor-task ownership change is required.
- One-shot completion must branch after the first straight stop-settle state,
  before the legacy wait/turn sequence.
- `ChassisInitOneShotStraight` now owns its runtime target/tolerance and enters
  `WAIT_IMU` independently of the legacy auto-test enable macro.
- The one-shot path transitions from the first stop-settle state to latched
  `DONE`; the legacy initializer still transitions to the wait/right-turn loop.
- In arm mode, both INS and chassis periodic tasks now run. The arm scheduler
  requires chassis `DONE` and host `ready`, while any chassis fault latches the
  scheduler in `FAILED`.
- Hardware testing showed the original logical-forward wheel signs drove the
  chassis physically backward. Both command signs and encoder feedback signs
  must be reversed together so physical forward remains positive odometry and
  satisfies the existing direction check.
- After the wheel-coordinate correction, hardware testing showed the IMU
  heading loop corrected in the opposite direction. The unified IMU Yaw/Gyro
  sign must also change from +1 to -1 so heading feedback matches the new
  logical wheel-yaw coordinate; the legacy -90 degree target remains a physical
  right turn in that coordinate.
- For close-contact relief, exhausting all four 10-position retreat attempts
  now enters `ARM_GRIPPER_FORCED_HELD`, preserves the stall fields, and
  increments `gripper_forced_held_count`, but does not latch a tool fault. The
  forced-held state completes the grip so pick/place continues. Feedback,
  transport, initialization, and non-contact timeout failures remain fatal.
- The requested finite field test is now three moves and three pick/place
  operations: 585 mm then point 1, 500 mm then point 2, 500 mm then point 1.
  After the third placement the scheduler latches DONE and does not start a
  fourth chassis move.
- Contact relief now allows four attempts (up to 40 position units total).
  Exhausting four contact retreats enters `FORCED_HELD` and continues; actual
  tool communication, feedback, initialization, or timeout failures remain
  fatal.
