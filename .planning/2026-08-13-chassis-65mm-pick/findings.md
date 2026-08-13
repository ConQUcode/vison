# Findings

- `APP_MODE_ARM` currently initializes only the arm path; chassis headers,
  `ChassisInit`, `INS_Init`, `INS_Task`, and `ChassisTask` are gated behind
  `APP_MODE_CHASSIS_ONE_METER`.
- The current chassis module automatically runs its legacy 1 m / right-turn
  loop after initialization, so it cannot be enabled unchanged in arm mode.
- The chassis already has encoder odometry and closed-loop straight driving,
  but the target distance is currently a compile-time constant.
