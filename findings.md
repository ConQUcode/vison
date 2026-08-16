# Findings

## Phase 21 BD left observation point

- 用户确认的新目标是工具中心`[0,57,210] mm`、世界绝对俯仰`-30deg`，方向仍朝左，因此底座预对准保持`q1=+90deg`。
- 当前默认固件继续保持打点模式；本轮只保存BD参数，不擅自切回BD运动测试。
- 必须用实时`ArmInverseKinematicsToolCenterAll()`和`ArmCartesianAppendToolCenterSegment()`等价约束检查底座预转后到目标点的整段轨迹，尤其关注工具径向投影大于目标Y而导致的径向过零与分支连续性。
- 实时`app_config.h`仍为旧BD参数`[0,150,280] mm/-50deg`，但底座预对准已是朝左`q1=+90deg`；默认模式已确认是`APP_MODE_ARM_TEACH_POINT`。
- 打点模式的应用任务只更新坐标Watch，达妙侧只轮询低频失能反馈，不发送周期控制帧，因此保存BD参数不会触发机械臂动作。
- 实时笛卡尔预检在1mm级采样上调用全候选工具中心逆解，并通过动态规划处理工具轴心靠近或跨过底座中心时的正/负径向分支；同时检查工作区、FK误差、软限位、自动安全和相邻采样步长。
- BD状态机在正常HOME后先提交底座朝左的关节命令；完成后才从真实反馈姿态提交工具中心直线命令，因此本轮参数变更无需改状态机。
- 目标工具中心命令显式携带世界绝对俯仰，速度仍独立使用`APP_ARM_BD_OBSERVATION_SPEED_MM_S=150mm/s`。
- 实时工具中心逆解会对方位角与其180deg反向都反算117mm工具偏移，再由基础IK按真实q1分支筛选并用工具中心FK复核误差；新点不能用单一`atan2`分支近似。
- 仓库已有`tools/arm_path_replay`，直接复用实时运动学/安全代码并实现多候选连续路径求解，但当前只覆盖AC镜像和后方释放路径，没有BD观察点用例。
- 当前关节边界为q1自动区`[-90,90]deg`、q2`[3,180]deg`、q3`[-190,-35]deg`；BD目标必须保持q1在左向`+90deg`分支并单独验证ID1相对俯仰。
- 回放工具与固件的相邻采样连续性阈值一致：q1不超过5deg、q2/q3不超过2deg；实时ID1相对俯仰范围为`[-90,92.4]deg`。
- BD底座预转命令保持HOME的q2/q3并只改q1，故理论起始关节为`[90,90,-60]deg`；该关节命令同时把工具世界绝对俯仰调整到目标`-30deg`，随后才开始工具中心直线段。
- 临时BD回放首次通过：起点工具中心约`[0,326.492,133.500]mm`，1mm直线采样共282点（含起点），终点选中`q=[90.000,168.151,-47.928]deg`，q1终点保持左向`+90deg`。
- 动态规划回溯的选中路径范围：q1=`90.000..90.001deg`（浮点误差，始终左向），q2=`90.000..168.151deg`，q3=`-60.000..-42.046deg`；最大相邻步长分别为`0.001/0.318/0.176deg`。
- 选中路径ID1相对俯仰为`-66.079..-0.070deg`，舵机位置为`500.291..775.327`，均在实时限制内；完整BD直线路径通过。
- 临时回放源码已完全撤销，专用BD CSV已删除；正式配置只修改Y/Z/绝对俯仰，X、左向q1与150mm/s速度保持不变。
- 正式配置复核为`q1=+90deg`、`[0,57,210]mm`、`-30deg`、`150mm/s`；默认模式仍为`APP_MODE_ARM_TEACH_POINT`。
- `git diff --check`通过，仅报告工作区既有LF/CRLF转换提示；ARM GCC严格检查覆盖运动学、轨迹、工具、应用和host-control条件编译后通过。
- 本轮首次Keil调用错误使用`Engineer/tmp/-r/-j1`，子进程长期无日志且产物未更新；已只终止该子进程，用户原有Keil图形进程未受影响。
- 当天成功打点重建的准确命令边界是`MDK-ARM/tmp`、`UV4.exe -cr Engineer.uvprojx -j0`，后续按此重试。
- 使用准确命令完成Keil全量重建：`0 Error(s), 0 Warning(s)`，`Code=82352`、`RO-data=896`、`RW-data=944`、`ZI-data=134484`；AXF/HEX/MAP均于21:55更新。

## Phase 22 Enable BD observation test

- 用户确认切换到已验证观察点的执行模式；默认`APP_MODE`由`APP_MODE_ARM_TEACH_POINT`改为`APP_MODE_ARM_BD_OBSERVATION_TEST`。
- 目标参数保持`q1=+90deg`、工具中心`[0,57,210]mm`、世界绝对俯仰`-30deg`和速度`150mm/s`，不修改状态机或运动约束。
- 模式/点位复核、`git diff --check`和ARM GCC严格检查通过；当前条件编译已覆盖BD状态机分支。
- Keil全量重建通过：`0 Error(s), 0 Warning(s)`，`Code=93860`、`RO-data=896`、`RW-data=932`、`ZI-data=134480`；AXF/HEX/MAP于22:01更新。

## Phase 23 BD coordinated staging path

- 当前第一段只将q1从HOME转到`+90deg`，q2/q3保持HOME；在目标绝对俯仰`-30deg`下工具中心半径约326.5mm，底座旋转末端Y会超过260mm。
- 用户要求底座与小臂配合避障，约束对象是第一段实际工具中心轨迹的Y坐标，不只是过渡终点坐标。
- 优先保留现有两段状态机：第一段同步提交q1和安全过渡q2/q3/ID1，第二段继续使用工具中心直线轨迹进入最终观察点。
- `ArmTrajectoryMoveJointWithOptions()`会对第一段的q1/q2/q3做同一时间轴插值；显式ID1参数是固定的相对小臂俯仰，运行时世界绝对俯仰随q2/q3连续变化，三台达妙与ID1协同而不是先后动作。
- 第一段安全预检已逐样本覆盖关节软限位、自动区、ID1相对俯仰和工作区，但当前没有`Y<=260mm`这一BD专用障碍约束，必须由明确过渡参数和离线回归测试证明。
- 1deg全范围搜索选定BD过渡姿态`q=[90,120,-48]deg`，对应固定ID1相对俯仰`-18deg`；第一段理论工具中心最大Y为`231.470mm`，相对260mm限制留出约28.5mm余量。
- 过渡姿态工具中心约为`[0,225.643,174.610]mm`，到最终`[0,57,210]mm/-30deg`的第二段为Y单调减小的侧向直线，仍需用实时IK等价回放确认关节分支连续。
- 正式回放通过：第一段`q=[0,90,-60] -> [90,120,-48]deg`的最大`|Y|=231.470mm`；第二段最终解为`q=[90.000,168.151,-47.928]deg`，底座始终保持左向且完整IK候选连续。
- 原有AC左右镜像接近/推进、左/右放置回正和前后绕行回放继续全部通过；ARM GCC严格检查与`git diff --check`通过。
- Keil ArmCC 5.06u7全量重建通过：`0 Error(s), 0 Warning(s)`，`Code=93880`、`RO-data=896`、`RW-data=932`、`ZI-data=134480`；AXF/HEX/MAP于22:20更新。
- 本轮没有烧录或执行机械臂实机动作；`231.470mm`是命令轨迹的同源运动学回放峰值，首次实机仍需观察`g_app_arm_bd_observation_debug.actual_center_mm[1]`并保留急停条件。

## Phase 24 BD right observation mirror

- 当前BD状态机和回放工具都从`APP_ARM_BD_OBSERVATION_BASE_Q1_DEG/Y_MM`取方向，没有写死左侧；右侧只需将`q1=+90deg`镜像为`-90deg`、目标Y从`+57mm`镜像为`-57mm`。
- `q2=120deg`、`q3=-48deg`、ID1相对小臂`-18deg`、最终世界俯仰`-30deg`、Z=210mm和150mm/s均保持不变；工作区保护与`|Y|<=260mm`检查本身与Y符号无关。
- 右侧同源回放通过：过渡关节`[-90,120,-48]deg`、过渡工具中心约`[0,-225.643,174.610]mm`，两段最大`|Y|=231.470mm`，最终解`[-90.000,168.151,-47.928]deg`。
- 原有AC左右镜像、放置和前后绕行回放继续通过；ARM GCC严格检查和`git diff --check`通过，当前模式文档已全部切换为BD右观察位。
- Keil ArmCC 5.06u7全量重建通过：`0 Error(s), 0 Warning(s)`，`Code=93880`、`RO-data=896`、`RW-data=932`、`ZI-data=134480`；AXF/HEX/MAP于22:31更新。
- 本轮没有烧录或执行实机机械臂动作，且没有修改`Engineer.uvoptx`。

## Phase 25 persist both BD observation points

- 当前左右观察点几何已经分别通过，但配置文件只保存了当前右侧符号；为避免以后切侧覆盖历史点，应显式保存LEFT/RIGHT的q1和工具中心XYZ，并由一个ACTIVE_SIDE宏映射到现有状态机接口。
- 两侧共享`q2=120deg`、`q3=-48deg`、最终俯仰`-30deg`、速度150mm/s和`|Y|<=260mm`；右侧X/Z从左侧别名派生、Y和q1显式取负，保证镜像关系不会漂移。
- 回放应独立验证左右两个配置点，而不是只验证ACTIVE_SIDE；这样当前上传右侧固件仍同时保留左侧路径回归边界。
- 正式配置已保存LEFT=`q1=+90deg/[0,+57,210]mm`与RIGHT=`q1=-90deg/[0,-57,210]mm`，右侧q1/Y/X/Z由左侧镜像宏派生；`APP_ARM_BD_OBSERVATION_ACTIVE_SIDE`当前选择RIGHT。
- 双侧回放同次通过：两侧过渡q2/q3均为`120/-48deg`、ID1相对俯仰均为`-18deg`、最大`|Y|`均为`231.470mm`；最终关节解分别为`[+90.000,168.151,-47.928]deg`和`[-90.000,168.151,-47.928]deg`。
- ARM GCC严格检查和`git diff --check`通过；Keil ArmCC 5.06u7全量重建0错误0警告，`Code=93880`、`RO-data=896`、`RW-data=932`、`ZI-data=134480`，AXF/HEX/MAP于22:41更新。
- 当前固件上电只执行RIGHT，LEFT只是永久保存并参与回放；本轮没有烧录或执行机械臂实机动作，也没有修改`Engineer.uvoptx`。
- 实时`AppArmBdObservationTask()`第一段复制当前q反馈后只覆盖q1，并根据该姿态计算ID1相对角，确认Y超限来自目标构造而非电机反馈等待。
- 现有关节运动层已对同步关节插值执行软限位、自动工作区和工具俯仰检查；新增过渡姿态可继续复用这些保护，但`Y<=260mm`是本次额外应用约束，需要离线逐样本证明。

## Current test baseline

- Active mode: `APP_MODE_ARM_POSTURE_TEST`.
- Current path before this task: `[0,-340,-135] -> [0,-460,-135] mm`, absolute tool pitch `-10 deg`.
- ID2 default/open position is `450`; close target is `660`.
- At `Y=-480`, previous source-equivalent calculation gives `q2` above the configured `10 deg` lower limit.

## Constraints to verify

- Existing posture mode does not initialize or poll `AppArmFlow` and transitions from pitch stability directly to `HOLDING` without closing ID2.
- Existing A-left place profile is a private static object in `app_fruit_task.c`, so posture mode cannot reuse it directly. Add a public profile builder/getter backed by the same centralized A-region macros rather than duplicating numeric parameters in `app_runtime.c`.
- Shoulder-first boot must account for the elbow synchronous-belt coupling and must not declare the remaining axes/tool ready before the shoulder reaches HOME.

## Live full-init path

- Normal builds select `ARM_BOOT_MODE_FULL_INIT`, whose implementation is `ArmProcessBootSequence()` plus `ArmProcessAutoInit()`; the older `ArmProcessStartup()` return order is not the active normal path.
- Current `ArmInit()` calls `ArmToolInit()` immediately, so ID1/ID2 initialize in parallel with DM enable.
- Current boot waits for the tool to become ready before `ArmProcessAutoInit()`, and auto-init resolves HOME then sends one `ArmCommandPose()` for all three joints simultaneously.
- Required replacement order: enable/hold all DM axes for mechanical control; keep tool init deferred; logically move q2 to HOME first while holding q1/q3 (elbow motor may compensate synchronous-belt coupling); then move q3, then q1; only after all three arrive call `ArmToolInit()` and wait for ID1/ID2 before READY.

## Pick/place reuse boundary

- `app_arm_flow.c` currently compiles its implementation only for `APP_ARM_ENABLED`, which would produce undefined `AppArmFlow*` symbols if posture mode starts using it. Broaden implementation to `APP_ARM_ENABLED || APP_ARM_POSTURE_TEST_ENABLED`, not teach-point mode.
- Posture mode can close ID2 directly through `ArmToolSetGripper(ARM_GRIPPER_COMMAND_CLOSE)`, wait for `ArmToolGripperActionComplete()`, then start the shared place flow.
- Add a public `AppFruitGetPlaceProfile(area, side, out)` that copies a centralized immutable A-left/A-right profile. This keeps posture mode explicit about LEFT without duplicating profile numbers.

## Phase 16 single-side command boundary

- The live left/right posture-test state machine already contains the complete verified operation: side staging, Cartesian approach, Cartesian advance, ID2 close, the matching A-side place profile, release, and return-to-front.
- The reusable interface must remain non-blocking: `Start(side, now_ms)` accepts one complete LEFT or RIGHT operation, while the existing 1 ms application task calls `Poll(now_ms)` until `DONE` or `FAILED`.
- A successful single-side operation must stop at `DONE`; automatic LEFT/RIGHT switching belongs only to the current `app_runtime.c` test caller. This keeps the future host protocol bridge independent of coordinates, arm command IDs, gripper details, and place profiles.
- Keep `g_app_arm_posture_test_debug` as the public Watch object when moving ownership into the new module, so existing bench Watch layouts continue to work.
- The new source must be added exactly once to the Keil APPLICATION group and to `gcc_arm_check.cmd`; `Engineer.uvoptx` remains user-owned and must not be changed.
- The extracted implementation passes the current strict ARM GCC check, and the existing replay still passes both mirrored post-place staging transitions, both 141-sample side paths, and the rear-route cases. The refactor did not change any path macro or profile value.
- The final module owns no automatic side switching: after a matching A-left/A-right place flow returns DONE it increments the corresponding counters, publishes the single-operation DONE state, and waits for the caller to submit another side. Only the posture-test wrapper contains `next_side`.
- Keil project inspection found exactly one `FileName` entry for `app_arm_side_pick_place.c`; the source is also present once in `gcc_arm_check.cmd`. Existing `.uvoptx` changes were not touched.

## Phase 17 TIM8 MG995 test

- The live MCU is STM32F407IGHx in UFBGA176; the `.ioc` records an APB2 timer clock of 168 MHz.
- Required mapping is valid: TIM8_CH2 can use PI6 for the physical right MG995, and TIM8_CH3 can use PI7 for the physical left MG995.
- PI6 is currently only initialized as a pull-up GPIO input and has no application read found; PI7 is not assigned in the live `.ioc`. Replacing PI6/PI7 with AF3 TIM8 does not conflict with current source-level ownership, but PCB net ownership still depends on the physical board.
- The user-generated TIM8 setup mirrors the existing servo-style timer parameters: `PSC=167`, `ARR=19999`, giving a 1us tick and 50Hz frame at the recorded 168MHz APB2 timer clock.
- Core generated files already contain unrelated uncommitted changes. All TIM8 edits must be applied on top of the live versions without regenerating or reverting those files wholesale.
- Re-reading the live diff showed the user had already generated the complete TIM8 configuration after the first inspection: `htim8`, `MX_TIM8_Init`, PI6/PI7 AF3 setup, `main.c` initialization, and `.ioc` channel declarations are all present with the correct 50Hz parameters. These generated changes should be preserved as-is.
- TIM8 shares `TIM8_TRG_COM_TIM14_IRQn` with the HAL TIM14 timebase. The generated handler services both handles; TIM8 PWM does not enable update/CC interrupts, so it will not create a servo interrupt stream. The shared handler remains required for the 1kHz HAL tick.
- The remaining implementation is application ownership: a reusable MG995 PWM wrapper, a dedicated current mode, and explicit suppression of DM/DJI periodic control in that mode.
- Final current mode is `APP_MODE_MG995_TEST`. The linked `AppInit` references only `DWT_Init` and `Mg995ServoInit`; linked `AppImuTask`, `AppChassisTask`, `AppArmTask`, `AppMotorControlTask`, and `AppUsbTask` are each 2-byte no-op functions.
- Both startup targets are 1500us. The wrapper maps right to TIM8_CH2/PI6 and left to TIM8_CH3/PI7, validates TIM8 `PSC=167/ARR=19999`, writes both CCR values before enabling PWM, and stops CH2 again if CH3 fails to start.
- The target-local TEMP/TMP full rebuild completed with ArmCC 5.06u7 at 0 errors and 0 warnings. Fresh AXF/HEX/MAP and `mg995_servo.o` were produced; the map resolves `AppInit -> Mg995ServoInit -> HAL_TIM_PWM_Start` and `htim8`.

## Phase 18 chassis velocity interface

- The live differential-drive core already computes `left=vx-wz*track/2` and `right=vx+wz*track/2`, then routes wheel targets through M3508 speed and current loops.
- The public command model only accepts finite `RELATIVE_STRAIGHT` and `RELATIVE_TURN` commands; its busy rule prevents continuous velocity refresh while RUNNING.
- The generated USB protocol currently contains only FruitDetection, ACK, heartbeat and handshake. This phase must not edit the protocol or hash.
- A dedicated `ChassisSubmitVelocityCommand()` can share the existing RUNNING/STOPPING states while using a new command type. Newer velocity IDs must refresh an active velocity command instead of returning BUSY.
- For zero requested `wz` with nonzero `vx`, capture and retain the current IMU yaw and feed the existing heading PID correction into the differential kinematics. Direct nonzero `wz` remains authoritative; when it returns to zero, capture a new hold heading.
- A fixed refresh watchdog should transition to the existing ramped CANCELLED stopping path rather than latch a fault, while motor/IMU loss and emergency stop remain faults.
- Combined `vx/wz` must proportionally scale both wheel targets before the existing per-wheel safety clamp so commanded curvature is retained at the speed boundary.
- Current application mode is MG995-only, so this interface will compile but cannot move the chassis until a future combined/chassis mode initializes IMU, chassis task and DJI motor control.
- Existing `ChassisBeginCommand()` already captures segment/heading state, enables both motors and enters RUNNING; existing CANCELLED stopping ramps both linear and angular commands to zero before disabling motors. Velocity mode can reuse these ownership boundaries.
- The existing strict GCC helper does not include `APPLICATION/chassis/chassis.c`; phase 18 must add it so the new public API and state-machine branch are checked with `-Wall -Wextra -Wshadow -Werror` before the native Keil rebuild.
- Keep the existing state enum stable. Add `CHASSIS_COMMAND_BODY_VELOCITY` and distinguish the stream by command type while preserving `RUNNING/STOPPING/CANCELLED` semantics.
- The implemented API uses `command_id`, `vx_mm_s` and `wz_rad_s`; accepted updates require a strictly newer ID. Finite relative commands retain their old enum values and submission behavior.
- Initial safe bounds are `|vx|<=200 mm/s`, `|wz|<=0.8 rad/s`, with 0.5mm/s and 0.005rad/s zero deadbands and a 300ms refresh watchdog.
- The existing IMU heading PID is active only when normalized `wz==0` and `vx!=0`. Repeated straight refreshes retain one heading target; a direct turn disables hold, and the first later zero-wz translation captures the new current yaw.
- `ChassisSetBodyVelocityTargets()` now proportionally scales the left/right pair when either exceeds 0.35m/s. Existing finite straight, finite turn and CANCELLED ramp-down also use this helper, preserving curvature at saturation.
- Status and Watch now expose target/actual vx/wz, heading-hold activity, refresh/timeout/capture counters, command age source tick and wheel scaling. Actual wz is the sign-corrected IMU gyro Z rate; actual vx is the mean encoder wheel speed.
- Documentation must distinguish the new internal command boundary from protocol integration: no packet ID, callback or protocol hash was added, and current MG995-only mode still compiles `AppChassisTask` as a no-op.
- Final native Keil rebuild compiled the updated chassis.c with ArmCC 5.06u7 and linked at 0 errors/0 warnings. The map contains the new API and then dead-strips it because the active MG995-only image has no chassis caller; this is expected and confirms the current image cannot move the chassis.

## Sequential DM implementation details

- With coupling active, `ArmCommandSingleAxis()` builds a full logical pose from current feedback, changes only the requested logical joint, then calls `ArmCommandPose()`. This lets q2 move first while q1/q3 remain logically fixed and the elbow motor compensates the synchronous belt.
- `ArmStartAxisMotion()`/`ArmRunAxisMotion()` already provide feedback validation, wrong-direction detection, timeout and stable-arrival handling, so auto-init should reuse them for q2, q3 and q1 rather than hand-roll arrival logic.
- `HuanerServoTask()` is a safe no-op before `HuanerServoInit()` because it returns immediately when the driver runtime is not initialized. A deferred tool state can therefore remain `ARM_TOOL_INIT_DISABLED` until DM HOME completes.
- The existing auto-init Watch states can be reused without changing the public enum: `MOVE_AXIS/WAIT_AXIS` for q2, then `MOVE_SAFE/WAIT_SAFE` for q3 and q1. The `step` field distinguishes q2=1, q3=2, q1=3.
- In full-init mode, `ArmInit()` must call a state-only deferred initializer; `ArmProcessBootSequence()` must call the real `ArmToolInit()` exactly once after q1 finishes, then use `WAIT_TOOL` only for ID1/ID2 readiness.
- `app_config.h` already contains the correct `[0,-340,-135] -> [0,-480,-135]` path, `-10 deg` pitch and `100 mm/s`; its mode-description comment still incorrectly says posture mode never runs a pick/place flow.
- A source-equivalent 1 mm scan over all 141 points from `Y=-340` through `Y=-480` passed the live q2/q3 and ID1 relative-pitch limits. Endpoint is approximately `q=[90,12.946,-102.421] deg`, ID1 relative `54.633 deg`; q2 lower-limit margin is `2.946 deg`. The approach point has the smallest ID1 margin, about `5.186 deg`.
- `git diff --check` exits successfully; Git only reports the repository's existing LF-to-CRLF conversion warnings.

## Unified arm command ID root cause

- Posture test commands use `0xA11C0001..0003`, while `AppArmFlow` starts again at `0xA11B0001`.
- The arm mailbox accepts only IDs newer than `latest_received_command_id`; therefore the first A-left transfer command is rejected as `ARM_COMMAND_DUPLICATE` before entering the mailbox.
- Durable fix: one application-owned monotonic arm command ID allocator shared by posture test and pick/place flow. Independent per-module bases must be removed.
- Chassis command IDs are a separate command domain and do not belong in this allocator.
- Live arm application producers are limited to `app_runtime.c` posture mode and `app_arm_flow.c`; all other `APPLICATION` command IDs found are chassis-domain IDs.
- The shared allocator should be initialized once from `AppInit()`, remain idempotent if initialization is requested again, skip ID zero on wrap, and expose last-issued/count/wrap Watch fields.
- Posture mode should allocate each ID immediately before its first submission attempt instead of reserving three IDs during initialization.
- `AppArmFlowInit()` must no longer reset a private command sequence; resetting a producer-local sequence while the arm mailbox retains its latest ID recreates the same class of bug.
- The allocator will use a short PRIMASK critical section so future application producers in different tasks cannot issue the same ID concurrently.
- Posture debug IDs remain zero until their corresponding command reaches its first submit state; BUSY retries retain the already issued ID because the mailbox did not accept it.
- Keil `APPLICATION` group currently lists `app_runtime.c`, `app_fruit_task.c`, and `app_arm_flow.c`; the new allocator source must be inserted there. `Engineer.uvoptx` remains user-owned and untouched.
- The repository has no current generated `Engineer.lnp`; verification will use source/project XML checks and the user's existing no-compile constraint.
- The mailbox explicitly accepts any non-zero first command while `latest_received_command_id == 0u`, so the shared `0xA1100000u` seed does not fail merely because bit 31 is set. Subsequent comparisons use the intended signed modular difference.
- The long-term ownership rule must be documented: every firmware-internal arm producer uses `AppArmCommandIdNext()`, while chassis commands remain in their independent mailbox/ID domain.

## Phase 7 pitch and synchronized arm initialization

- The requested 5 deg move toward horizontal means changing the current posture-test absolute pitch from `-10 deg` to `-5 deg` while keeping `[0,-340,-135] -> [0,-480,-135] mm`.
- A source-equivalent 1 mm scan rejects `-5 deg` at the first approach point: required ID1 relative pitch is about `+90.834 deg` and servo position about `121.53`, beyond the configured `+90 deg` and position `125` lower limits.
- The mathematical full-path boundary is about `-5.708 deg`, already at zero ID1 margin. `-6 deg` passes but leaves only about `0.346 deg` at the approach point, so the requested exact 5 deg change is not safe to apply and the current `-10 deg` value should remain.
- Current full init is q2 -> q3 -> q1 -> tool. Calling the single-axis starter twice cannot create synchronized q2/q3 motion because the second call rebuilds a pose from feedback and replaces the first target.
- Synchronized q2/q3 HOME must submit one coupled pose containing held q1 plus both HOME targets, with per-axis stable-arrival and wrong-direction timers, per-axis hard-boundary checks, shared timeout, feedback/motor/CAN checks, then run the existing single-axis q1 stage and deferred ID1/ID2 initialization.
- User selected `-7 deg` after the `-5 deg` rejection. The 1 mm full-path scan passes: approach relative pitch is about `+88.462 deg` with servo position `131.41`; the minimum combined margin is about `1.538 deg`, and endpoint q2 margin against the then-current 10 deg limit is about `1.794 deg`.

## Phase 9 ID2 release dropout and lower-Z boundary

- Hardware Watch captured the release-open host command `0xA1100007` faulting with `ARM_COMMAND_NOT_READY`; `gripper_state=ARM_GRIPPER_FAULT`, `tool_error_code=ARM_TOOL_ERROR_SERVO_FEEDBACK`, while all DM motion remained `ARM_MOTION_HOLDING` with no motion fault.
- ID2 later recovered to feedback position 455 for target 450, but the tool and place-flow failure latches intentionally prevented release-clearance and base-return commands from running.
- Before the phase-9 patch, opening logic faulted immediately once ID2 became offline during `OPENING`; it had no action-local recovery window even when feedback later recovered near the safe-open target.
- The implemented fix waits for bounded feedback recovery during ID2 READY/OPEN motions, preserves the original action deadline, and only completes after fresh feedback proves arrival; it never treats lost feedback as successful release.
- Current source uses `ARM_TOOL_PITCH_DIRECTION=-1`. Earlier notes that mapped relative pitch `+88.462deg` to servo position `868.59` were directionally stale; the live formula yields about `131.41`, and the affected tuning/planning values have been corrected.
- The current `[0,-340,-135] -> [0,-480,-135] mm`, `-7 deg` path ends at about `q=[90,11.794,-103.080] deg`; with the old 10 deg q2 limit, lowering the full path first reaches q2 at about `z=-143.745 mm`.
- Lowering the normal q2 limit to 3 deg moves its mathematical boundary to about `z=-176.889 mm`, but the approach point reaches the ID1 relative-pitch `+90 deg` limit earlier at about `z=-162.466 mm`; therefore ID1 becomes the next limiting condition and about `-160 mm` is the practical software-margin recommendation.
- ID2 READY/OPEN recovery is intentionally action-local: the first stale-feedback sample starts a 500 ms window, a recovered fresh sample clears the loss timer and increments the recovery Watch count, and completion still requires confirmed arrival. The original 1000 ms total READY/OPEN deadline remains authoritative.

## Phase 11 ID1 limit and -5 deg / -150 mm path

- User confirmed that ID1 can accept control position 115. With the live negative direction mapping, position 115 corresponds to relative pitch `+92.4 deg`; changing only the position limit would still be rejected by the duplicate `+90 deg` relative-angle limit, so both limits and the self-test must stay synchronized.
- The requested `[0,-340,-150] -> [0,-480,-150] mm`, `-5 deg` path has its tightest ID1 point at the approach: relative pitch is about `+91.706 deg` and control position about `117.893`, leaving about `0.694 deg` and `2.893` position units of software margin.

## Phase 12 world-Y coordinate mirror

- The requested convention is `+X` forward, `+Y` physical left, `-Y` physical right and `+Z` upward. Point identities and physical routes stay fixed: A-left remains q1 `+90 deg`, A-right remains q1 `-90 deg`.
- A coordinate-only mirror must update the full conversion chain. Changing only target constants would reverse the physical motion because FK/IK, tool-center offset and application base aiming previously encoded the opposite Y sign.
- The active physical-left posture path is mirrored from `[0,-340,-150] -> [0,-480,-150]` to `[0,340,-150] -> [0,480,-150]`; its q2/q3/ID1 feasibility margins must remain numerically unchanged.
- Across all 141 samples, q2 minimum is about `7.881 deg`, q3 remains about `-106.155..-70.762 deg`, and ID1 relative pitch remains about `+60.964..+91.706 deg`; the path passes the updated q2/q3/ID1 limits.

## Phase 13 mirrored left/right alternating pick-place

- Requested active test order is left pick/place, then right pick/place, repeating indefinitely at the original chassis position.
- Left reference path remains `[0,340,-150] -> [0,480,-150] mm`; right must be generated only by negating Y, with the same X/Z, absolute pitch, speed, waits and gripper behavior.
- Place routing must follow the physical side explicitly: left uses the A-left positive-q1 profile and right uses the A-right negative-q1 profile. Side selection must not be inferred from stale prior state.
- The active posture state machine already contains approach, advance, grip and place states and currently terminates in `HOLDING`; a single state machine can loop by switching the explicit side after successful place completion and returning to base aiming.
- `AppFruitGetPlaceProfile(area, side, out)` already exposes both centralized A profiles, so posture mode can select the matching side without duplicating placement angles or inferring side from q1/Y later in the flow.
- Each cycle must clear `base_command_id`, `target_command_id`, `advance_command_id`, `active_command_id`, pitch-stability timing and prior submit/profile results before returning to `WAIT_READY`; otherwise the arm mailbox sees an already-completed command ID and the next side can skip or stall.
- The current `WAIT_PLACE` success path increments a legacy flow cycle count and enters terminal `HOLDING`. Phase 13 should instead count the completed side, switch the explicit side, rebuild mirrored targets/profile, and return to `WAIT_READY`; `FAILED` remains terminal.
- Posture initialization currently writes only the positive-Y target and loads only `APP_FRUIT_SIDE_LEFT`. A cycle-preparation helper should own side-to-sign mapping, profile loading and command/debug reset both at boot and after each successful place.
- Both A profiles are already exact q1 mirrors (`+90/+135/+180` versus `-90/-135/-180`) while sharing q2/q3, release pitch and timing, so no new right-side placement constants are needed.
- Live q1 limits are symmetric: joint/escape `-180..+180 deg`, automatic Cartesian IK `-90..+90 deg`, and base pre-aim uses an absolute `89.5 deg` clamp. The mirrored side therefore receives the same angular margin.
- Workspace safety is Y-sign independent: rear classification uses only X, front-barrier gating uses `abs(q1)` plus X and q2, and q2/q3/tool-pitch limits are shared. No right-side-only limit override is required; regression tests must prove the mirrored samples produce negated q1 and identical q2/q3/ID1 values.
- A completed place flow sets status `DONE` and active flow `NONE` only after the directed return-to-front command completes. The next mirrored cycle can safely begin through the existing public start path without reinitializing `AppArmFlow`.
- Existing failure semantics remain suitable for an indefinite loop: flow failure, host fault/ESTOP, command rejection, gripper fault or invalid profile all latch posture `FAILED` and must not advance the side/cycle counters.
- The current config still describes and names only one positive-Y cycle. Phase 13 should expose explicit left/right approach and advance Y macros, with each right macro defined as the unary negative of its left counterpart so symmetry is structural rather than maintained by duplicate literals.
- The existing host verifier scans only positive Y. It should solve both targets at every millimetre using mirrored seeds, check both against `ArmAutoPoseIsSafe`, and compare q1 signs plus q2/q3 and derived ID1 values within a tight tolerance.
- Implemented cycle preparation now owns the side-to-coordinate mapping, matching A profile, command-ID reset and pitch-stability reset; it is used both for the initial left cycle and every post-place side switch.
- The loop advances counters and side only after `APP_ARM_FLOW_DONE`; failed or invalid states never increment completion counts.
- Runtime review confirmed side switching occurs only after the place flow's return-to-front command completes. Side validity is checked before completion counters are updated; next-cycle profile failure still leaves the just-completed side accurately counted and then latches `FAILED`.
- The mirrored host replay now passes all 141 paired samples: q1 is sign-mirrored, q2/q3 and ID1 values match, both sides pass automatic workspace safety, and the shared worst-case values remain q2 `7.881 deg`, ID1 relative pitch `91.706 deg`, position `117.893`.
- Current user-facing documents still describe a one-shot A-left cycle ending in `HOLDING`; they must be updated to show the indefinite `left -> right -> left` loop and both mirrored coordinate paths.
- The electrical/lower-controller verification table also needed to state paired `+/-Y` sampling; it now records 141 samples per side and the mirrored `q1=+/-90 deg` endpoint result.
- Final current-mode search finds no remaining `单次抓放` or successful `HOLDING` description/state in the application source or maintained documents.
- Final structural audit confirms right approach/advance Y macros are defined directly as the negatives of the left macros, and the A-left/A-right placement endpoints remain the existing `+180/-180 deg` mirror profiles.

## Phase 14 right-side post-base-aim stop diagnosis

- Hardware result: the full left pick/place succeeds, but on the following right cycle the base rotates and the remaining arm motion never starts. This places the first suspected boundary at posture `WAIT_BASE_AIM -> SUBMIT_TARGET` or the tool-center command preflight immediately after submission.
- Previous mirrored replay used ideal mirrored seeds near `[+/-90,80,-90]`; it did not reproduce the actual q2/q3/tool state left by A-left release-clearance and return-to-front. Diagnosis must use the live post-place pose and the exact base-only pre-aim command semantics.
- The available Watch surface can distinguish the failure without guessing: posture `state/submit_result/command_state`, host `last_command_state/last_command_result`, and motion `workspace_safety_result`, preflight IK fields and reject reason are all retained.
- `AppArmPostureTestSubmitCenter()` reaches the public `ArmSubmitToolCenterCommand()` mailbox entry. A return of `ARM_COMMAND_OK` only proves queue acceptance; the command may still be rejected later when the arm task performs trajectory preflight, so both posture submit result and host last-command state/result are required.
- Tool-center commands are converted to generic `ARM_COMMAND_TYPE_CARTESIAN` before mailbox submission and dispatched from `arm.c`; the right-side stop can therefore be a later Cartesian preflight failure even when the posture state's immediate `submit_result` is zero/OK.
- Host status maps an aborted/error Cartesian trajectory to `last_command_state=FAULTED` and normally `last_command_result=ARM_COMMAND_PREFLIGHT_FAILED`; therefore a right-side preflight rejection should drive posture `AppArmPostureTestCommandFinished()` into `FAILED`, matching the observed permanent stop.
- If staging fails immediately, mailbox processing records `last_command_state=REJECTED` with the exact execution result (`PREFLIGHT_FAILED`, `NOT_READY`, `INVALID`, etc.); if staging starts and fails later, host status records `FAULTED`. The Watch state therefore identifies whether the arm never started versus aborted after starting.
- A-left place returns to front after release-clearance while preserving approximately `q2=120 deg`, `q3=-80 deg` and ID1 relative pitch `-45 deg`; its directed front rotation changes q1 only. The next posture base-aim command also changes q1 only, so the right Cartesian target starts near `q=[-89.5,120,-80]`, not the ideal replay seed `[-90,80,-90]`.
- This makes the implementation geometrically non-mirrored at the transition boundary even though the two target paths and static limits are mirrored. The most likely failure is straight-line tool-center preflight/continuity from the post-release pose to `[0,-340,-150]`, not rejection of the final right target itself.
- `ArmMoveLinearToolCenter()` first rejects an unsafe start pose/tool pitch, then checks target IK, then samples the complete straight-line tool-center segment with continuity and workspace checks. From q1 `-89.5 deg` the right target does not cross the front/rear X boundary, so no bypass route is inserted; the full post-release-to-approach move must pass as one straight segment.
- Source-equivalent reproduction with the actual transition seed confirms the target itself is valid: `[0,-340,-150]`, pitch `-5 deg` solves to `q=[-90.000,12.533,-70.762]` with negligible FK error. The generated start center from `q=[-89.5,120,-80]` is approximately `[2.015,-230.866,365.895]`.
- The direct segment fails at sample 1, center `[2.011,-231.073,364.918]`, before any motor motion. One IK candidate exists but none passes both pose/pitch safety and the configured continuity step from the post-release seed. This directly explains “base rotates, then arm does not move” as an immediate Cartesian preflight rejection.
- Detailed first-sample replay isolates the decisive check: candidate `q=[-89.501389,120.030922,-79.756546] deg` passes joint soft limits, automatic region, ID1 pitch (`position about 603.28`) and joint-step continuity, but fails the front-barrier workspace check because tool-center `X=2.011 mm > 2.0 mm` while `q2=120.031 deg > 120.0 deg` and the base still counts as front-facing.
- The current posture-test pre-aim in `app_runtime.c` copies all feedback joints and overwrites only q1. Formal pick flow already has the durable behavior in `AppArmFlowSubmitBaseAim()`: submit q1 together with staging `q2=80 deg`, `q3=-90 deg`, and ID1 relative pitch `-80 deg`.
- Therefore the long-term repair belongs at the posture-test pre-aim command boundary; weakening the front-barrier threshold, changing the right target, or special-casing the right side would hide the asymmetric transition instead of fixing it.

## Phase 15 durable posture pre-aim repair

- Preserve the current mirrored coordinates, `-5 deg` absolute pick pitch, place profiles, command-ID allocator and all workspace/soft-limit constants.
- Reuse the existing staging constants so the test and formal pick flow cannot silently diverge: q2 `80 deg`, q3 `-90 deg`, ID1 relative pitch `-80 deg`.
- The repaired transition must be verified from both real post-place seeds (`[+/-89.5,120,-80]`) into their corresponding staging pose and then through all 141 approach/advance samples.
- `Arm_Joint_Command_s` already carries `tool_relative_pitch_valid/tool_relative_pitch_deg`; `ArmTrajectoryMoveJointWithOptions()` validates the synchronized ID1 target over the complete joint path, so no direct servo command or safety bypass is needed.
- A shared staging-command builder in `app_arm_flow` is preferable to copying the atan2/clamp/staging-field logic into posture mode: formal pick and posture-test pick then consume one definition of q1 aiming, q2/q3 staging and ID1 relative pitch while retaining their own command IDs and state machines.
- The A-place profile returns q1 to `0 deg` after release clearance while keeping q2/q3 at `120/-80 deg`; the durable transition replay must therefore start from `[0,120,-80]`, not the already-rotated failure seed or an ideal HOME seed.
- Host verification should include both layers: sampled synchronized joint interpolation from the real post-place pose to `[+/-89.5,80,-90]` with relative ID1 `-80 deg`, followed by Cartesian continuity from that staging pose to `[0,+/-340,-150]` and onward to `[0,+/-480,-150]` at absolute pitch `-5 deg`.
- The extended host replay passes both mirrored staging paths and both Cartesian segments. Approach endpoints solve to `q=[+/-90,12.533,-70.762]`; advance endpoints solve to `q=[+/-90,7.881,-106.155]`, with the existing worst ID1 values unchanged (`relative 91.706 deg`, position `117.893`).
- Maintained docs already describe synchronized staging for formal `AppArmFlowStartPick()`, but the active posture-test loop needs an explicit statement that every side, including the side entered after place return, uses the same staging builder; this is the regression boundary that failed on hardware.
- Final static verification passes: all eight ARM/application translation units compile under the configured GCC warning-as-error check, full host replay passes, `git diff --check` has no whitespace errors, and posture mode contains no old `base_target_deg` path.

## Phase 16 reusable one-side pick-place command

- Future host semantics are one command with an explicit side: LEFT performs the full left approach/advance/grip/A-left place/return sequence; RIGHT performs the mirrored right sequence and A-right place/return.
- The API must be non-blocking. `Start(side, now_ms)` validates and accepts one task; the existing 1 ms arm application task owns `Poll(now_ms)` until `DONE/FAILED`. A blocking function would stall FreeRTOS servicing, CAN feedback and servo communication.
- Current posture-test auto alternation must become a caller of the same public API rather than remain embedded inside the one-side state machine. This preserves the current hardware test while making a later USB protocol bridge a thin command adapter.
- Busy submission must not reset command IDs, active side, flow state or profile. Invalid side and latched failure must return explicit results without moving hardware.

## Phase 19 upper-controller protocol sync

- The three user-updated generated files are `protocol.h`, `protocol.c`, and `PROTOCOL_DOC.md`; the new wire hash is `0x2588BA9A`.
- New inbound application messages are reliable `StateMachineCommand`, reliable `ArmTarget`, and non-reliable `VelocityCommand`; `ExecutionCallback` is the lower-controller outbound completion message.
- The generated FSM now sends heartbeat replies and reliable inbound ACKs itself. The existing strong heartbeat callback also sends a reply, so an unmodified replacement would produce duplicate heartbeat frames.
- The new configuration sets `CFG_REQUIRE_HANDSHAKE=0` and `CFG_STRICT_HEARTBEAT=0`, while the existing runtime hard-codes both as mandatory. The runtime must consume the generated macros instead of preserving the obsolete policy.
- `VelocityCommand.linear_x` is m/s and maps directly to the existing internal `vx_mm_s` after multiplying by 1000; `angular_z` already uses rad/s.
- `StateMachineCommand` maps task 0 to ID2 grip/open and task 1 to both mirrored MG995 camera axes look-down/look-up. The packet carries no application command ID, so the bridge must make repeated identical commands idempotent.
- `ArmTarget` explicitly describes camera-frame meters. Until fixed camera extrinsics and capture-pose association exist, executing it as a base-frame target would be unsafe; this phase only validates and records it.
- Phase 20 uses `P_B = T_B_E * T_E_C * P_C`. The live arm FK defines the wrist origin at the ID1 pitch axis and the tool center 117 mm farther along the absolute tool pitch; therefore the camera mount reference must explicitly select either the wrist/small-link frame or the tool-center/tool-pitch frame.
- The generated `ArmTarget` packet has no frame or capture ID. The transform API can enforce snapshot IDs and age, but the current bridge cannot prove image-to-pose association from the wire packet alone; transformed points remain observation-only until the protocol or capture trigger supplies that association.
- Phase 20 validation passed 29 native math checks, strict Cortex-M4 GCC syntax checks, and Keil ArmCC 5 full rebuilds in both host-control and restored MG995 modes. The host MAP confirms the live ArmTarget callback reaches the transform; no flash, calibration or physical motion was performed.
- A dedicated host-control app mode is required because the current full-arm mode owns an automatic fruit task and the active MG995 mode intentionally disables USB/chassis/arm. Sharing either mode would create actuator ownership conflicts.
- The final bridge maps camera commands to mirrored physical angles `-45 deg` down and `+45 deg` up, waits 500 ms after PWM submission, then sends the completed callback; MG995 has no position feedback, so this is a timed completion rather than measured arrival.
- Both active-mode variants link under ArmCC 5: host-control mode retains `UpperControllerBridgeInit/Task`, strong protocol callbacks and `ChassisSubmitVelocityCommand`; the final rebuilt image is restored to `APP_MODE_MG995_TEST`.
