# Findings

## Phase 51 staged upper-controller HOME

- Production full boot explicitly performs `q2+q3 synchronous HOME -> q1 HOME -> ArmToolInit(ID1/ID2)`. Coupling is enabled before the q2/q3 motion and the tool initialization is deferred until all three DM axes are home.
- The current task 6 reset state machine already cancels active motion first, but then submits one joint command containing q1/q2/q3 plus an ID1 relative-pitch target. This does not preserve the boot ordering and does not explicitly restore ID2.
- Phase 51 should retain the existing asynchronous command/host-status pattern, callback 6 semantics, reset failure latch, and no-heap behavior while splitting HOME into explicit stages.
- Normal joint trajectories already latch the current ID1 angle relative to the small link when `tool_relative_pitch_valid=0`. Therefore the q2/q3 and q1 stages keep the ID1 servo control position instead of compensating to hold world pitch.
- At the HOME joint pose, `ArmToolSmallLinkPitchFromJoint([0,110,-40])` is `-30deg`; commanding ID1 absolute pitch to this value restores relative pitch `0deg`/neutral position. ID2 should then use the same `GRIPPER_READY` action as the full boot sequence.
- Reset rejection diagnostics need the reset state and a completed-stage mask; the generic ArmTarget stage alone cannot identify whether staged HOME failed during cancel, q2/q3, q1, ID1, or ID2.
- Keil ArmCC 5.06u7 accepts the staged state machine with no warnings. Final image is Code=143924, RO-data=3660, RW-data=1432 and ZI-data=137560; the existing 132-byte rejection snapshot remains in normal `.bss`.

## Phase 50 HOME/reset and latched rejection diagnostics

- The HOME/reset implementation already had a complete cancel-then-HOME asynchronous state machine. The actual blockage was the temporary `UPPER_DEBUG_BLOCK_HOME_AFTER_ARM_FAILURE_ENABLE` branch in the protocol receive path.
- Re-enabling HOME without another change would erase `g_arm_target_debug.stage` in `UpperControllerClearTaskStateForReset()`, so current state and historical failure evidence must remain separate.
- `g_upper_arm_reject_diagnostic` now snapshots the ArmTarget stage, flow step and failure source, command/motion/tool faults, path preflight failure point, AC advance rejection details, and the public arm-host terminal status before live state is cleared.
- `flow_diagnostic_valid` prevents pre-flow failures such as AC advance rejection from presenting stale `g_app_arm_pick_place_test_debug` fields as current evidence. In that case the latched `advance_*` fields are authoritative.
- A failure of the reset cancel/HOME sequence also creates a snapshot with source `UPPER_ARM_REJECT_SOURCE_RESET_HOME` and records the submitted bridge command ID/result plus host terminal status.
- HOME/reset never clears the latched snapshot. A later failure replaces its detailed fields and increments `count`; bridge initialization is the only automatic clear.
- Keil MAP confirms the snapshot occupies 132 bytes in normal `.bss`. The complete Phase 50 image uses Code=143232, RO-data=3648, RW-data=1432 and ZI-data=137560 with no compiler or linker warnings.

## Phase 49 shared arm path preflight

- Platform is STM32F407 + Keil; the task is application-layer kinematics/path planning, not startup, RTOS, driver, or protocol transport.
- Core FK/IK is relatively contained and already enumerates candidates, filters automatic joint limits, and verifies tool-center FK round trips.
- Production path validation uses all-candidate dynamic programming, joint continuity, tool pitch, and workspace checks in `arm_trajectory.c`.
- AC advance fallback in `upper_controller_bridge.c` currently uses greedy point IK plus pitch checks only, so it can disagree with production trajectory preflight.
- HOST replay recompiles production kinematics but duplicates production path construction, safety predicates, continuity cost, and dynamic programming.
- Phase 49 preserves current Z/pitch/limits/speeds/protocol and all existing user changes; only the known AC advance-preflight mismatch may change behavior.
- Production segment planning performs two passes: dynamic programming stores only predecessor indices, then IK candidates are regenerated to populate the selected joint samples. The reusable workspace therefore needs predecessor/count/selected arrays but not every candidate pose.
- `ArmMoveLinearToolCenter()` owns route construction and execution state; the reusable Phase 49 unit can initially target one explicit tool-center segment, which is sufficient for each production route segment and AC approach/advance checks.
- Existing workspace helpers mutate `g_arm_motion_debug`; the shared planner must instead return diagnostics and let `arm_trajectory.c` translate them into legacy Watch fields.
- Existing production planner services ID1 once per sample in both passes. The shared planner needs an explicit optional service hook so firmware timing remains equivalent while HOST tests remain hardware-independent.
- The previous AC 30mm/near-clamp edits are now part of the repository baseline; Phase 49 starts with no pending changes in those four files and must not assume they are still an uncommitted patch.
- Candidate cost and preflight failure masks are only used by the old segment function, so they can move wholly into the shared planner when the production adapter replaces that function.
- Keil project, strict GCC script, and HOST replay build lists all require an explicit new `arm_path_planner.c` entry.
- Production adapter can map the shared planner result back to all legacy preflight Watch fields without changing trajectory execution, duration calculation, sample storage, or command state transitions.
- The resumed integration audit confirmed the shared advance API owns the full `staging FK -> staging-to-approach plan -> requested advance plan -> reachable-prefix quantization -> truncated replan` sequence. `AppArmFlowSelectReachablePickAdvance()` can therefore delegate this sequence without retaining any FK, segment, or fallback arithmetic.
- After that delegation, strict ARM GCC and the existing HOST replay retained the reported baseline endpoints and sample counts. New tests should call `ArmPathSelectReachableAdvance()` directly and exercise full success, intermediate fallback, 1 mm acceptance, 0 mm rejection and approach-failure diagnostics without reimplementing planner rules.
- A temporary production-API scan at the unchanged AC values (`q staging=[89.5,80,-90]deg`, `Z=-105mm`, pitch `-15deg`, request `30mm`, step `1mm`) found stable far-boundary vectors at `X=-160mm`: approach `Y=543mm` selects `29mm`, `Y=571mm` selects exactly `1mm`, and `Y=572mm` rejects at advance sample 1 with zero reachable prefix. Mirroring Y, advance sign and staging q1 supplies the right-side equivalents.
- Limit-contract tests can use the production `ArmAutoPoseIsSafe*`, `ArmJointPoseWithinSoftLimits` and tool-pitch helpers. Current exact ranges are normal q1 `+/-90deg`, AC q1 `+/-115deg`, q2 `0..180deg`, q3 `-190..-35deg`, all with `0.2deg` numeric tolerance; ID1 relative pitch is `-90..+92.4deg` and servo position is `115..875`.
- Final diagnostics retain both the actually executable truncated plan and the original requested-advance failure. Successful fallback keeps business rejection at NONE while Watch exposes the full 30mm request's limiting status/mask/sample/coordinate; `approach_failed` separately distinguishes staging-to-approach rejection.
- Final Keil ArmCC 5.06u7 rebuild compiled and linked `arm_path_planner.c` with `0 Error(s), 0 Warning(s)`. MAP confirms production trajectory and advance wrappers call the shared planner. Final image size is Code=142816, RO-data=3648, RW-data=1432, ZI-data=137444; `Engineer.uvoptx` SHA-256 remained `6ae31918e053173831440f0220423ff8d652ee822188d32c34f9ddc02a018fcb`.

## Phase 30 AC post-grip Y limit

- 当前AC放置从`APP_ARM_PLACE_STEP_SUBMIT_TRANSFER`开始，直接把抓取终点关节姿态联合插值到profile的`safe_q_deg`；现有状态机没有`430mm`专用Y限制。
- A区左右profile的安全姿态分别为`[+90,90,-100]deg`和`[-90,90,-100]deg`，后续才沿各自方向转到后方；问题发生在底座后转之前的第一段。
- `AppArmFlowSubmitJoint()`没有为该段提交新的ID1俯仰目标，因此候选路径必须按实际普通关节命令语义和现有工具状态逐采样计算夹爪中心，而不能只比较起终点Y。
- 普通放置过渡命令以当前三轴反馈为起点，`move_type=ARM_MOVE_LINEAR`，同时提交q1/q2/q3目标；该命令的`tool_relative_pitch_valid`保持0，后续需从`ArmExecuteJointCommand`和轨迹层确认ID1在插值期间的实际保持语义。
- 当前AC抓取终点宏为左右`Y=+420/-420mm`，抓取后停留500ms；现有profile的转移姿态q2/q3来自`APP_FRUIT_A_TRANSFER_*`，并非笛卡尔轨迹点。
- `app_fruit_task_config.h`内仍有业务任务表的`[0,+/-400,-100]mm, -90deg`旧A点；HOST/专项AC单侧抓放实际使用`app_config.h`的`[0,+/-420,-145]mm, -5deg`，本轮回放起点必须采用后者，不能混用旧业务点。
- 离线工具已有`staging_pose_safe()`、1deg关节插值采样、工具中心FK及左右镜像检查，可直接扩展为AC抓取终点到放置准备的永久`|Y|<=430mm`回归测试。
- 轨迹层`ArmTrajectoryMoveJointWithOptions()`会先解析当前/指定ID1相对俯仰，再分别检查起点、waypoint和终点的ID1可达性，同时通过`ArmWorkspaceJointPathSafe()`检查关节直线段；但它仍没有AC专用430mm限制。
- 现有关节命令原生支持一个`waypoint_q_deg`，可在一条命令中执行“起点->waypoint->safe_q”并保留现有命令完成/失败语义；若能找到满足约束的镜像waypoint，这是比新增多条状态更小的实现。
- `ArmCartesianResolveRelativeToolPitch()`明确规定普通关节动作锁存动作开始时的ID1相对小臂角，而非保持世界绝对俯仰；轨迹中绝对俯仰会随q2/q3变化。
- 当前抓取终点约`q=[+/-90,12.964,-88.139]deg`、绝对俯仰`-5deg`，对应锁存相对俯仰约`+73.9deg`；直达`safe_q=[+/-90,90,-100]deg`时工具绝对俯仰会大幅上抬，需按该相对角计算真实工具中心Y峰值。
- 修改前基线`tools/arm_path_replay/run.cmd`已通过：AC左右终点解仍为`[+/-90,12.964,-88.139]deg`，BD双侧和后方绕行回放均正常；后续新增检查可用该基线判断是否引入回归。
- 临时搜索器按真实起点和锁存相对俯仰复现：从`q=[90,12.964,-88.139]deg`直达`[90,90,-100]deg`时夹爪中心峰值`|Y|=479.161mm`，明确超过用户要求的430mm。
- q1保持当前侧`+/-90deg`并增加q2/q3 waypoint可消除外凸；粗网格找到1870个满足430mm的候选。最低峰值候选贴近q3上边界，因此正式参数需保留软件限位余量后再选。
- q3普通软件范围为`-190..-35deg`。已测简单候选中`q=[90,20,-50]deg`峰值`424.376mm`且离q3上限15deg；`[90,30,-60]deg`峰值`427.887mm`。前者更有Y余量，但仍继续比较q3=-45deg组以增加动态余量。
- `q=[+/-90,20,-45]deg`保留q3上限10deg余量，两段理论峰值均为起点`420.000mm`，相对430mm留10mm。
- 运行时`ArmCartesianInterpolateJointSamples()`严格在`start -> waypoint -> target`的相邻样本间做分段线性关节插值，不使用会越过waypoint的高阶关节曲线；现有`ArmWorkspaceJointPathSafe()`也对两段逐1deg预检，因此离线两段采样与实际规划几何一致。
- 正式实现已把A区profile首段改为单命令双段关节轨迹：`抓取终点 -> q=[+/-90,20,-45]deg -> safe_q=[+/-90,90,-100]deg`；普通关节命令仍锁存抓取完成时ID1相对俯仰。
- 永久回放检查通过，左右逐采样`maxAbsY=420.000mm < 430mm`，并同时验证左右工具中心严格镜像、关节/ID1/现有工作空间安全；BD双侧、下一次抓取过渡和后方绕行基线仍通过。
- `gcc_arm_check.cmd`已通过全部配置单元及显式HOST模式`app_runtime.c`检查；profile新字段只有A区左右两个静态初始化器，均已补齐，`git diff --check`无空白错误（仅现有CRLF提示）。
- README、HANDOFF、FRUIT_TASK_FLOW和TUNING_GUIDE原先只描述直达`safe_q`；本轮已同步为先经过`[+/-90,20,-45]deg`并明确`420/430mm`验证边界。
- 为避免未来点位或实际ID1反馈变化后只依赖离线常量，A区profile新增运行时Y上限；提交首段前按实际q反馈和ID1相对角逐1deg复核两段，超限或反馈无效时以`ARM_COMMAND_PREFLIGHT_FAILED`锁存失败，不开始运动。
- 最终验证：AC回放`maxAbsY=420.000mm/limit=430.0mm`，BD双侧、下一侧抓取和后方绕行回放全部通过；严格ARM GCC含HOST模式通过，`git diff --check`通过。按工程约束未启动Keil，避免改写当前脏的`Engineer.uvoptx`。

## Phase 31 AC endpoint and raised retract

- 实时配置仍为接近`[0,+/-380,-145]mm`、抓取终点`[0,+/-420,-145]mm`、抓取俯仰`-5deg`，阶段30 waypoint为`[+/-90,20,-45]deg`、运行时Y上限430mm。
- 用户本轮目标按上下文解释为：接近与终点Z都提高5mm到`-140mm`，终点Y恢复`+/-440mm`；接近Y保持`+/-380mm`，因此低速推进距离从40mm变为60mm。
- 抓后第一段应在向内收Y时把工具中心从`Z=-140mm`抬到约`-120mm`，且第一段不能先向更低Z下探；完整放置准备改为`|Y|<=440mm`并保持运行时实际反馈预检。
- 工作区仍包含大量既有修改，`Engineer.uvoptx`已脏；本轮继续只改AC参数、A区放置profile、回放和维护文档，不触碰无关文件。
- 新终点`[0,+440,-140]mm/-5deg`的IK为`q=[90,13.017,-92.631]deg`，普通关节动作锁存ID1相对角约`69.353deg`。
- 原waypoint`[90,20,-45]deg`在新相对俯仰下虽然Y向内，但Z会先下探到`-176.105mm`且waypoint只有`-168.376mm`，不符合抬高20mm要求。
- 0.1deg网格找到236个同时满足第一段Y单调向内、Z不下探、waypoint Z在`-120+/-0.5mm`和两段`|Y|<=440mm`的候选；当前最佳`[90,27.3,-62.7]deg`对应中心约`[0,340.525,-120.007]mm`，全程最低Z正好为起点`-140mm`。
- 邻近简单点对比：`[27,-63]deg`到`Z=-121.218mm`（抬18.782mm），`[27.5,-62.5]deg`到`Z=-119.201mm`（抬20.799mm）；采用搜索最佳`[27.3,-62.7]deg`，抬高`19.993mm`。
- 正式实现将把Z要求写入profile并按实际反馈运行时预检：第一段Z不能相对上一采样下降，waypoint相对起点的抬高量必须在`20+/-2mm`内；第二段继续沿用原工作空间安全和Y上限检查。
- 当前profile只有A左/A右两个静态初始化器，适合在现有`transfer_path_y_max_mm`旁新增抬高目标和容差字段；运行时预检集中在`AppArmFlowTransferPathWithinYLimit()`，可在不改底层全局轨迹器的前提下扩展为Y+Z专用检查。
- 正式参数已更新：接近点`[0,+/-380,-140]mm`、抓取终点`[0,+/-440,-140]mm`，waypoint`[+/-90,27.3,-62.7]deg`，Y上限440mm，抬高目标`20+/-2mm`。
- 永久回放通过：终点IK`[+/-90,13.017,-92.631]deg`；第一段Z单调从`-140`到`-120.007mm`，抬高`19.993mm`；两段`maxAbsY=440.000mm`，左右严格镜像。接近/60mm推进、下一侧准备、BD和后方绕行均继续通过。
- 严格ARM GCC全部配置单元及显式HOST模式通过，`git diff --check`通过（仅现有CRLF提示）。
- README、HANDOFF、FRUIT_TASK_FLOW、TUNING_GUIDE、PROJECT_OVERVIEW和电气验证表已同步到`+/-440/-140/60mm推进/[+/-90,27.3,-62.7]/440mm/抬高约20mm`，旧`420/-145/430/[20,-45]`表述已清理。
- A区profile仍被旧正式业务点`[0,+/-400,-100]mm/-90deg`共享；在最终确定运行时Z约束归属前必须回放该入口，避免新增`20+/-2mm`检查导致非AC流程被拒绝。
- 兼容性回放确认风险真实存在：旧正式A点起始`q=[90,32.859,-101.444]deg`、ID1相对角`-44.303deg`，若复用AC新waypoint会把工具中心从`Z=-100`降到`-162.483mm`，且Z非单调上升，因此公共profile会被AC专用运行时检查拒绝。
- 最终边界修正：`App_Arm_Place_Profile_s`增加可选waypoint和约束开关；`app_fruit_task.c`中的A左/A右公共profile保持两开关为0，`AppArmSidePickPlacePrepare()`仅在复制后的AC单侧profile中注入`[+/-90,27.3,-62.7]deg`、440mm上限和`20+/-2mm`抬高约束。
- 最终复核确认工程内只有A左/A右两份静态profile初始化器，新增字段均已显式初始化；公共A业务继续直接进入`safe_q`，AC任务才分两段执行waypoint。
- 收尾验证再次通过：AC抓后`waypointZ=-120.007mm/raiseZ=19.993mm/maxAbsY=440.000mm`，BD双侧、AC接近/60mm推进、下一侧准备和后方路径全部通过；严格ARM GCC含显式HOST模式通过，`git diff --check`无空白错误。
- 六份维护文档已明确上述AC副本作用域，并再次搜索确认没有有效的`420/-145/430/最后40mm/[20,-45]`旧参数残留。本轮未启动Keil、未烧录、未做实机运动验证，`Engineer.uvoptx`未被本轮命令改写。

## Phase 32 camera extrinsic persistence and AC hardware rejection

- 用户确认D435i螺丝到RGB光心偏移来自官方数据、XYZ轴方向正确，相机安装在ID1之后并随夹爪一起俯仰；Fusion参考点为ID1轴心。
- 当前固件使用`CAMERA_TARGET_REFERENCE_TOOL_CENTER`，且ID1轴心到夹爪中心沿工具+X为117mm，因此已确认的ID1轴到RGB光心平移`[36.944894,69.184360,48.192321]mm`应换算为工具中心到RGB光心`[-80.055106,69.184360,48.192321]mm`；旋转矩阵不变。
- 实机截图显示`APP_ARM_POSTURE_TEST_FAILED`、左侧active、完成计数0、`place_start_result=0/ACCEPTED`、主机`fault_code=0`、夹爪已进入终态，且停在放置首步附近；这排除放置profile启动失败和主机故障，指向首段运行时预检或关节命令提交拒绝。
- `AppArmFlowPoll()`先调用`AppArmFlowUpdateWatch()`再执行`AppArmFlowPollPlace()`；若首段同一拍失败，上层立即停止Poll，所以侧任务中的`arm_flow_status/place_step`副本会永久停留在失败前的RUNNING/SUBMIT_TRANSFER，不能据截图中的数值否定拒绝。
- 精确拒绝来源应看`g_app_arm_pick_place_test_debug.failure_source/fault`以及`transfer_path_y_check_passed/peak_abs_y/start_z/waypoint_z/z_raise`；截图中的`submit_result=0`是旧命令结果，不代表放置首段已提交成功。
- 正式边界调整为：公共A左/A右profile恢复无AC waypoint/无AC YZ约束的原始直接安全姿态过渡；`app_arm_side_pick_place`取得profile副本后，仅为AC命令覆盖`[+/-90,27.3,-62.7]deg`、440mm和20+/-2mm约束。
- 本轮不得修改全局工作空间边界或`Engineer.uvoptx`；实现范围限定为AC放置准备状态和`tools/arm_path_replay`验证。

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

## Phase 26 upper-controller chassis mode

- 现有`APP_MODE_HOST_CONTROL`已完整启用USB、协议runtime、`UpperControllerBridge`、INS和`ChassisTask`，并由1ms底盘任务持续执行；无需新增模式或重写底盘控制器。
- `VelocityCommand`协议ID为`0x14`，payload为两个float共8字节：`linear_x`单位m/s、`angular_z`单位rad/s；回调将linear_x乘1000后提交`ChassisSubmitVelocityCommand()`，每帧使用递增command_id刷新同一连续速度命令。
- 底盘接受范围为`|vx|<=200mm/s`和`|wz|<=0.80rad/s`；超范围帧由底盘接口返回INVALID。连续命令300ms不刷新会进入平滑停车并最终CANCELLED，因此上位机应以至少10Hz持续发送。
- 包只有在bridge已初始化、协议连接ready且link online时才执行；`CFG_STRICT_HEARTBEAT=0`，但USB链路仍必须在线。`wz=0`且`vx!=0`时固件捕获当前IMU航向并自动直行保持。
- HOST_CONTROL既有初始化还包含MG995和`ArmInit()`，机械臂会正常上电HOME，但不会运行BD观察状态机；本轮按已有模式语义保留，不改执行机构所有权。
- 默认模式已切到`APP_MODE_HOST_CONTROL`，未修改速度限幅、超时停车、AC参数或已保存的BD左右观察点。
- BD双侧/AC路径回放和ARM GCC严格检查通过；后者包含`app_runtime.c in host-control mode`专门检查，`git diff --check`无空白错误。
- Keil ArmCC 5.06u7全量重建`host_control_chassis_rebuild_20260816.log`为0错误0警告，`Code=124532`、`RO-data=3464`、`RW-data=1408`、`ZI-data=136432`，新AXF/HEX/MAP于23:01生成。
- 最终MAP确认`protocol_fsm_feed -> on_receive_VelocityCommand -> ChassisSubmitVelocityCommand`，并保留`UpperControllerBridgeInit/Task`；本轮未烧录，也未做真实上位机USB或底盘硬件动作测试。

## Phase 27 host chassis speed saturation

- 当前`ChassisVelocityCommandValid()`把超出`200mm/s`或`0.8rad/s`的任一有限字段判为INVALID，整包不进入底盘；用户观察与源码语义一致。
- 仅把线速度接口上限改为`1.0m/s`仍会被现有`0.35m/s`单轮上限二次比例缩小，因此HOST线/角上限、单轮上限和加速度必须配套调整。
- 选定`1.0m/s`线速度、`1.5rad/s`角速度和`1.3m/s`单轮上限；最大组合命令的理论单轮峰值为`1.0+1.5*0.32/2=1.24m/s`，低于单轮上限，不改变给定曲率。
- 有限超限输入应逐字段饱和，正负号保持；NaN/Inf、空指针和无效command_id仍必须拒绝。300ms失联停车、线/角零值死区和20mm/s停稳阈值不属于本轮提速范围。
- 实现后`ChassisVelocityCommandValid()`只验证指针、ID和有限数；提交路径保存原始请求，分别钳位vx/wz并统计总/线/角钳位次数，然后用钳位值刷新目标，超限包返回ACCEPTED。
- Phase 27通过BD/AC路径回放、ARM GCC严格检查、`git diff --check`和Keil ArmCC 5.06u7全量重建；`host_control_chassis_1ms_saturation_rebuild_20260817.log`为0错误0警告，AXF/HEX/MAP于01:14生成。

## Phase 28 upper-controller AC side command

- 新状态继续复用`StateMachineCommand`的两个业务字节，无需改协议结构：`task_id=2`，status 0/1/2分别为AC左、右、左右都抓；callback_id固定为2，status 1/0表示开始/完成。
- 当前桥在接收侧用`task_id>1`和`task_status>1`直接拒绝新状态；协议业务桥是首个缺口。
- 现有`AppArmSidePickPlaceStart/Poll`已封装一侧完整AC接近、推进、抓取、对应侧放置和回正，但真实实现只编译于ARM自动模式或POSTURE测试模式；HOST当前得到的是FAILED stub，也未初始化或轮询该状态机。
- 正确接法是HOST启用真实单侧状态机与`AppArmFlowInit()`，由机械臂应用任务唯一调用Poll；USB桥只负责Start、读取状态、status 2时在左侧DONE后启动右侧、以及发送一次开始/最终完成回调。
- 实现后旧任务仍按各自status上限校验，只有task_id 2允许status 2；相同运行中可靠重传计入duplicate，不会重启任务，不同离散命令计入busy。
- 双侧请求首侧受理后只发送一次执行中回调；左侧DONE后直接Start RIGHT，不发送中间完成，右侧DONE后才发送最终完成。FAILED/异常IDLE均结束桥任务并计失败，不伪报完成。
- BD/AC路径回放、HOST严格GCC检查和`git diff --check`通过；Keil全量重建`host_control_ac_side_command_rebuild_20260817.log`为0错误0警告，`Code=131044`、`RO-data=3600`、`RW-data=1432`、`ZI-data=137036`，新AXF/HEX/MAP于01:50生成。
- MAP确认`AppArmTask -> AppArmSidePickPlacePoll`、Bridge AC Start/Poll、单侧Start/GetStatus和ExecutionCallback发送链均进入最终镜像；未烧录、未做上位机或机械臂实机动作测试。
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
- Phase 29 protocol audit: the user-updated generated files define wire hash `0x740E426B` and remove `FruitDetection` (packet ID `0x10`, packet type, send/receive hooks and FSM dispatch).
- The new `ExecutionCallback` mapping is `0=gripper`, `1=camera servo`, `2=AC automatic pick`, `3=ArmTarget`; AC `task_id=2` remains `status 0=left`, `1=right`, `2=left then right`.
- `ArmTarget` still only validates, transforms and records camera coordinates. Because calibration/snapshot/execution inputs are incomplete, callback ID 3 is reserved and no false start/completion callback is emitted.
- Maintained firmware still referenced the deleted generated type through `protocol_runtime.c`, `fruit_usb_bridge.*`, `app_runtime.c`, `gcc_arm_check.cmd` and `Engineer.uvprojx`; Phase 29 removes that obsolete bridge without changing generated protocol files.
- Phase 29 verification passed: Keil ArmCC 5.06u7 rebuilt the HOST image at 02:58 with `Code=130740`, `RO-data=3600`, `RW-data=1436`, `ZI-data=136984`, `0 Error(s), 0 Warning(s)`; MAP contains the AC, ArmTarget and chassis command paths and contains no FruitDetection bridge symbols.
- Phase 33 changes only the AC copied-profile runtime Y threshold from 440 mm to 445 mm. The pick endpoints and nominal replay peak remain +/-440 mm and 440.000 mm; the added 5 mm is feedback tolerance, not a new target coordinate. Waypoint, Z constraints, public A profiles and global workspace limits are unchanged.
- Phase 34 initial ownership audit: in `APP_MODE_HOST_CONTROL`, `AppArmSidePickPlacePoll()` is called by `AppArmTask()` only. `AppUsbTask()` calls `UpperControllerBridgeTask()`, which starts pending AC work but does not Poll the arm flow. No second periodic arm-flow owner has been found yet.
- The intermittent base twitch must be checked against the rear placement boundary: the A-left/A-right profiles use directed base endpoints at `q1=+180/-180 deg`, while the trajectory and workspace code repeatedly normalizes q1 deltas with `ArmCartesianWrapTo180()`. Feedback/target representation at this exact equivalent-angle boundary is a higher-risk hypothesis than the new 445 mm AC Y threshold.
- Phase 34 sequencing audit: `AppArmFlowCommandFinished()` requires `host.last_command_id == active_command_id` plus `COMPLETED/OK`; it does not accept a stale previous completion. Place transfer, rotate-to-place, release pose, open, clearance and rotate-to-front are therefore serialized by command ID.
- The joint trajectory enters `SETTLING` after the reference curve ends and calls `ArmUpdateJointReference(final_q)` every arm cycle until feedback is stable. Application arrival error wraps q1 differences to +/-180. The remaining critical question is whether the low-level Damiao target path also treats +180 and -180 as equivalent; if it does not, an exact rear target at the representation boundary can produce a physical correction despite a near-zero application-level error.
- RTOS ownership is structured rather than concurrent: `ArmControlTask` is AboveNormal and runs `AppArmTask()` every 1 ms; `MotorControlTask` is High and runs `DMMotorControl()` with `vTaskDelayUntil(...,1ms)`. The arm task computes/writes references and the higher-priority motor task sends the latest stored references. No second q1 planner was found.
- `ArmUpdateJointReference()` validates the supplied q array and passes it unchanged to `ArmCommandPoseWithAxisSpeeds()`. Unlike arrival/error calculations, this boundary does not itself apply q1 wrap/continuity selection. The next audit target is the logical-q to Damiao-position conversion and whether the stored motor target can jump between equivalent +pi/-pi representations.
- The low-level mapping is linear and absolute: `ArmJointDegToMotorRadBase()` applies zero trim/direction without modulo, and `DMMotorSetPositionSpeed()` stores `position_ref_rad` unchanged. Therefore an upstream +180/-180 representation change would become a real near-2pi target jump. However the feedback conversion is also linear and no pi wrap has yet been found, so runtime evidence is required before naming boundary sign flip as the root cause.
- A source-confirmed target change occurs twice at the rear: release-pose and release-clearance commands start by copying all `arm->q_feedback_deg`, then replace only q2/q3. Their q1 targets therefore change from the prior planned rear endpoint to the instantaneous q1 feedback. This can remove/reapply base position error or capture load deflection and is currently the strongest source-level explanation for repeated small twitches.
## Phase 34 resumed evidence (2026-08-17)

- Re-read the live dirty checkout and confirmed Phase 34 remains diagnosis-only; no motion parameter or source behavior has been changed in this phase.
- `AppArmFlowSubmitJoint()` documents and implements that every unspecified joint is initialized from `arm->q_feedback_deg`. The release-pose submission (place steps 5/6) and release-clearance submission (place steps 10/11) both leave q1 unspecified, so each replaces the prior planned rear q1 hold target with the instantaneous base feedback.
- The place profile validator requires both `release_q_deg[q1]` and `release_clearance_q_deg[q1]` to equal `rotate_to_place_target_q1_deg`, yet those stored q1 values are not submitted in the two calls above. This mismatch strengthens the hypothesis that q1 target re-locking is unintended rather than a required profile behavior.
- The exact gripper-open completion path and the concrete left/right rear q1 profile values still need targeted source extraction before concluding root cause.

### Phase 34 root-cause conclusion

- AC uses the A-area rear routes exactly at the periodic boundary: LEFT `+135 -> +180 deg`, RIGHT `-135 -> -180 deg`; return routes are `+90 -> 0 deg` and `-90 -> 0 deg`.
- q1 arrival error is evaluated through `ArmCartesianWrapTo180()`. Combined with `ARM_LIMIT_TOLERANCE_DEG=0.2`, a physical feedback value such as `+180.1 deg` is legal and is considered only `-0.1 deg` from the `+180 deg` target.
- The next release-pose command copies this raw `+180.1 deg` feedback into both the command target and the trajectory start sample. `ArmCartesianInterpolateJointSamples()` applies an outer `ArmCartesianWrapTo180()` to every q1 reference, immediately converting `+180.1 deg` to `-179.9 deg`. The right side has the symmetric `-180.1 -> +179.9 deg` failure.
- `ArmUpdateJointReference() -> ArmCommandPoseWithAxisSpeeds() -> ArmSetJointCommandForPose() -> DMMotorSetPositionSpeed()` sends this as a linear absolute motor position; the lower layer has no matching periodic unwrapping. Therefore the numerical wrap becomes a real near-360-degree target discontinuity.
- The same raw-feedback q1 re-lock occurs again in release-clearance, explaining why one place cycle can twitch more than once. Boundary crossing depends on small overshoot/load/noise, explaining why the fault is intermittent.
- ID2 open is not completed on submit: it must reach `ARM_GRIPPER_OPEN` after valid feedback, empty TX pending state, arrived feedback and matching target position. It is therefore not the primary source of the premature-looking base motion.
- Source-level diagnosis is complete. Hardware confirmation can be obtained by capturing `place_step`, `q_feedback_deg[0]`, `trajectory_q_deg[0]`, `q_target_deg[0]` and `motor_command_rad[0]` at the twitch; no firmware behavior was changed in Phase 34.

## Phase 35 implementation

- User approved moving the rear placement yaw inward. A-area LEFT/RIGHT rear q1 targets are now exactly mirrored at `+178/-178deg`, retaining the existing `+/-135deg` directed waypoints and `+/-90 -> 0deg` front-return routes.
- Release-pose now explicitly submits `release_q_deg[q1]`; release-clearance explicitly submits `release_clearance_q_deg[q1]`. The profile validator already guarantees both values equal the selected rear target, so neither step can replace q1 with instantaneous feedback.
- This is intentionally a local application-flow fix. Global `WrapTo180`, q1 limits/tolerance, Damiao absolute-position conversion, AC Y/Z transfer constraints, speeds, ID1/ID2 behavior, BD observation, chassis and protocol paths are unchanged.
- Maintenance audit found four documents still describing the superseded `+/-180deg` and instantaneous-feedback hold behavior. PROJECT_OVERVIEW, TUNING_GUIDE, HANDOFF and FRUIT_TASK_FLOW were updated to the implemented `+/-178deg` explicit-profile hold semantics; unrelated values were left unchanged.
- Final validation passed: the complete path replay retained AC `maxAbsY=440.000mm` under the `445.0mm` runtime limit and `19.993mm` Z raise while BD, mirrored picks, post-place transitions and rear routes passed; all strict ARM GCC units plus explicit HOST-mode `app_runtime.c` compiled with `-Werror`; `git diff --check` found no whitespace errors. Remaining `180` documentation references are intentional boundary/soft-limit descriptions.

## Phase 36 closed-loop observation posture

- Phase 35 is now explicitly retained as the AC open-loop fallback; it must not be removed while the camera/host closed-loop protocol is still pending.
- The requested first closed-loop bring-up posture is interpreted as tool-center world pitch `-90deg` (vertical downward), base yaw `+90deg` (physical left), `X=0mm`, and preferred `Y=40mm`. Z remains to be selected by full kinematic and path replay rather than by point-only IK.
- Communication and camera target execution remain out of scope for this phase; the immediate deliverable is a firmware test mode that reaches and holds the selected left observation point.
- 2026-08-17 Phase 36 resumed: terminal execution is pinned to native `cmd.exe` because PowerShell still fails with `8009001d`; source remains on Windows paths and no Keil GUI will be opened during candidate search.
- The existing BD verifier entry points are `verify_bd_observation_side()` at line 485, `verify_bd_observation_transition()` at line 583, and `main()` at line 879 of `tools/arm_path_replay/arm_path_replay.c`; candidate selection will extend this source-equivalent path validation rather than use endpoint-only IK.
- The live observation runtime already performs the intended sequence: HOME `[0,90,-60]` -> synchronized staging `q1/q2/q3` with ID1 relative pitch -> tool-center linear move -> hold. Existing staging `q=[+90,120,-48]deg` requires ID1 relative pitch `-78deg` when the world tool pitch is `-90deg`, which is inside the configured ID1 range.
- Current permanent BD verification samples the HOME-to-staging joint interpolation, enforces `|Y|<=260mm`, samples the staging-to-target Cartesian segment, solves dynamic IK continuously, and verifies the final q1 branch. It needs temporary parameterization/metrics to compare candidate Z heights; the temporary scan will be removed after selecting the final point.
- First full-path scan with legacy staging `[90,120,-48]deg` rejected every `Z=180..320mm` candidate. The continuous IK branch disappears mid-segment around `Y=90..108mm`, `Z=142..157mm`; endpoint reachability is therefore not the deciding issue. A new staging q2/q3 must be searched under the unchanged joint, workspace, `|Y|<=260mm`, and ID1 limits.
- A broader grid over HOME-safe staging `q2=60..120deg`, `q3=-120..-40deg` still found no complete straight-line route to any `[0,40,Z]mm`, `Z=180..320mm`, pitch `-90deg` target. Endpoint candidate rejection must be isolated before changing geometry or adding a non-linear approach.
- Endpoint isolation proved `[0,40,Z]mm` is geometrically solvable by the three DM joints but not by ID1: the left-branch ID1 relative demand ranges from `-107.625deg` at Z=180 to `-130.401deg` at Z=320, below the configured and measured `-90deg` minimum. Raising Z worsens the ID1 demand at fixed Y=40; the safe solution must increase radial Y or relax vertical pitch, and the user explicitly requires vertical pitch.
- At pitch `-90deg` and the existing `|Y|<=260mm` observation envelope, the first endpoint barely enters ID1 range at `(Y,Z)=(150,180),(175,190),(210,200)mm`, each with only about `0.03..0.16deg` margin. No endpoint is safe for Z>=210 through Y=260, so a practical observation pose needs lower Z than the old 210mm point unless vertical pitch is relaxed.
- Endpoint combinations retaining about 10deg ID1 margin are approximately `(Y,Z)=(95,100),(105,110),(120,120),(140,130),(160,140),(190,150)mm`. These are the meaningful tradeoff frontier for the user's fixed vertical pitch; full HOME-to-staging-to-target validation will compare rounded candidates on this frontier.
- Rounded frontier targets all have at least one complete route, but an angle-margin-only staging score selects paths whose staging tool-center minimum Z ranges from `-64.6mm` to `52.3mm`. Because the user wants a raised observation approach and has already identified obstacle risks, candidate ranking must include the whole HOME-to-staging minimum Z, not only joint/ID1 margins.
- Final closed-loop observation test selection is left tool center `[0,+105,110]mm`, base `q1=+90deg`, pitch `-90deg`, with staging `[+90,90,-80]deg` and ID1 relative `-80deg`. The HOME-to-staging minimum tool Z is `82.056mm`; the full path reaches `max|Y|=256.050mm`, and the final solution is approximately `[90,125.436,-44.186]deg` with ID1 `-79.622deg` and minimum reported angular margin about `9.186deg`.
- The selected point is intentionally not Y=40: fixed Y40 requires ID1 below its real lower bound. The right point remains `[0,-105,110]mm/q1=-90deg` as a strict mirror, while only LEFT is active in the current test firmware source.

## Phase 37 corrected BD observation radius

- The user corrected the intended BD observation Y from about 40mm to about 400mm. With X=0, Z=110mm and pitch=-90deg, the endpoint is kinematically reachable at approximately `q=[+90,56.100,-112.631]deg`, requiring ID1 relative pitch about `-78.732deg`.
- `APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM` is BD replay-only; AC independently uses `APP_ARM_POSTURE_TEST_TRANSFER_PATH_Y_MAX_MM=445mm` in its side-pick profile and replay. Phase 37 must not route either limit into the other flow.
- Permanent replay with the corrected configuration passes both mirrors: BD final q is `[+/-90,56.100,-112.631]deg`, staging relative ID1 stays `-80deg`, and `max|Y|=400.000mm` under the BD-only 405mm limit. The same run still reports AC `max|Y|=440.000mm` under its independent 445mm limit.
- Phase 37 final audit found no stale `+105mm` or `256.050mm` BD values in source/maintenance docs. The current default remains the LEFT BD observation test; communication, camera execution, AC motion, chassis and protocol behavior were not changed.

## Phase 38 raised BD observation posture

- Directly changing only Z to 350mm while retaining Y=400mm/pitch=-90deg fails the permanent straight-line replay at approximately `[0,289.177,203.609]mm`: the arm solution makes the small-link absolute pitch cross 0deg, so ID1 would need slightly below its real `-90deg` relative limit. Continuing toward the endpoint would also worsen the wrist reach because a vertical 117mm tool offset raises the ID1 axis.
- Relaxing pitch to -60deg remains insufficient: the path reaches approximately `[0,389.483,328.526]mm`, where the small-link pitch rises to about +30deg and ID1 again needs below -90deg.
- The smallest tested practical adjustment is pitch=-45deg. It preserves the requested tool-center `[0,+/-400,350]mm` without shrinking Y, passes the whole mirrored HOME -> staging -> Cartesian route, and ends at `q=[+/-90,69.663,-139.562]deg`. Endpoint ID1 relative pitch is approximately `-74.225deg`, leaving about `15.775deg` to its -90deg lower limit.
- The BD-only replay peak remains `max|Y|=400.000mm` under 405mm. The same permanent run still passes AC `max|Y|=440.000mm` under its independent 445mm post-grip limit; no AC parameter or flow was changed.

## Phase 39 reduced BD observation Y

- The active left BD tool-center Y changed from +400mm to +200mm; the right target remains generated as a strict mirror and is therefore -200mm. Z=350mm, absolute pitch=-45deg, staging, speed and the BD-only 405mm replay limit are unchanged.
- Both complete mirrored paths pass. Final q is approximately `[+/-90,114.050,-96.794]deg`; endpoint ID1 relative pitch is about `-75.844deg`, leaving `14.156deg` to its -90deg lower limit.
- The whole-path `max|Y|` is `338.781mm`, larger than the 200mm endpoint because HOME-to-staging temporarily extends the tool farther sideways. It remains below the 405mm BD replay limit. AC independently remains `440/445mm` and was not modified.

## Phase 40 BD observation pitch -30deg

- The preferred absolute tool pitch of -30deg passes both complete mirrored BD routes at the unchanged `[0,+/-200,350]mm` targets; the -35deg fallback is not needed.
- Staging remains `[+/-90,90,-80]deg`, with ID1 relative pitch changing from -35deg to -20deg. Final q is approximately `[+/-90,120.249,-87.711]deg`; endpoint ID1 relative pitch is about `-57.960deg`, leaving `32.040deg` to its -90deg lower limit.
- Whole-path minimum Z is `102.373mm` and `max|Y|` becomes `357.375mm`, still below the unchanged 405mm BD-only replay limit. The independent AC fallback remains `440/445mm` and unchanged.

## Phase 41 downward-pitch boundary and final -53deg

- The user clarified that the desired direction is closer to vertical downward (-90deg), superseding the temporary Phase 40 interpretation. At `[0,+/-200,350]mm`, complete replay rejects -75deg near `[0,235.622,273.186]mm`, -70deg near `[0,228.773,295.161]mm`, -65deg near `[0,218.766,318.125]mm`, and -60deg near `[0,206.577,339.982]mm`, each when ID1 would cross below its -90deg relative limit.
- Pitch=-55deg is the first tested complete pass, but final ID1 is approximately -87.269deg, leaving only 2.731deg lower-limit margin. Pitch=-50deg passes with about 8.395deg margin. The user selected the intermediate final value -53deg.
- Final pitch=-53deg passes both mirrored routes. Final q is approximately `[+/-90,110.456,-101.558]deg`; endpoint ID1 is `-85.014deg`, leaving `4.986deg` to its -90deg lower limit. Whole-path minimum Z is `80.112mm` and `max|Y|=326.462mm < 405mm`; AC remains independently `440/445mm`.

## Phase 42 confirmed observation baseline

- The user confirmed both BD observation poses. They are now documented as the fixed motion baseline for the next upper/lower-controller protocol update, not as provisional test points.
- Parameter ownership is explicit: protocol/runtime/bridge code may select LEFT or RIGHT, but must not duplicate or override observation coordinates, pitch, staging or speed. `app_config.h` remains the single source for the geometry.
- No wire format or runtime protocol behavior changed in this phase. The next protocol sync must first compare the new generated `protocol.h`, `protocol.c` and `PROTOCOL_DOC.md`, then adapt project-maintained runtime and bridge code without retuning the confirmed poses.

## Phase 43 generated protocol resync

- Live generated files still use `PROTOCOL_HASH=0x740E426B`, packet IDs 0x11/0x12/0x13/0x14, CRC8, optional handshake and non-strict heartbeat. `protocol.h` and `protocol.c` introduce no new binary layout relative to the already integrated Phase 29 contract; transport/runtime replacement is unnecessary.
- The generated semantic table declares `StateMachineCommand task_id=4/status=0` as QR recognition pose and `task_id=5/status=0..3` as current area A/B/C/D, with callback IDs 4 and 5. The maintained bridge currently validates only task IDs 0, 1 and 2, so both new semantics are rejected before reaching business handling.
- No QR pose coordinates, joint targets or existing application entry point exist anywhere in the live checkout. It is unsafe to map task 4 to a guessed arm pose or report callback 4 completed. Task 5 is state-only and can be integrated idempotently without moving hardware.
- Confirmed BD LEFT/RIGHT geometry remains owned by `app_config.h`; the protocol table has no BD side-selection command in this version, so the sync must not claim that current-area task 5 selects a BD observation side.
- Task 5 is implemented as an orthogonal, idempotent state update: wire status 0..3 maps to a dedicated `Upper_Controller_Area_e`, updates can arrive while another discrete actuator command is active, and consumers use `UpperControllerGetCurrentArea()` rather than the Watch struct. It does not overwrite `pending_task_id/status`. Callback 5/1 then 5/0 is deferred to the next bridge task so the generated FSM can enqueue its automatic reliable ACK first.
- Task 4/status 0 is explicitly recognized and counted, then returned to IDLE without motion or callback 4 because the protocol has no failure callback status and no QR pose exists. Reliable ACK remains receipt-only; reporting completed would be false.

## Phase 44 AC closed-loop observation and pick

- Current HOST `StateMachineCommand task_id=2` semantics supersede the Phase 28/29 open-loop bridge behavior. `status=0` enters the confirmed left observation posture, `status=1` enters the confirmed right observation posture, and `status=2` is invalid because the following `ArmTarget` frame can only correspond to one active observation side.
- Observation execution is a two-step arm sequence: first joint staging to the selected side with the configured ID1 relative pitch, then tool-center motion to the configured `[0,+/-150,300]mm` point at `pitch=-53deg` and `150mm/s`. On successful arrival the bridge calls `UpperControllerCaptureCameraPose()` with an internal `0xACxxxxxx` capture ID and sends callback 2 completed.
- D435i extrinsic is now enabled for AC closed-loop bring-up. The stored tool-center transform remains `t_E_C=[-80.055106,69.184360,48.192321]mm` with the previously confirmed RGB optical rotation matrix. If real known-point testing shows axis or offset mismatch, `CAMERA_TARGET_DEFAULT_CALIBRATED` must be returned to 0 before further motion.
- `ArmTarget` still has no wire-level `capture_id`, so this version deliberately uses only the latest AC observation snapshot and enforces the existing 5000ms age limit. Transform success alone is not enough: the bridge also requires AC observation holding, no pending discrete command, no running AC fallback task and no running ArmTarget pick. A successful pick start consumes the observation state, so the same snapshot cannot be reused for another pick without a new observation command.
- Closed-loop AC pick target generation is intentionally simple for first hardware bring-up: `X/Y` come from the transformed base-frame point, `Z` is fixed at `APP_ARM_AC_CLOSED_LOOP_PICK_Z_MM=-100mm`, and tool pitch reuses the AC pick pitch configuration. The motion is submitted through `AppArmFlowStartPick()`, so IK, software limits, workspace safety and normal pick sequencing remain in the arm layer.
- ExecutionCallback boundaries remain layered: generated reliable ACK means packet receipt only; callback 2 means observation motion state; callback 3 means ArmTarget closed-loop pick state. Transport success, transform success and physical pick completion must not be treated as the same condition in logs or upper-computer tests.
- Phase 44 validation passed strict Cortex-M4 GCC including forced HOST runtime, permanent arm-path replay, camera transform 29 checks and `git diff --check`. No Keil build, AXF/HEX generation, flash, live USB exchange or physical arm/camera validation was performed.

## Phase 45 default mode back to HOST

- The default application mode is now `APP_MODE_HOST_CONTROL`, so the firmware no longer boots directly into the left BD observation-point test. This enables upper-computer control of chassis velocity, gripper, MG995 servos, AC observation pose and ArmTarget closed-loop pick by default.
- `APP_MODE_ARM_BD_OBSERVATION_TEST` is still present for manual regression testing of the confirmed left observation point. No BD/AC observation geometry, D435i extrinsic, protocol wire layout, chassis limits or arm path parameter changed in this phase.

## Phase 46 observation point lowered and pulled inward

- The current closed-loop observation geometry is now `[0,+/-150,300]mm/pitch=-53deg`: Z was lowered by 50mm from 350mm to 300mm, and Y was moved 50mm toward zero from +/-200mm to +/-150mm. The right point remains a strict mirror of the left point.
- Complete replay passes both mirrors. Final q is approximately `[+/-90,125.540,-81.916]deg`; endpoint ID1 relative pitch is about `-80.456deg`, leaving about `9.544deg` to the `-90deg` lower limit.
- The path-wide `max|Y|` remains `326.462mm` under the unchanged BD-only 405mm replay limit because the peak still occurs during HOME-to-staging, not at the lowered endpoint. AC pick endpoints, D435i extrinsic, protocol layout, staging, speed and chassis parameters are unchanged.

## Phase 47 observation pitch moved 5deg downward

- The current AC/BD observation geometry keeps `[0,+/-150,300]mm` and changes only world absolute pitch from `-53deg` to `-58deg`, moving 5deg closer to vertical downward (`-90deg`). RIGHT remains the strict mirror of LEFT.
- Staging remains `[+/-90,90,-80]deg`; at staging, the small-link absolute pitch is `-10deg`, so the commanded ID1 relative pitch is `-48deg`, not `-68deg`.
- Complete replay passes both mirrors. Final q is approximately `[+/-90,123.291,-84.168]deg`; endpoint ID1 relative pitch is about `-85.459deg`, leaving about `4.541deg` to the `-90deg` lower limit. Whole-path minimum Z is `77.557mm` and `max|Y|=318.051mm < 405mm`; AC pick endpoints, D435i extrinsic, protocol layout, staging, speed and chassis parameters are unchanged.

## Phase 48 AC closed-loop vertical pick pitch

- Before this phase, `APP_ARM_AC_CLOSED_LOOP_PICK_TOOL_PITCH_DEG` aliased `APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG`, so ArmTarget closed-loop picking inherited the AC open-loop side-push pitch of `-5deg`.
- The AC closed-loop target generation is now explicit: `X/Y` come from the camera-to-base transform, `Z` remains fixed at `APP_ARM_AC_CLOSED_LOOP_PICK_Z_MM=-100mm`, and the gripper world absolute pitch is fixed at `-90deg` for vertical downward picking.
- This change affects only the ArmTarget closed-loop pick submitted through `upper_controller_bridge.c`; AC open-loop side-push picking remains `-5deg`, and the observation posture remains `[0,+/-150,300]mm/pitch=-58deg`.
# Phase 52 AC闭环接近路径修正

- 实机目标经坐标变换及右侧X补偿后的 approach 为 `[437.806335,-538.556763,-105.0]mm`，30mm名义终点为 `[437.806335,-568.556763,-105.0]mm`。
- 当前拒绝发生在 `staging -> approach` 的采样341，失败工具中心约为 `[395.945801,-487.063049,-22.566711]mm`；`failed_check_mask=IK`，尚未进入推进量降级选择。
- 不能仅凭该日志认定名义终点可达；Phase 52 必须先用生产IK/规划器分别验证 approach 和最终点，再选择路径修复。
- 当前 `Arm_Path_Advance_Request_s` 只有 staging 关节、approach 和推进参数；生产内核固定把接近过程表示成单段 `staging -> approach` 工具中心直线，无法表达绕开伸直边界的安全中间点。
- 当前抓取 staging 为目标方位限幅后的 q1、`q2=80deg`、`q3=-90deg`；AC闭环抓取目标绝对俯仰为 `-15deg`。
- 精确生产IK回放结果：approach `[437.806335,-538.556763,-105]mm` 和30mm名义终点 `[437.806335,-568.556763,-105]mm` 均返回 `ARM_IK_OUT_OF_REACH`。因此本次不是“终点可达但直线路径有问题”；中间点只能解决路径拓扑，不能使这两个端点变为可达。
- approach 水平半径约694mm；按117mm工具长度和-15deg俯仰估算，腕部水平需求仍约581mm，超过260+260mm主臂总长，拒绝符合几何边界。
# Phase 54 ID1延迟动作与50mm抓后抬升

- 当前AC约束放置的首条命令直接以`release_q_deg`为终点、`transfer_waypoint_q_deg=[+/-90,75,-62.7]deg`为中间点，并从命令开始显式插值到释放ID1相对俯仰；因此机械臂尚未抬离地面时ID1已经动作。
- 现有状态机已经有`WAIT_TRANSFER -> SUBMIT_ROTATE_TO_PLACE`边界，可把首条联合route拆成独立到达transfer waypoint的命令，不必修改底层轨迹格式。
- 普通关节命令在`tool_relative_pitch_valid=0`时由轨迹层锁存动作开始时的ID1相对小臂角，因此第一段可以保持ID1机械相对位置不变；到位后现有`AppArmFlowSubmitDirectedReleaseRotation()`再设置释放俯仰。
- 当前q2=75deg/q3=-62.7deg经过点在旧固定测试起点下工具中心Z约191.37mm，已有运行时FK会基于每次真实反馈检查抬升量。用户要求应落实为最小50mm门槛，而不是仅依赖名义点高度。
- 最终实现将首条AC命令终点直接设为transfer waypoint，`waypoint_valid=0/tool_relative_pitch_valid=0`；状态机进入`WAIT_TRANSFER`并等待整条抬升命令完成，随后才进入`SUBMIT_ROTATE_TO_PLACE`。第二条命令继续使用既有safe waypoint到release的后转路径并设置释放ID1相对俯仰。
- 50mm门槛使用运行时实际三轴和ID1反馈的工具中心FK，`TRANSFER_Z_TOLERANCE_MM=0`，不会把48mm等不足值放行。固定HOST向量实际抬升331.370mm，最大|Y|=449.671mm/455mm，左右严格镜像。
- 最终HOST回放、ARM GCC `-Werror`和Keil ArmCC 5.06u7全量重建均通过。Keil镜像为Code=143972、RO-data=3660、RW-data=1432、ZI-data=137560，0错误0警告。

# Phase 53 闭爪超时降级与AC抓取参数调整

- 实机故障快照为 `pick_step=FAILED`、`command_state=FAULTED`、`command_result=NOT_READY`、`tool_error_code=SERVO_TIMEOUT`、`gripper_state=FAULT`；机械臂中心和俯仰均已到位，放置流程尚未启动。
- 当前ID2为轮询反馈状态机：正常闭合截止时间1500ms；闭爪超时会进入`ARM_GRIPPER_RELIEF_TIMEOUT`卸力，但无论卸力到位还是尝试耗尽，最终都收敛为`ARM_GRIPPER_FAULT`。
- 用户要求夹住不理想时继续放置。实现边界限定为：仅`RELIEF_TIMEOUT`在ID2仍在线/反馈有效的卸力路径中降级为`FORCED_HELD`；反馈离线、发送失败、初始化故障和非闭爪JAM保持FAULT。
- AC闭环抓取绝对俯仰将从`-15deg`改为`-20deg`；近端是否扩大必须以共享生产规划器回放结果为准，不仅凭端点IK判断。
- 首轮`-20deg`生产回放显示旧边界`X=-160/Y=543mm`从29mm降级改善为完整30mm推进，说明可达边界发生预期变化；旧HOST断言需要重新扫描而不能机械改值。
- ARM GCC第一次从仓库根目录启动，脚本内部相对路径导致找不到`../Core/Src/tim.c`；这是调用目录错误，不是源码编译错误，后续从`Engineer/MDK-ARM`运行。
- `-20deg`远端生产扫描（X=-160mm）结果：Y=545mm仍完整30mm，546mm选29mm，574mm选1mm，575mm为0mm拒绝，576mm起approach本身失败。相比`-15deg`旧向量542/543/571/572mm，完整推进、29mm、1mm和0mm边界均向外改善约3mm。
- 从正确目录运行的ARM GCC严格检查已通过，包括`arm_tool.c`、共享规划器、桥和HOST_CONTROL分支。
- `X=0/Z=-105/pitch=-20deg`的初步近端扫描显示Y=150..240mm均在staging-to-approach失败，Y=245mm起完整30mm推进通过；当前270mm业务钳位不会自动利用该余量，需要精扫240..245后同步边界才会实际更近。
- 回放一致性测试第一次仍重复旧543mm样本，导致与新546mm fallback快照比较失败；这是测试向量同步遗漏，生产规划结果本身正常。
- 1mm精扫确认X=0时244mm approach失败、245mm完整30mm成功；正式近端边界已改为245mm，最大钳位仍30mm，所以215..245mm钳到245mm，低于215mm仍拒绝。
- 当前实机变换/补偿后的右侧approach `[97.403122,-415.169006,-105]mm` 在-20deg下通过完整30mm推进永久断言。
- 用户最终撤回`-20deg/Z=-105mm`组合，要求恢复绝对俯仰`-15deg`并把固定Z下调10mm到`-115mm`；前述`-20deg`扫描仅保留为调参历史，不再代表当前固件。
- 最终生产规划扫描在`X=0/Z=-115mm/pitch=-15deg`下确认：`|Y|=274mm`的approach失败，`275mm`起可完整推进30mm。因此近端边界改为275mm，最大欠距仍30mm，业务层接受`245..275mm`并钳到275mm，低于245mm或侧别错误仍拒绝。
- 最终远端生产规划扫描在`X=-160mm/Z=-115mm/pitch=-15deg`下确认：Y<=536mm完整推进30mm，537mm为29mm，565mm为1mm，566mm为0mm拒绝，567mm起approach失败。
- 最终状态机审计发现：超时卸力正常到位会保留`SERVO_TIMEOUT`，但卸力尝试耗尽的可降级分支曾无条件覆盖为`GRIPPER_STALL`。已改为超时降级始终保留`SERVO_TIMEOUT`；接触卡滞耗尽仍记录`GRIPPER_STALL`，便于赛后区分。
- 完整HOST回放、ARM GCC `-Werror`、相机变换29项测试和`git diff --check`均通过；后者只有现有行尾提示。
# Phase 55 连续放置经过点与观察后高位抓取路径

- Phase54通过两条命令保证ID1延迟，但`WAIT_TRANSFER`要求q2=75deg经过点完整到位，实机会出现硬停顿。若不扩展底层分段ID1插值，最稳妥的连续方案是一条`当前抓取姿态 -> transfer waypoint -> release`关节轨迹全程锁存ID1，后方到位后再单独提交释放ID1角；这满足“不在低位动ID1”且waypoint不停顿。
- 观察后ArmTarget已有连续关节staging：当前q1下先到q2/q3=`[80,-90]deg`，再连续对准目标q1，随后才执行staging到approach的工具中心直线。现有staging是关节安全准备点，不保证工具中心高于观察点，需要用生产FK和完整路径重新选点。
- 最终选定高位抓取staging为`[目标q1,100,-110]deg`、ID1相对俯仰`-80deg`；生产FK给出工具中心约`Z=358.4mm`，比`[0,+/-150,300]mm`观察点高约58mm。相比候选`[110,-110]deg`，该点仍满足先抬高，同时保留旧近端边界。现有base-aim命令本来就把staging作为连续关节waypoint，因此无需增加新状态。
- AC抓后连续路线改为一条命令从抓取姿态经过`[+/-90,75,-62.7]deg`到后方release q，且`tool_relative_pitch_valid=0`；ID1只在后方到位后改变。HOST关节路径回放通过左右镜像，名义经过点抬升`331.370mm`，`max|Y|=449.671/455mm`。
- 高位staging使旧共享规划器近端向量`275通过/274拒绝`失效：`Y=275mm`现返回规划状态4。定稿前必须重算近端/远端边界并同步钳位常量与永久回放。
- 候选对比扫描确认：`[110,-110]deg`会把X=0近端边界外移到286mm；最终`[100,-110]deg`恢复`275通过/274拒绝`。两者远端均保持536满30、537降29、565降1、566为0、567 approach失败，因此最终无需修改近端钳位或推进测试向量。
- 最终验证：HOST回放通过连续抓后转移、观察到高位staging双侧镜像、后续抓放过渡、30/29/1/0mm推进降级、现场坐标和规划器边界；ARM GCC全量`-Werror`通过。Keil工具在当前环境不可用，未进行Keil重建或烧录。
- 右侧MG995校准第一次符号验证失败：`-15deg`使实机反而更高，说明该舵机正方向与左侧相反。已修正为右侧专用物理PWM补偿`+15deg`；逻辑90deg仍代表摄像头水平，实际脉宽约1583us，对应物理105deg。
- Phase 57 source audit: `UpperControllerArmTargetGateFlags()` previously rejected every non-AC observation group. `on_receive_ArmTarget()` then unconditionally built the AC place profile, clamped Y, fixed Z to `APP_ARM_AC_CLOSED_LOOP_PICK_Z_MM`, applied AC advance and `-15deg`. B区 now needs a pick-only completion path because `AppArmSidePickPlaceBuildPlaceProfile()` explicitly documents AC-only profiles.
- Phase 57 validation: B分支编译通过，AC既有共享推进与放置回放保持通过。B区实际链路为 `task_id=5,status=1 -> task_id=2,status=0/1 -> ArmTarget`，变换后的 `base_point_mm` 直接作为Y/Z和目标高度，X仅叠加AC侧别补偿；目标俯仰为0deg。B区抓取完成后不自动放置，等待后续独立BD放置策略。
- Phase 58 source finding: before the change, current-area handling only moved both cameras to `-45deg` for AC and left BD at the previous angle. The area-group branch now explicitly maps AC to `UPPER_CAMERA_LOOK_DOWN_DEG` and BD to `UPPER_CAMERA_LOOK_UP_DEG` (`+45deg`).
