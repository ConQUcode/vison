# 水果采摘规则与流程整理计划

## 目标

基于《水果采摘规则》和当前实机测试代码，实施采摘区域/点位、底盘相对运动、机械臂安全转运及释放策略的分层整理，并保证完成后自动运行 `585/A左 -> 500/A右 -> 500/A左` 三次测试流程。

## 当前约束

- 本阶段实施已获用户确认的整理计划。
- 当前实机方向以用户最新验证结果为准。
- 最终行为保持当前测试顺序不变。
- 保留工作区内全部已有修改，不回退无关内容。

## 阶段

- [complete] 阶段1：完整阅读并核对水果采摘规则 PDF。
- [complete] 阶段2：审计当前 585/500 底盘测试流程及点位语义。
- [complete] 阶段3：审计安全点、同侧转运和释放流程的耦合关系。
- [complete] 阶段4：设计面向后续上位机接入的底盘命令边界。
- [complete] 阶段5：形成分阶段实施、兼容和验证计划。
- [complete] 阶段6：实施底盘通用接口、机械臂 profile 和 A 区任务调度。
- [complete] 阶段7：更新 Watch、项目文档并完成静态校验。

## 验收标准

- 明确 `585 = 500 + 85` 只用于首段安装补偿，后续相邻水果组均前进 `500 mm`。
- 明确点1=A区左侧水果点、点2=A区右侧水果点。
- 安全点和释放策略支持按区域/侧别配置，不假设后续区域完全复用A区参数。
- 底盘测试编排与底盘运动控制解耦，可平滑替换为上位机命令来源。
- 重构后测试顺序和现有实机方向保持不变。

## 错误记录

- PowerShell 启动失败 `8009001d`：后续只使用原生 `cmd.exe`。
- 系统无 Poppler、无 Python：改用 Node 环境中的 `pdfjs-dist` 与 `@napi-rs/canvas`。
- `pdfjs-dist` 首次加载缺少 DOM 图形对象：正在通过画布模块补齐兼容对象后重新载入。
- `cmd rmdir /s /q` 删除已精确核对的 `undefined/` 缓存时被执行策略拒绝：改用 Windows `Remove-Item -LiteralPath`，不使用通配符。
- Windows `cmd` 对带空格或 `|` 的搜索表达式发生引号误解析：最终残留核对改用逐符号固定字符串搜索，不把失败查询当作通过证据。

## 最终验证

- `git diff --check` 通过，仅有仓库Windows行尾转换提示。
- 任务表静态确认：一次 `APP_FRUIT_AREA_A_FIRST_MOVE_MM`、两次 `APP_FRUIT_AREA_A_GROUP_SPACING_MM`，距离为585/500/500，点位顺序为A左/A右/A左。
- 源码中无 `ChassisInitOneShotStraight`、`ChassisStartOneShotStraight`、`ChassisOneShotDone` 和旧 `APP_ARM_BETWEEN_PICK_CHASSIS_DISTANCE_M`。
- 放置调用只通过 `AppArmFlowStartPlace(task->place_profile, now_ms)`，底层无按抓取q1正负选择路线的函数。
- `app_fruit_task.c` 已加入Keil工程，协议目录无改动。
- 按用户要求未运行Keil/GCC编译、未烧录、未进行硬件测试。
