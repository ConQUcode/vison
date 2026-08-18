# 末端摄像头目标坐标转换

更新时间：2026-08-17

## 1. 当前完成边界

上位机 `ArmTarget.target_x/y/z` 表示水果在摄像头坐标系中的位置，单位为米。
固件先转换为毫米，再使用拍照时机械臂姿态和固定相机外参计算水果在机械臂
基座坐标系中的坐标：

```text
P_B_F = T_B_E(capture_pose) * T_E_C(calibration) * P_C_F
```

- `B`：机械臂基座系，`+X`车头前方、`+Y`物理左侧、`+Z`向上。
- `E`：相机安装所依附的机械臂末端参考系。
- `C`：上位机发送坐标所使用的相机坐标系。
- `F`：水果目标点。

核心算法位于 `camera_target_transform.c/.h`，配置位于
`camera_target_transform_config.h`。当前已保存并启用D435i官方/Fusion外参，
用于AC闭环联调；若实机已知点验证发现左右、前后或高度方向不一致，应立即把
`CAMERA_TARGET_DEFAULT_CALIBRATED`改回0。

当前 `ArmTarget` 的执行边界是：必须先由`StateMachineCommand task_id=2`
完成左/右观察并保存pose snapshot；随后`ArmTarget`用最近一次快照换算目标。
换算成功且机械臂空闲时，抓取终点`X/Y`取基座系结果，`Z`固定为`-100 mm`，
夹爪世界绝对俯仰固定为竖直向下`-90 deg`，再进入`AppArmFlowStartPick()`。可靠ACK仍然只表示下位机收到了包，闭环抓取
执行状态由`ExecutionCallback(callback_id=3)`报告。一次`ArmTarget`成功启动后会
消费当前观察状态，避免同一快照在5000ms窗口内重复触发抓取。

## 2. 两种安装参考系

配置 `CAMERA_TARGET_DEFAULT_REFERENCE_FRAME` 必须与实物安装一致：

| 配置 | E系原点 | E系前向 | 适用安装位置 |
|---|---|---|---|
| `CAMERA_TARGET_REFERENCE_TOOL_CENTER` | 夹爪中心 | ID1控制后的夹爪绝对俯仰方向 | 摄像头安装在ID1之后，随夹爪俯仰 |
| `CAMERA_TARGET_REFERENCE_WRIST_PITCH_AXIS` | ID1俯仰轴心 | 小臂绝对俯仰方向 | 摄像头安装在ID1之前，固定在小臂/腕部 |

两种E系均定义为右手系：E-X沿对应末端前向，E-Y为该朝向的水平左侧，
E-Z补成右手系。机械臂只能提供底座Yaw和末端Pitch，摄像头安装产生的固定
Roll必须包含在 `R_E_C` 中。

## 3. 当前已确认的D435i安装外参

相机固定在ID1之后并随夹爪绝对俯仰一起转动，使用
`CAMERA_TARGET_REFERENCE_TOOL_CENTER`。Fusion测得ID1轴心到D435i底部螺丝
圆心为`[26.800000,36.726000,35.577000]mm`；结合官方螺丝到RGB光心偏移，
ID1轴心到`camera_color_optical_frame`光心为
`[36.944894,69.184360,48.192321]mm`。

当前ID1轴心到夹爪中心沿工具`+X`为117mm，因此配置中保存的工具中心外参为：

```text
t_E_C = [-80.055106, 69.184360, 48.192321] mm

R_E_C =
[ -0.00239767   0.00237706   0.99999400 ]
[ -0.99999200   0.00320883  -0.00240529 ]
[ -0.00321453  -0.99999200   0.00236935 ]
```

旋转矩阵把RGB光学`+X向右/+Y向下/+Z向前`映射到工具中心E系。若实物
117mm长度、相机支架或D435i安装位置变化，必须重新计算并恢复
`calibrated=0`。

当前已为AC联调启用该外参，但正式闭环协议仍建议补齐运行链：同一个非零
`capture_id`关联拍照姿态和目标包，使用实机已知点验证左右/前后/高度误差，并在
变换后再应用AC固定Z；不能在变换前丢弃相机深度，也不能绕过工具中心IK、工作
空间和完整路径预检。

固件矩阵方向固定为：

```text
P_E = R_E_C * P_C + t_E_C
```

`R_E_C` 的三列依次是相机 `C-X/C-Y/C-Z` 单位轴在E系中的坐标。例如，若
相机C-Z朝E-X、C-X朝E-Y、C-Y朝E-Z，则三列按该轴映射填写。矩阵必须正交且
行列式接近 `+1`；把某一轴符号填反造成镜像矩阵时，固件会明确拒绝。

如果用角度描述安装方向，可调用 `CameraTargetBuildExtrinsicFromRpy()`；其
约定为 `R_E_C=Rz(yaw)*Ry(pitch)*Rx(roll)`，零角表示C/E轴完全重合。对于
相机光学系这种轴置换，直接填写三列轴方向通常更不易出错。

## 4. 拍照姿态关联

未来图像触发处必须立即调用：

```c
UpperControllerCaptureCameraPose(capture_id, now_ms);
```

该接口读取当时三关节反馈、腕点、夹爪中心、小臂绝对俯仰和ID1绝对俯仰，
生成并保存 `T_B_E`。不能等 `ArmTarget` 到达后再读取“当前姿态”替代拍照
姿态，因为上位机推理期间机械臂可能已经移动。

当前 `ArmTarget` 没有 `capture_id`，桥只能使用最新AC观察快照，并通过
`CAMERA_TARGET_DEFAULT_MAX_POSE_AGE_MS=5000ms`限制年龄。正式闭环前建议把同一个
非零帧ID同时带入图像触发和目标包；变换接口已支持`expected_capture_id`匹配
以及快照最大年龄检查，无需重写矩阵算法。

## 5. 校验和Watch

变换前会拒绝：未初始化、未标定、非有限值、外参过大、旋转矩阵不正交、
行列式不是 `+1`、姿态无效、参考系不一致、快照ID不匹配、快照过期和目标
坐标越界。变换后的基座坐标仍必须经过现有工具中心IK、软件限位、工作空间
和整条路径预检，不能绕过机械臂安全层。

主要观察量：

- `g_camera_target_transform_debug.extrinsic`：当前 `R_E_C/t_E_C`和参考系。
- `g_camera_target_transform_debug.pose_snapshot`：拍照ID、时刻、关节和`T_B_E`。
- `g_camera_target_transform_debug.last_result`：相机点、E系点和基座系点。
- `g_camera_target_transform_debug.last_status`：唯一变换失败原因。
- `g_upper_controller_debug.arm_target_camera_mm`：上位机原始目标。
- `g_upper_controller_debug.arm_target_base_mm`：成功时的基座坐标。
- `g_upper_controller_debug.arm_target_transform_status`：桥接层观察状态。

离线测试位于 `tools/camera_target_transform_test`，覆盖单位矩阵、纯平移、
绕X/Y/Z 90度、机械臂Yaw/Pitch、左右镜像、反射矩阵、参考系不匹配、帧ID
不匹配和过期快照。
