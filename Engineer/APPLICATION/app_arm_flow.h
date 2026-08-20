/**
 * @file app_arm_flow.h
 * @brief 机械臂两条应用子流程：坐标抓取(PickFlow)和固定角度放置(PlaceFlow)。
 *
 * 工程内机械臂应用动作只有两类控制方式，本模块把它们各自封装成
 * 独立子状态机，调用方通过 Start/Poll 串联，不再混在一个大状态机里：
 * - PickFlow：工具中心坐标抓取。底座先对准目标方位（限幅在
 *   +/-89.5deg内避免跨X=0），同时让q2/q3进入俯仰可达的准备姿态；
 *   随后按世界坐标和绝对工具俯仰执行工具中心IK轨迹并闭合夹爪。
 * - PlaceFlow：显式profile关节控制。转移姿态、底座引导旋转到后方、
 *   固定释放姿态、张开夹爪、小臂上抬净空并让底座回到前方。
 *
 * 同一时刻只允许一个子流程活动；任一命令失败后锁存FAILED原位保持，
 * 不自动重试，与旧抓放测试行为一致。
 */

#ifndef APP_ARM_FLOW_H
#define APP_ARM_FLOW_H

#include <stdint.h>

/** 当前活动的子流程；NONE表示空闲，可接受新的Start请求。 */
typedef enum {
    APP_ARM_FLOW_NONE = 0,
    APP_ARM_FLOW_PICK,
    APP_ARM_FLOW_PLACE
} App_Arm_Flow_Active_e;

/** 子流程总状态；DONE/FAILED为终态，FAILED后拒绝再次Start。 */
typedef enum {
    APP_ARM_FLOW_IDLE = 0,
    APP_ARM_FLOW_RUNNING,
    APP_ARM_FLOW_DONE,
    APP_ARM_FLOW_FAILED
} App_Arm_Flow_Status_e;

typedef enum {
    APP_ARM_FLOW_START_ACCEPTED = 0,
    APP_ARM_FLOW_START_BUSY,
    APP_ARM_FLOW_START_NOT_CONFIGURED,
    APP_ARM_FLOW_START_INVALID,
    APP_ARM_FLOW_START_FAILED
} App_Arm_Flow_Start_Result_e;

/** 工具中心坐标抓取子流程步骤；旧编号尽量保持稳定，执行顺序看状态机。 */
typedef enum {
    APP_ARM_PICK_STEP_IDLE = 0,
    APP_ARM_PICK_STEP_SUBMIT_BASE_AIM,   /* 底座对准并同步进入抓取准备姿态。 */
    APP_ARM_PICK_STEP_WAIT_BASE_AIM,
    APP_ARM_PICK_STEP_SUBMIT_APPROACH,   /* 可选接近点，闭环抓取用。 */
    APP_ARM_PICK_STEP_WAIT_APPROACH,
    APP_ARM_PICK_STEP_SUBMIT_TARGET,     /* 提交夹爪中心坐标和绝对俯仰。 */
    APP_ARM_PICK_STEP_WAIT_TARGET,
    APP_ARM_PICK_STEP_WAIT_PITCH_STABLE, /* 等ID1反馈稳定在目标附近。 */
    APP_ARM_PICK_STEP_PICK_DWELL,        /* 抓取前固定停留。 */
    APP_ARM_PICK_STEP_SUBMIT_CLOSE,      /* 夹爪闭合，含堵转分级卸力。 */
    APP_ARM_PICK_STEP_WAIT_CLOSE,
    APP_ARM_PICK_STEP_POST_GRIP_DWELL,   /* 抓取后固定停留。 */
    APP_ARM_PICK_STEP_DONE,
    APP_ARM_PICK_STEP_FAILED,
    APP_ARM_PICK_STEP_SUBMIT_SAFE_STAGING, /* 先收大臂/小臂/ID1，底座保持当前角。 */
    APP_ARM_PICK_STEP_WAIT_SAFE_STAGING
} App_Arm_Pick_Step_e;

/** 显式profile放置子流程步骤；旧编号尽量保持稳定，执行顺序看状态机。 */
typedef enum {
    APP_ARM_PLACE_STEP_IDLE = 0,
    APP_ARM_PLACE_STEP_SUBMIT_TRANSFER,        /* AC连续抬升并后转，ID1保持不动。 */
    APP_ARM_PLACE_STEP_WAIT_TRANSFER,          /* 非AC放置的过渡等待。 */
    APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_PLACE, /* 左逆时针/右顺时针转到后方。 */
    APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_PLACE,
    APP_ARM_PLACE_STEP_SUBMIT_RELEASE_POSE,    /* 释放关节角+ID1相对俯仰联合命令。 */
    APP_ARM_PLACE_STEP_WAIT_RELEASE_POSE,
    APP_ARM_PLACE_STEP_WAIT_RELEASE_PITCH,     /* 确认ID1反馈到位后才允许释放。 */
    APP_ARM_PLACE_STEP_SUBMIT_OPEN,            /* 夹爪张开释放。 */
    APP_ARM_PLACE_STEP_WAIT_OPEN,
    APP_ARM_PLACE_STEP_SUBMIT_RELEASE_CLEARANCE, /* ID2释放后小臂上抬。 */
    APP_ARM_PLACE_STEP_WAIT_RELEASE_CLEARANCE,
    APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_FRONT, /* 沿本次抓取侧返回前方。 */
    APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_FRONT,
    APP_ARM_PLACE_STEP_DONE,
    APP_ARM_PLACE_STEP_FAILED,
    APP_ARM_PLACE_STEP_SUBMIT_REAR_STAGING,    /* 后方旋转前，底座不动，先收大臂/小臂/ID1。 */
    APP_ARM_PLACE_STEP_WAIT_REAR_STAGING
} App_Arm_Place_Step_e;

/** 失败来源分类，避免机械臂故障码与命令结果码数值重叠。 */
typedef enum {
    APP_ARM_PICK_PLACE_FAILURE_NONE = 0,
    APP_ARM_PICK_PLACE_FAILURE_ARM,
    APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
    APP_ARM_PICK_PLACE_FAILURE_COMMAND_EXECUTION
} App_Arm_Pick_Place_Failure_Source_e;

/** AC抓后转移阶段拒绝原因；0表示本轮转移预检/提交未拒绝。 */
typedef enum {
    APP_ARM_TRANSFER_REJECT_NONE = 0,
    APP_ARM_TRANSFER_REJECT_FEEDBACK_INVALID,
    APP_ARM_TRANSFER_REJECT_RELATIVE_PITCH_RANGE,
    APP_ARM_TRANSFER_REJECT_PATH_INVALID,
    APP_ARM_TRANSFER_REJECT_Y_LIMIT,
    APP_ARM_TRANSFER_REJECT_Z_RAISE,
    APP_ARM_TRANSFER_REJECT_COMMAND_SUBMIT
} App_Arm_Transfer_Reject_Reason_e;

/**
 * 工具中心抓取目标。x/y/z为夹爪中心世界坐标，tool_pitch_deg为
 * 世界绝对俯仰角；四个字段都直接参与运动规划。
 */
typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float tool_pitch_deg;
    uint8_t approach_valid;
    float approach_x_mm;
    float approach_y_mm;
    float approach_z_mm;
} App_Arm_Pick_Target_s;

typedef enum {
    APP_ARM_ADVANCE_REJECT_NONE = 0,
    APP_ARM_ADVANCE_REJECT_INVALID,
    APP_ARM_ADVANCE_REJECT_BUSY,
    APP_ARM_ADVANCE_REJECT_APPROACH,
    APP_ARM_ADVANCE_REJECT_IK,
    APP_ARM_ADVANCE_REJECT_JOINT_LIMIT,
    APP_ARM_ADVANCE_REJECT_TOOL_PITCH,
    APP_ARM_ADVANCE_REJECT_WORKSPACE,
    APP_ARM_ADVANCE_REJECT_CONTINUITY,
    APP_ARM_ADVANCE_REJECT_SAMPLE_CAPACITY
} App_Arm_Advance_Reject_Reason_e;

typedef struct {
    float requested_mm;
    float selected_mm;
    uint8_t approach_failed;
    App_Arm_Advance_Reject_Reason_e reject_reason;
    uint32_t planner_status;
    uint32_t ik_status;
    uint32_t workspace_safety_result;
    uint32_t failed_check_mask;
    uint16_t failed_sample;
    float failed_center_mm[3];
} App_Arm_Advance_Result_s;

/**
 * 工具中心抓取前的统一关节准备姿态。q1由目标XY方位计算并限幅，q2/q3
 * 和ID1相对俯仰来自同一组已验证配置，正式抓取与专项测试必须共同使用。
 */
typedef struct {
    float q_deg[3];
    float tool_relative_pitch_deg;
} App_Arm_Pick_Staging_s;

/**
 * 单个已实测放置策略。三组q均为完整三轴关节位姿；两个waypoint显式
 * 约束底座绕行方向。未实测区域必须保持configured=0，禁止复用A区。
 */
typedef struct {
    uint32_t profile_id;
    uint8_t configured;
    uint8_t transfer_waypoint_valid;
    uint8_t transfer_path_constraints_enabled;
    float transfer_waypoint_q_deg[3];
    float transfer_path_y_max_mm;
    float transfer_waypoint_z_raise_mm;
    float transfer_waypoint_z_tolerance_mm;
    float safe_q_deg[3];
    float rotate_to_place_waypoint_q1_deg;
    float rotate_to_place_target_q1_deg;
    float release_q_deg[3];
    float release_tool_relative_pitch_deg;
    uint32_t release_pitch_wait_timeout_ms;
    float release_clearance_q_deg[3];
    float rotate_to_front_waypoint_q1_deg;
    float rotate_to_front_target_q1_deg;
} App_Arm_Place_Profile_s;

/**
 * 抓放子流程的紧凑Watch变量；符号名沿用旧抓放测试，Watch配置不变。
 * center和wrist单位mm，pitch/q单位deg；workspace_safety_result对应arm.h枚举。
 */
typedef struct {
    uint8_t active_flow;  /* App_Arm_Flow_Active_e：当前活动子流程。 */
    uint8_t flow_status;  /* App_Arm_Flow_Status_e：RUNNING/DONE/FAILED。 */
    uint8_t pick_step;    /* App_Arm_Pick_Step_e：抓取子流程步骤。 */
    uint8_t place_step;   /* App_Arm_Place_Step_e：放置子流程步骤。 */
    uint32_t submit_result;
    uint32_t fault; /* 兼容旧Watch；新代码应优先查看下面四个明确字段。 */
    App_Arm_Pick_Place_Failure_Source_e failure_source;
    uint32_t arm_fault_code;
    uint32_t command_state;
    uint32_t command_result;
    uint32_t motion_state;
    uint32_t motion_fault;
    uint32_t tool_error_code;
    uint32_t active_command_id;
    uint32_t place_profile_id;
    App_Arm_Flow_Start_Result_e last_start_result;
    uint32_t cycle_count;
    uint32_t state_elapsed_ms; /* 当前步骤已持续时间ms。 */
    uint8_t servo1_communication_ok;
    uint8_t servo2_communication_ok;
    uint8_t gripper_state;
    float pitch_target_deg;
    float pitch_feedback_deg;
    float pitch_error_deg; /* 当前绝对俯仰目标减实际反馈，单位deg。 */
    float target_center_mm[3];
    float feedback_center_mm[3];
    float center_error_mm[3];
    float center_error_norm_mm;
    float target_wrist_mm[3];
    float target_q_deg[3];
    uint8_t transfer_path_y_check_passed;
    float transfer_path_y_limit_mm;
    float transfer_path_peak_abs_y_mm;
    uint8_t transfer_path_z_check_passed;
    float transfer_path_start_z_mm;
    float transfer_path_waypoint_z_mm;
    float transfer_path_z_raise_mm;
    uint8_t transfer_reject_reason; /* App_Arm_Transfer_Reject_Reason_e */
    uint8_t safety_route_enabled;
    uint8_t safety_route_segment;
    uint32_t workspace_safety_result;
    uint32_t ik_status;
    uint16_t path_sample_count;
    uint32_t preflight_duration_ms;
    uint32_t preflight_motor_service_count;
    uint32_t preflight_tool_service_count;
    uint8_t dm_online[3];       /* 底座、大臂、小臂达妙当前在线状态。 */
    uint32_t dm_rx_count[3];    /* 三轴累计CAN反馈帧数。 */
    uint32_t dm_feedback_age_ms[3]; /* 三轴最新反馈距当前任务时间。 */
    uint8_t preflight_failed_segment;
    uint16_t preflight_failed_sample;
    uint32_t preflight_failed_check_mask;
    float preflight_failed_center_mm[3];
    float preflight_failed_q_deg[3];
} App_Arm_Pick_Place_Test_Debug_s;

extern App_Arm_Pick_Place_Test_Debug_s g_app_arm_pick_place_test_debug;

/**
 * 根据夹爪中心目标XY构造抓取准备姿态。返回0表示参数无效；成功时完整
 * 写出q1/q2/q3和ID1相对俯仰，不提交命令，也不改变子流程状态。
 */
uint8_t AppArmFlowBuildPickStaging(float target_x_mm, float target_y_mm,
                                   App_Arm_Pick_Staging_s *staging);

/**
 * 用正式轨迹规划器选择Y方向最大连续可达推进量并更新target终点。
 * 不提交运动；返回0时result给出明确拒绝原因。
 */
uint8_t AppArmFlowSelectReachablePickAdvance(
    App_Arm_Pick_Target_s *target, float advance_sign,
    float requested_advance_mm, float sample_step_mm,
    App_Arm_Advance_Result_s *result);

/** 清零两个子流程和Watch状态；上电初始化时调用一次。 */
void AppArmFlowInit(void);

/**
 * 启动工具中心坐标抓取子流程。返回1表示已受理；有子流程在运行、参数
 * 无效或此前已锁存FAILED时返回0，不打断当前动作。
 */
uint8_t AppArmFlowStartPick(const App_Arm_Pick_Target_s *target,
                            uint32_t now_ms);

/**
 * 启动显式profile放置子流程。函数在提交任何动作前完整校验profile，
 * 未配置和字段非法分别返回NOT_CONFIGURED/INVALID。
 */
App_Arm_Flow_Start_Result_e AppArmFlowStartPlace(
    const App_Arm_Place_Profile_s *profile, uint32_t now_ms);

/**
 * 周期推进当前子流程并刷新Watch；无活动流程时只刷新Watch。
 * 返回值为当前子流程总状态；DONE后保持直到下一次Start。
 */
App_Arm_Flow_Status_e AppArmFlowPoll(uint32_t now_ms);
/** 只读当前子流程总状态；协议桥用它发送ArmTarget执行回调。 */
App_Arm_Flow_Status_e AppArmFlowGetStatus(void);
/**
 * 上位机return initial pose/reset命令使用：退出当前抓取或放置子流程，
 * 清除FAILED/DONE锁存，使后续HOME命令和新任务能重新受理。
 * 本函数只复位应用层状态，不直接提交电机取消；调用方负责发送取消命令。
 */
void AppArmFlowAbort(uint32_t now_ms);

#endif
