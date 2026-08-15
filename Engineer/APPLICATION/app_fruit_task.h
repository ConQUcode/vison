/**
 * @file app_fruit_task.h
 * @brief A 区水果采摘测试的场地任务模型、调度器和 Watch。
 */

#ifndef APP_FRUIT_TASK_H
#define APP_FRUIT_TASK_H

#include <stdint.h>

#include "app_arm_flow.h"

typedef enum {
    APP_FRUIT_AREA_UNCONFIGURED = 0,
    APP_FRUIT_AREA_A
} App_Fruit_Area_e;

typedef enum {
    APP_FRUIT_SIDE_NONE = 0,
    APP_FRUIT_SIDE_LEFT,
    APP_FRUIT_SIDE_RIGHT
} App_Fruit_Side_e;

typedef enum {
    APP_FRUIT_POINT_NONE = 0,
    APP_FRUIT_POINT_A_LEFT,
    APP_FRUIT_POINT_A_RIGHT
} App_Fruit_Point_Id_e;

typedef enum {
    APP_FRUIT_TASK_WAIT_READY = 0,
    APP_FRUIT_TASK_WAIT_CHASSIS,
    APP_FRUIT_TASK_PICK,
    APP_FRUIT_TASK_PLACE,
    APP_FRUIT_TASK_DONE,
    APP_FRUIT_TASK_FAILED
} App_Fruit_Task_State_e;

typedef enum {
    APP_FRUIT_FAILURE_NONE = 0,
    APP_FRUIT_FAILURE_NOT_CONFIGURED,
    APP_FRUIT_FAILURE_CHASSIS_SUBMIT,
    APP_FRUIT_FAILURE_CHASSIS,
    APP_FRUIT_FAILURE_ARM_START,
    APP_FRUIT_FAILURE_ARM
} App_Fruit_Failure_Source_e;

typedef struct {
    App_Fruit_Task_State_e state;
    App_Fruit_Area_e area;
    App_Fruit_Side_e side;
    App_Fruit_Point_Id_e point_id;
    uint8_t task_index;
    uint8_t task_count;
    uint8_t completed_count;
    uint32_t place_profile_id;
    uint32_t chassis_command_id;
    float chassis_target_distance_mm;
    float chassis_actual_distance_mm;
    uint8_t arm_active_flow;
    uint8_t arm_pick_step;
    uint8_t arm_place_step;
    App_Fruit_Failure_Source_e failure_source;
    uint32_t failure_code;
} App_Fruit_Task_Debug_s;

extern App_Fruit_Task_Debug_s g_app_fruit_task_debug;

/** 按显式区域和侧别复制已配置的放置profile；未配置时返回0并清空输出。 */
uint8_t AppFruitGetPlaceProfile(App_Fruit_Area_e area,
                                App_Fruit_Side_e side,
                                App_Arm_Place_Profile_s *profile);

void AppFruitTaskInit(void);
void AppFruitTask(uint32_t now_ms);

#endif
