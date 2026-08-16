/**
 * @file camera_target_transform_config.h
 * @brief Camera extrinsic placeholders to be replaced after physical measurement.
 *
 * Keep CAMERA_TARGET_DEFAULT_CALIBRATED at 0 until both translation and camera
 * axis orientation have been measured. Identity values below are inert while
 * calibration is disabled.
 */

#ifndef CAMERA_TARGET_TRANSFORM_CONFIG_H
#define CAMERA_TARGET_TRANSFORM_CONFIG_H

#define CAMERA_TARGET_DEFAULT_CALIBRATED                 0u

/* Select where the camera is rigidly mounted. */
#define CAMERA_TARGET_DEFAULT_REFERENCE_FRAME \
    CAMERA_TARGET_REFERENCE_TOOL_CENTER

/* Camera optical origin expressed in the selected E frame, millimetres. */
#define CAMERA_TARGET_DEFAULT_T_E_C_X_MM                  0.0f
#define CAMERA_TARGET_DEFAULT_T_E_C_Y_MM                  0.0f
#define CAMERA_TARGET_DEFAULT_T_E_C_Z_MM                  0.0f

/*
 * R_E_C: each column is one camera axis expressed in E.
 * Do not enable calibration until camera X/Y/Z axis directions are confirmed.
 */
#define CAMERA_TARGET_DEFAULT_R_E_C_00                    1.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_01                    0.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_02                    0.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_10                    0.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_11                    1.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_12                    0.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_20                    0.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_21                    0.0f
#define CAMERA_TARGET_DEFAULT_R_E_C_22                    1.0f

/* ArmTarget may only use a recently stored capture-time pose. */
#define CAMERA_TARGET_DEFAULT_MAX_POSE_AGE_MS          5000u

#endif
