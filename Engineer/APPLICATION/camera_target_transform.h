/**
 * @file camera_target_transform.h
 * @brief Rigid transform from a camera target point to the arm base frame.
 *
 * Frame convention:
 *   B: arm base, +X vehicle front, +Y physical left, +Z up.
 *   E: selected camera mount reference frame on the arm.
 *   C: camera coordinate frame used by the upper controller.
 *
 * The calibrated extrinsic is T_E_C:
 *   p_E = R_E_C * p_C + t_E_C
 * The capture-time arm pose is T_B_E:
 *   p_B = R_B_E * p_E + t_B_E
 */

#ifndef CAMERA_TARGET_TRANSFORM_H
#define CAMERA_TARGET_TRANSFORM_H

#include <stdint.h>

#define CAMERA_TARGET_VECTOR_SIZE 3u

typedef enum {
    CAMERA_TARGET_REFERENCE_WRIST_PITCH_AXIS = 0,
    CAMERA_TARGET_REFERENCE_TOOL_CENTER = 1
} Camera_Target_Reference_Frame_e;

typedef enum {
    CAMERA_TARGET_STATUS_OK = 0,
    CAMERA_TARGET_STATUS_NULL_ARGUMENT,
    CAMERA_TARGET_STATUS_NOT_INITIALIZED,
    CAMERA_TARGET_STATUS_NOT_CALIBRATED,
    CAMERA_TARGET_STATUS_INVALID_REFERENCE_FRAME,
    CAMERA_TARGET_STATUS_NONFINITE_VALUE,
    CAMERA_TARGET_STATUS_ROTATION_NOT_ORTHONORMAL,
    CAMERA_TARGET_STATUS_ROTATION_BAD_DETERMINANT,
    CAMERA_TARGET_STATUS_INVALID_POSE,
    CAMERA_TARGET_STATUS_NO_POSE_SNAPSHOT,
    CAMERA_TARGET_STATUS_REFERENCE_FRAME_MISMATCH,
    CAMERA_TARGET_STATUS_CAPTURE_ID_MISMATCH,
    CAMERA_TARGET_STATUS_POSE_SNAPSHOT_EXPIRED,
    CAMERA_TARGET_STATUS_POINT_OUT_OF_RANGE,
    CAMERA_TARGET_STATUS_RESULT_NONFINITE
} Camera_Target_Transform_Status_e;

typedef struct {
    /* R_E_C: columns are the camera C axes expressed in reference frame E. */
    float rotation_e_from_c[3][3];
    /* t_E_C: camera optical origin expressed in E, millimetres. */
    float translation_e_from_c_mm[3];
    Camera_Target_Reference_Frame_e reference_frame;
    uint8_t calibrated;
} Camera_Target_Extrinsic_s;

typedef struct {
    uint32_t capture_id;
    uint32_t capture_tick_ms;
    Camera_Target_Reference_Frame_e reference_frame;
    float q_deg[3];
    float reference_pitch_deg;
    float rotation_b_from_e[3][3];
    float translation_b_from_e_mm[3];
    uint8_t valid;
} Camera_Arm_Pose_Snapshot_s;

typedef struct {
    Camera_Target_Transform_Status_e status;
    uint32_t capture_id;
    uint32_t capture_tick_ms;
    float camera_point_mm[3];
    float reference_point_mm[3];
    float base_point_mm[3];
    uint8_t valid;
} Camera_Target_Transform_Result_s;

typedef struct {
    uint8_t initialized;
    uint8_t extrinsic_configured;
    uint8_t pose_snapshot_valid;
    Camera_Target_Transform_Status_e last_status;
    Camera_Target_Extrinsic_s extrinsic;
    Camera_Arm_Pose_Snapshot_s pose_snapshot;
    Camera_Target_Transform_Result_s last_result;
    float rotation_orthogonality_error;
    float rotation_determinant;
    uint32_t extrinsic_accept_count;
    uint32_t extrinsic_reject_count;
    uint32_t pose_store_count;
    uint32_t pose_reject_count;
    uint32_t transform_success_count;
    uint32_t transform_fail_count;
} Camera_Target_Transform_Debug_s;

extern Camera_Target_Transform_Debug_s g_camera_target_transform_debug;

/** Load the compile-time placeholder/configuration; never moves hardware. */
void CameraTargetTransformInit(void);

/**
 * Configure T_E_C. A zero calibrated flag returns NOT_CALIBRATED and leaves
 * transformation disabled even if the numeric fields contain identity values.
 */
Camera_Target_Transform_Status_e CameraTargetSetExtrinsic(
    const Camera_Target_Extrinsic_s *extrinsic);

/** Clear calibration and any stored capture pose. */
void CameraTargetClearCalibration(void);

/**
 * Build R_E_C from Z-Y-X yaw/pitch/roll angles in degrees. At zero angles the
 * C and E axes are aligned. Translation is still the C origin expressed in E.
 */
Camera_Target_Transform_Status_e CameraTargetBuildExtrinsicFromRpy(
    Camera_Target_Reference_Frame_e reference_frame,
    const float translation_e_from_c_mm[3],
    float roll_deg, float pitch_deg, float yaw_deg,
    uint8_t calibrated,
    Camera_Target_Extrinsic_s *extrinsic);

/**
 * Build T_B_E from a capture-time arm pose. The wrist reference uses the ID1
 * pitch-axis origin and small-link absolute pitch. The tool reference uses the
 * gripper centre and absolute tool pitch. The arm has no controlled roll.
 */
Camera_Target_Transform_Status_e CameraTargetBuildArmPoseSnapshot(
    Camera_Target_Reference_Frame_e reference_frame,
    uint32_t capture_id,
    uint32_t capture_tick_ms,
    const float q_deg[3],
    const float wrist_origin_b_mm[3],
    const float tool_center_b_mm[3],
    float small_link_pitch_deg,
    float tool_pitch_deg,
    Camera_Arm_Pose_Snapshot_s *snapshot);

/** Store an already built capture pose for TransformLatest(). */
Camera_Target_Transform_Status_e CameraTargetStorePoseSnapshot(
    const Camera_Arm_Pose_Snapshot_s *snapshot);

/** Pure transform using explicit calibration and pose values. */
Camera_Target_Transform_Status_e CameraTargetTransformPoint(
    const Camera_Target_Extrinsic_s *extrinsic,
    const Camera_Arm_Pose_Snapshot_s *snapshot,
    const float camera_point_mm[3],
    Camera_Target_Transform_Result_s *result);

/**
 * Transform with the configured extrinsic and stored pose. expected_capture_id
 * 0 disables ID matching. max_pose_age_ms 0 disables the age check.
 */
Camera_Target_Transform_Status_e CameraTargetTransformLatest(
    uint32_t expected_capture_id,
    uint32_t now_ms,
    uint32_t max_pose_age_ms,
    const float camera_point_mm[3],
    Camera_Target_Transform_Result_s *result);

const Camera_Target_Extrinsic_s *CameraTargetGetExtrinsic(void);
const Camera_Arm_Pose_Snapshot_s *CameraTargetGetPoseSnapshot(void);

#endif
