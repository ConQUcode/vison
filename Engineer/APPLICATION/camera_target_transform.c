/**
 * @file camera_target_transform.c
 * @brief Validated camera-to-arm-base rigid point transformation.
 */

#include "camera_target_transform.h"

#include <math.h>
#include <string.h>

#include "camera_target_transform_config.h"

#define CAMERA_TARGET_PI                         3.14159265358979323846f
#define CAMERA_TARGET_DEG_TO_RAD                 (CAMERA_TARGET_PI / 180.0f)
#define CAMERA_TARGET_ROTATION_TOLERANCE          0.01f
#define CAMERA_TARGET_DETERMINANT_TOLERANCE       0.01f
#define CAMERA_TARGET_MAX_EXTRINSIC_MM         2000.0f
#define CAMERA_TARGET_MAX_POINT_MM            20000.0f

Camera_Target_Transform_Debug_s g_camera_target_transform_debug;

static uint8_t CameraTargetReferenceFrameValid(
    Camera_Target_Reference_Frame_e reference_frame)
{
    return (uint8_t)(
        reference_frame == CAMERA_TARGET_REFERENCE_WRIST_PITCH_AXIS ||
        reference_frame == CAMERA_TARGET_REFERENCE_TOOL_CENTER);
}

static uint8_t CameraTargetVectorFinite(const float value[3])
{
    return (uint8_t)(value != NULL &&
        isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]));
}

static void CameraTargetMatrixVectorMultiply(
    const float matrix[3][3], const float input[3], float output[3])
{
    uint8_t row;

    for (row = 0u; row < 3u; ++row) {
        output[row] = matrix[row][0] * input[0] +
                      matrix[row][1] * input[1] +
                      matrix[row][2] * input[2];
    }
}

static Camera_Target_Transform_Status_e CameraTargetValidateRotation(
    const float rotation[3][3], float *orthogonality_error,
    float *determinant)
{
    float maximum_error = 0.0f;
    float det;
    uint8_t row;
    uint8_t column;
    uint8_t element;

    if (rotation == NULL) {
        return CAMERA_TARGET_STATUS_NULL_ARGUMENT;
    }
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            if (!isfinite(rotation[row][column])) {
                return CAMERA_TARGET_STATUS_NONFINITE_VALUE;
            }
        }
    }
    for (row = 0u; row < 3u; ++row) {
        for (column = 0u; column < 3u; ++column) {
            float dot = 0.0f;
            float expected = row == column ? 1.0f : 0.0f;
            float error;

            for (element = 0u; element < 3u; ++element) {
                dot += rotation[element][row] *
                       rotation[element][column];
            }
            error = fabsf(dot - expected);
            if (error > maximum_error) {
                maximum_error = error;
            }
        }
    }
    det =
        rotation[0][0] *
            (rotation[1][1] * rotation[2][2] -
             rotation[1][2] * rotation[2][1]) -
        rotation[0][1] *
            (rotation[1][0] * rotation[2][2] -
             rotation[1][2] * rotation[2][0]) +
        rotation[0][2] *
            (rotation[1][0] * rotation[2][1] -
             rotation[1][1] * rotation[2][0]);
    if (orthogonality_error != NULL) {
        *orthogonality_error = maximum_error;
    }
    if (determinant != NULL) {
        *determinant = det;
    }
    if (maximum_error > CAMERA_TARGET_ROTATION_TOLERANCE) {
        return CAMERA_TARGET_STATUS_ROTATION_NOT_ORTHONORMAL;
    }
    if (fabsf(det - 1.0f) > CAMERA_TARGET_DETERMINANT_TOLERANCE) {
        return CAMERA_TARGET_STATUS_ROTATION_BAD_DETERMINANT;
    }
    return CAMERA_TARGET_STATUS_OK;
}

static Camera_Target_Transform_Status_e CameraTargetValidateExtrinsic(
    const Camera_Target_Extrinsic_s *extrinsic,
    float *orthogonality_error, float *determinant)
{
    Camera_Target_Transform_Status_e status;
    uint8_t axis;

    if (extrinsic == NULL) {
        return CAMERA_TARGET_STATUS_NULL_ARGUMENT;
    }
    if (extrinsic->calibrated == 0u) {
        return CAMERA_TARGET_STATUS_NOT_CALIBRATED;
    }
    if (!CameraTargetReferenceFrameValid(extrinsic->reference_frame)) {
        return CAMERA_TARGET_STATUS_INVALID_REFERENCE_FRAME;
    }
    if (!CameraTargetVectorFinite(extrinsic->translation_e_from_c_mm)) {
        return CAMERA_TARGET_STATUS_NONFINITE_VALUE;
    }
    for (axis = 0u; axis < 3u; ++axis) {
        if (fabsf(extrinsic->translation_e_from_c_mm[axis]) >
            CAMERA_TARGET_MAX_EXTRINSIC_MM) {
            return CAMERA_TARGET_STATUS_POINT_OUT_OF_RANGE;
        }
    }
    status = CameraTargetValidateRotation(extrinsic->rotation_e_from_c,
                                          orthogonality_error,
                                          determinant);
    return status;
}

static Camera_Target_Transform_Status_e CameraTargetValidateSnapshot(
    const Camera_Arm_Pose_Snapshot_s *snapshot)
{
    Camera_Target_Transform_Status_e status;

    if (snapshot == NULL) {
        return CAMERA_TARGET_STATUS_NULL_ARGUMENT;
    }
    if (snapshot->valid == 0u || snapshot->capture_id == 0u) {
        return CAMERA_TARGET_STATUS_INVALID_POSE;
    }
    if (!CameraTargetReferenceFrameValid(snapshot->reference_frame)) {
        return CAMERA_TARGET_STATUS_INVALID_REFERENCE_FRAME;
    }
    if (!CameraTargetVectorFinite(snapshot->q_deg) ||
        !isfinite(snapshot->reference_pitch_deg) ||
        !CameraTargetVectorFinite(snapshot->translation_b_from_e_mm)) {
        return CAMERA_TARGET_STATUS_NONFINITE_VALUE;
    }
    status = CameraTargetValidateRotation(snapshot->rotation_b_from_e,
                                          NULL, NULL);
    return status == CAMERA_TARGET_STATUS_OK ?
        CAMERA_TARGET_STATUS_OK : CAMERA_TARGET_STATUS_INVALID_POSE;
}

static void CameraTargetBuildYawPitchRotation(
    float yaw_deg, float pitch_deg, float rotation_b_from_e[3][3])
{
    float yaw_rad = yaw_deg * CAMERA_TARGET_DEG_TO_RAD;
    float pitch_rad = pitch_deg * CAMERA_TARGET_DEG_TO_RAD;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);

    /* Columns are E forward, E left and E up expressed in B. */
    rotation_b_from_e[0][0] = cos_pitch * cos_yaw;
    rotation_b_from_e[1][0] = cos_pitch * sin_yaw;
    rotation_b_from_e[2][0] = sin_pitch;
    rotation_b_from_e[0][1] = -sin_yaw;
    rotation_b_from_e[1][1] = cos_yaw;
    rotation_b_from_e[2][1] = 0.0f;
    rotation_b_from_e[0][2] = -sin_pitch * cos_yaw;
    rotation_b_from_e[1][2] = -sin_pitch * sin_yaw;
    rotation_b_from_e[2][2] = cos_pitch;
}

void CameraTargetTransformInit(void)
{
    Camera_Target_Extrinsic_s extrinsic;

    memset(&g_camera_target_transform_debug, 0,
           sizeof(g_camera_target_transform_debug));
    memset(&extrinsic, 0, sizeof(extrinsic));
    extrinsic.reference_frame = CAMERA_TARGET_DEFAULT_REFERENCE_FRAME;
    extrinsic.calibrated = CAMERA_TARGET_DEFAULT_CALIBRATED;
    extrinsic.translation_e_from_c_mm[0] =
        CAMERA_TARGET_DEFAULT_T_E_C_X_MM;
    extrinsic.translation_e_from_c_mm[1] =
        CAMERA_TARGET_DEFAULT_T_E_C_Y_MM;
    extrinsic.translation_e_from_c_mm[2] =
        CAMERA_TARGET_DEFAULT_T_E_C_Z_MM;
    extrinsic.rotation_e_from_c[0][0] = CAMERA_TARGET_DEFAULT_R_E_C_00;
    extrinsic.rotation_e_from_c[0][1] = CAMERA_TARGET_DEFAULT_R_E_C_01;
    extrinsic.rotation_e_from_c[0][2] = CAMERA_TARGET_DEFAULT_R_E_C_02;
    extrinsic.rotation_e_from_c[1][0] = CAMERA_TARGET_DEFAULT_R_E_C_10;
    extrinsic.rotation_e_from_c[1][1] = CAMERA_TARGET_DEFAULT_R_E_C_11;
    extrinsic.rotation_e_from_c[1][2] = CAMERA_TARGET_DEFAULT_R_E_C_12;
    extrinsic.rotation_e_from_c[2][0] = CAMERA_TARGET_DEFAULT_R_E_C_20;
    extrinsic.rotation_e_from_c[2][1] = CAMERA_TARGET_DEFAULT_R_E_C_21;
    extrinsic.rotation_e_from_c[2][2] = CAMERA_TARGET_DEFAULT_R_E_C_22;
    g_camera_target_transform_debug.initialized = 1u;
    g_camera_target_transform_debug.extrinsic = extrinsic;
    if (extrinsic.calibrated != 0u) {
        g_camera_target_transform_debug.last_status =
            CameraTargetSetExtrinsic(&extrinsic);
    } else {
        g_camera_target_transform_debug.last_status =
            CAMERA_TARGET_STATUS_NOT_CALIBRATED;
    }
}

Camera_Target_Transform_Status_e CameraTargetSetExtrinsic(
    const Camera_Target_Extrinsic_s *extrinsic)
{
    Camera_Target_Transform_Status_e status;
    float orthogonality_error = 0.0f;
    float determinant = 0.0f;

    if (g_camera_target_transform_debug.initialized == 0u) {
        return CAMERA_TARGET_STATUS_NOT_INITIALIZED;
    }
    status = CameraTargetValidateExtrinsic(extrinsic,
                                           &orthogonality_error,
                                           &determinant);
    g_camera_target_transform_debug.rotation_orthogonality_error =
        orthogonality_error;
    g_camera_target_transform_debug.rotation_determinant = determinant;
    g_camera_target_transform_debug.last_status = status;
    if (status != CAMERA_TARGET_STATUS_OK) {
        g_camera_target_transform_debug.extrinsic_configured = 0u;
        g_camera_target_transform_debug.extrinsic_reject_count++;
        return status;
    }
    g_camera_target_transform_debug.extrinsic = *extrinsic;
    g_camera_target_transform_debug.extrinsic_configured = 1u;
    g_camera_target_transform_debug.extrinsic_accept_count++;
    return CAMERA_TARGET_STATUS_OK;
}

void CameraTargetClearCalibration(void)
{
    Camera_Target_Reference_Frame_e reference_frame =
        g_camera_target_transform_debug.extrinsic.reference_frame;

    memset(&g_camera_target_transform_debug.extrinsic, 0,
           sizeof(g_camera_target_transform_debug.extrinsic));
    g_camera_target_transform_debug.extrinsic.reference_frame =
        reference_frame;
    memset(&g_camera_target_transform_debug.pose_snapshot, 0,
           sizeof(g_camera_target_transform_debug.pose_snapshot));
    memset(&g_camera_target_transform_debug.last_result, 0,
           sizeof(g_camera_target_transform_debug.last_result));
    g_camera_target_transform_debug.extrinsic_configured = 0u;
    g_camera_target_transform_debug.pose_snapshot_valid = 0u;
    g_camera_target_transform_debug.last_status =
        CAMERA_TARGET_STATUS_NOT_CALIBRATED;
    g_camera_target_transform_debug.last_result.status =
        CAMERA_TARGET_STATUS_NOT_CALIBRATED;
}

Camera_Target_Transform_Status_e CameraTargetBuildExtrinsicFromRpy(
    Camera_Target_Reference_Frame_e reference_frame,
    const float translation_e_from_c_mm[3],
    float roll_deg, float pitch_deg, float yaw_deg,
    uint8_t calibrated,
    Camera_Target_Extrinsic_s *extrinsic)
{
    float roll;
    float pitch;
    float yaw;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float sin_yaw;
    float cos_yaw;

    if (translation_e_from_c_mm == NULL || extrinsic == NULL) {
        return CAMERA_TARGET_STATUS_NULL_ARGUMENT;
    }
    if (!CameraTargetReferenceFrameValid(reference_frame)) {
        return CAMERA_TARGET_STATUS_INVALID_REFERENCE_FRAME;
    }
    if (!CameraTargetVectorFinite(translation_e_from_c_mm) ||
        !isfinite(roll_deg) || !isfinite(pitch_deg) ||
        !isfinite(yaw_deg)) {
        return CAMERA_TARGET_STATUS_NONFINITE_VALUE;
    }
    memset(extrinsic, 0, sizeof(*extrinsic));
    extrinsic->reference_frame = reference_frame;
    extrinsic->calibrated = calibrated != 0u ? 1u : 0u;
    memcpy(extrinsic->translation_e_from_c_mm,
           translation_e_from_c_mm,
           sizeof(extrinsic->translation_e_from_c_mm));
    roll = roll_deg * CAMERA_TARGET_DEG_TO_RAD;
    pitch = pitch_deg * CAMERA_TARGET_DEG_TO_RAD;
    yaw = yaw_deg * CAMERA_TARGET_DEG_TO_RAD;
    sin_roll = sinf(roll);
    cos_roll = cosf(roll);
    sin_pitch = sinf(pitch);
    cos_pitch = cosf(pitch);
    sin_yaw = sinf(yaw);
    cos_yaw = cosf(yaw);

    /* R_E_C = Rz(yaw) * Ry(pitch) * Rx(roll). */
    extrinsic->rotation_e_from_c[0][0] = cos_yaw * cos_pitch;
    extrinsic->rotation_e_from_c[0][1] =
        cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    extrinsic->rotation_e_from_c[0][2] =
        cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    extrinsic->rotation_e_from_c[1][0] = sin_yaw * cos_pitch;
    extrinsic->rotation_e_from_c[1][1] =
        sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    extrinsic->rotation_e_from_c[1][2] =
        sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    extrinsic->rotation_e_from_c[2][0] = -sin_pitch;
    extrinsic->rotation_e_from_c[2][1] = cos_pitch * sin_roll;
    extrinsic->rotation_e_from_c[2][2] = cos_pitch * cos_roll;
    return calibrated != 0u ?
        CameraTargetValidateExtrinsic(extrinsic, NULL, NULL) :
        CAMERA_TARGET_STATUS_NOT_CALIBRATED;
}

Camera_Target_Transform_Status_e CameraTargetBuildArmPoseSnapshot(
    Camera_Target_Reference_Frame_e reference_frame,
    uint32_t capture_id,
    uint32_t capture_tick_ms,
    const float q_deg[3],
    const float wrist_origin_b_mm[3],
    const float tool_center_b_mm[3],
    float small_link_pitch_deg,
    float tool_pitch_deg,
    Camera_Arm_Pose_Snapshot_s *snapshot)
{
    const float *origin;
    float pitch_deg;

    if (q_deg == NULL || wrist_origin_b_mm == NULL ||
        tool_center_b_mm == NULL || snapshot == NULL) {
        return CAMERA_TARGET_STATUS_NULL_ARGUMENT;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    if (!CameraTargetReferenceFrameValid(reference_frame)) {
        return CAMERA_TARGET_STATUS_INVALID_REFERENCE_FRAME;
    }
    if (capture_id == 0u || !CameraTargetVectorFinite(q_deg)) {
        return CAMERA_TARGET_STATUS_INVALID_POSE;
    }
    if (reference_frame == CAMERA_TARGET_REFERENCE_WRIST_PITCH_AXIS) {
        if (!CameraTargetVectorFinite(wrist_origin_b_mm) ||
            !isfinite(small_link_pitch_deg)) {
            return CAMERA_TARGET_STATUS_INVALID_POSE;
        }
        origin = wrist_origin_b_mm;
        pitch_deg = small_link_pitch_deg;
    } else {
        if (!CameraTargetVectorFinite(tool_center_b_mm) ||
            !isfinite(tool_pitch_deg)) {
            return CAMERA_TARGET_STATUS_INVALID_POSE;
        }
        origin = tool_center_b_mm;
        pitch_deg = tool_pitch_deg;
    }
    snapshot->capture_id = capture_id;
    snapshot->capture_tick_ms = capture_tick_ms;
    snapshot->reference_frame = reference_frame;
    memcpy(snapshot->q_deg, q_deg, sizeof(snapshot->q_deg));
    memcpy(snapshot->translation_b_from_e_mm, origin,
           sizeof(snapshot->translation_b_from_e_mm));
    snapshot->reference_pitch_deg = pitch_deg;
    CameraTargetBuildYawPitchRotation(q_deg[0], pitch_deg,
                                      snapshot->rotation_b_from_e);
    snapshot->valid = 1u;
    return CameraTargetValidateSnapshot(snapshot);
}

Camera_Target_Transform_Status_e CameraTargetStorePoseSnapshot(
    const Camera_Arm_Pose_Snapshot_s *snapshot)
{
    Camera_Target_Transform_Status_e status;

    if (g_camera_target_transform_debug.initialized == 0u) {
        return CAMERA_TARGET_STATUS_NOT_INITIALIZED;
    }
    status = CameraTargetValidateSnapshot(snapshot);
    g_camera_target_transform_debug.last_status = status;
    if (status != CAMERA_TARGET_STATUS_OK) {
        g_camera_target_transform_debug.pose_snapshot_valid = 0u;
        g_camera_target_transform_debug.pose_reject_count++;
        return status;
    }
    g_camera_target_transform_debug.pose_snapshot = *snapshot;
    g_camera_target_transform_debug.pose_snapshot_valid = 1u;
    g_camera_target_transform_debug.pose_store_count++;
    return CAMERA_TARGET_STATUS_OK;
}

Camera_Target_Transform_Status_e CameraTargetTransformPoint(
    const Camera_Target_Extrinsic_s *extrinsic,
    const Camera_Arm_Pose_Snapshot_s *snapshot,
    const float camera_point_mm[3],
    Camera_Target_Transform_Result_s *result)
{
    Camera_Target_Transform_Status_e status;
    float reference_rotated[3];
    float base_rotated[3];
    uint8_t axis;

    if (result == NULL) {
        return CAMERA_TARGET_STATUS_NULL_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    status = CameraTargetValidateExtrinsic(extrinsic, NULL, NULL);
    if (status != CAMERA_TARGET_STATUS_OK) {
        result->status = status;
        return status;
    }
    status = CameraTargetValidateSnapshot(snapshot);
    if (status != CAMERA_TARGET_STATUS_OK) {
        result->status = status;
        return status;
    }
    if (extrinsic->reference_frame != snapshot->reference_frame) {
        result->status = CAMERA_TARGET_STATUS_REFERENCE_FRAME_MISMATCH;
        return result->status;
    }
    if (!CameraTargetVectorFinite(camera_point_mm)) {
        result->status = CAMERA_TARGET_STATUS_NONFINITE_VALUE;
        return result->status;
    }
    for (axis = 0u; axis < 3u; ++axis) {
        if (fabsf(camera_point_mm[axis]) > CAMERA_TARGET_MAX_POINT_MM) {
            result->status = CAMERA_TARGET_STATUS_POINT_OUT_OF_RANGE;
            return result->status;
        }
    }
    result->capture_id = snapshot->capture_id;
    result->capture_tick_ms = snapshot->capture_tick_ms;
    memcpy(result->camera_point_mm, camera_point_mm,
           sizeof(result->camera_point_mm));
    CameraTargetMatrixVectorMultiply(extrinsic->rotation_e_from_c,
                                     camera_point_mm,
                                     reference_rotated);
    for (axis = 0u; axis < 3u; ++axis) {
        result->reference_point_mm[axis] = reference_rotated[axis] +
            extrinsic->translation_e_from_c_mm[axis];
    }
    CameraTargetMatrixVectorMultiply(snapshot->rotation_b_from_e,
                                     result->reference_point_mm,
                                     base_rotated);
    for (axis = 0u; axis < 3u; ++axis) {
        result->base_point_mm[axis] = base_rotated[axis] +
            snapshot->translation_b_from_e_mm[axis];
    }
    if (!CameraTargetVectorFinite(result->reference_point_mm) ||
        !CameraTargetVectorFinite(result->base_point_mm)) {
        result->status = CAMERA_TARGET_STATUS_RESULT_NONFINITE;
        return result->status;
    }
    result->status = CAMERA_TARGET_STATUS_OK;
    result->valid = 1u;
    return CAMERA_TARGET_STATUS_OK;
}

Camera_Target_Transform_Status_e CameraTargetTransformLatest(
    uint32_t expected_capture_id,
    uint32_t now_ms,
    uint32_t max_pose_age_ms,
    const float camera_point_mm[3],
    Camera_Target_Transform_Result_s *result)
{
    Camera_Target_Transform_Status_e status;

    if (result == NULL) {
        return CAMERA_TARGET_STATUS_NULL_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    if (g_camera_target_transform_debug.initialized == 0u) {
        status = CAMERA_TARGET_STATUS_NOT_INITIALIZED;
    } else if (g_camera_target_transform_debug.extrinsic_configured == 0u) {
        status = CAMERA_TARGET_STATUS_NOT_CALIBRATED;
    } else if (g_camera_target_transform_debug.pose_snapshot_valid == 0u) {
        status = CAMERA_TARGET_STATUS_NO_POSE_SNAPSHOT;
    } else if (expected_capture_id != 0u &&
               expected_capture_id !=
                   g_camera_target_transform_debug.pose_snapshot.capture_id) {
        status = CAMERA_TARGET_STATUS_CAPTURE_ID_MISMATCH;
    } else if (max_pose_age_ms != 0u &&
               (uint32_t)(now_ms -
                   g_camera_target_transform_debug.pose_snapshot.
                       capture_tick_ms) > max_pose_age_ms) {
        status = CAMERA_TARGET_STATUS_POSE_SNAPSHOT_EXPIRED;
    } else {
        status = CameraTargetTransformPoint(
            &g_camera_target_transform_debug.extrinsic,
            &g_camera_target_transform_debug.pose_snapshot,
            camera_point_mm, result);
    }
    result->status = status;
    g_camera_target_transform_debug.last_status = status;
    g_camera_target_transform_debug.last_result = *result;
    if (status == CAMERA_TARGET_STATUS_OK) {
        g_camera_target_transform_debug.transform_success_count++;
    } else {
        g_camera_target_transform_debug.transform_fail_count++;
    }
    return status;
}

const Camera_Target_Extrinsic_s *CameraTargetGetExtrinsic(void)
{
    return &g_camera_target_transform_debug.extrinsic;
}

const Camera_Arm_Pose_Snapshot_s *CameraTargetGetPoseSnapshot(void)
{
    return &g_camera_target_transform_debug.pose_snapshot;
}
