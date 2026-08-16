#include "camera_target_transform.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_TOLERANCE 0.001f

static uint32_t test_count;
static uint32_t failure_count;

static void expect_true(uint8_t condition, const char *name)
{
    test_count++;
    if (condition == 0u) {
        failure_count++;
        printf("FAIL: %s\n", name);
    }
}

static uint8_t near_value(float actual, float expected)
{
    return (uint8_t)(fabsf(actual - expected) <= TEST_TOLERANCE);
}

static uint8_t near_vector(const float actual[3],
                           float x, float y, float z)
{
    return (uint8_t)(near_value(actual[0], x) &&
                     near_value(actual[1], y) &&
                     near_value(actual[2], z));
}

static Camera_Target_Extrinsic_s identity_extrinsic(
    Camera_Target_Reference_Frame_e reference,
    float tx, float ty, float tz)
{
    Camera_Target_Extrinsic_s extrinsic;
    float translation[3] = {tx, ty, tz};

    memset(&extrinsic, 0, sizeof(extrinsic));
    (void)CameraTargetBuildExtrinsicFromRpy(
        reference, translation, 0.0f, 0.0f, 0.0f, 1u, &extrinsic);
    return extrinsic;
}

static Camera_Arm_Pose_Snapshot_s pose_snapshot(
    Camera_Target_Reference_Frame_e reference,
    uint32_t capture_id, uint32_t capture_tick,
    float yaw_deg, float pitch_deg,
    float x, float y, float z)
{
    Camera_Arm_Pose_Snapshot_s snapshot;
    float q_deg[3] = {yaw_deg, 90.0f, -90.0f};
    float wrist[3] = {x, y, z};
    float tool[3] = {x, y, z};

    memset(&snapshot, 0, sizeof(snapshot));
    (void)CameraTargetBuildArmPoseSnapshot(
        reference, capture_id, capture_tick, q_deg, wrist, tool,
        pitch_deg, pitch_deg, &snapshot);
    return snapshot;
}

static void test_identity_and_translation(void)
{
    Camera_Target_Extrinsic_s extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    Camera_Arm_Pose_Snapshot_s snapshot = pose_snapshot(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 1u, 10u,
        0.0f, 0.0f, 100.0f, 200.0f, 300.0f);
    Camera_Target_Transform_Result_s result;
    float point[3] = {10.0f, 20.0f, 30.0f};
    Camera_Target_Transform_Status_e status;

    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK),
                "identity status");
    expect_true(near_vector(result.reference_point_mm,
                            10.0f, 20.0f, 30.0f),
                "identity reference point");
    expect_true(near_vector(result.base_point_mm,
                            110.0f, 220.0f, 330.0f),
                "identity base point");

    extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 5.0f, -2.0f, 7.0f);
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK),
                "translation status");
    expect_true(near_vector(result.reference_point_mm,
                            15.0f, 18.0f, 37.0f),
                "translation reference point");
    expect_true(near_vector(result.base_point_mm,
                            115.0f, 218.0f, 337.0f),
                "translation base point");
}

static void test_rotations(void)
{
    Camera_Target_Extrinsic_s extrinsic;
    Camera_Arm_Pose_Snapshot_s snapshot;
    Camera_Target_Transform_Result_s result;
    float translation[3] = {0.0f, 0.0f, 0.0f};
    float point[3] = {10.0f, 0.0f, 0.0f};
    Camera_Target_Transform_Status_e status;

    status = CameraTargetBuildExtrinsicFromRpy(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, translation,
        0.0f, 0.0f, 90.0f, 1u, &extrinsic);
    snapshot = pose_snapshot(CAMERA_TARGET_REFERENCE_TOOL_CENTER,
                             2u, 20u, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK),
                "camera yaw build");
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK &&
                          near_vector(result.base_point_mm,
                                      0.0f, 10.0f, 0.0f)),
                "camera yaw +90");

    extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    snapshot = pose_snapshot(CAMERA_TARGET_REFERENCE_TOOL_CENTER,
                             3u, 30u, 90.0f, 0.0f,
                             0.0f, 0.0f, 0.0f);
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK &&
                          near_vector(result.base_point_mm,
                                      0.0f, 10.0f, 0.0f)),
                "arm yaw +90");

    snapshot = pose_snapshot(CAMERA_TARGET_REFERENCE_TOOL_CENTER,
                             4u, 40u, 0.0f, 90.0f,
                             0.0f, 0.0f, 0.0f);
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK &&
                          near_vector(result.base_point_mm,
                                      0.0f, 0.0f, 10.0f)),
                "arm pitch +90");

    status = CameraTargetBuildExtrinsicFromRpy(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, translation,
        0.0f, 90.0f, 0.0f, 1u, &extrinsic);
    snapshot = pose_snapshot(CAMERA_TARGET_REFERENCE_TOOL_CENTER,
                             5u, 50u, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f);
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK &&
                          near_vector(result.base_point_mm,
                                      0.0f, 0.0f, -10.0f)),
                "camera pitch +90");

    point[0] = 0.0f;
    point[1] = 10.0f;
    status = CameraTargetBuildExtrinsicFromRpy(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, translation,
        90.0f, 0.0f, 0.0f, 1u, &extrinsic);
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK &&
                          near_vector(result.base_point_mm,
                                      0.0f, 0.0f, 10.0f)),
                "camera roll +90");
}

static void test_left_right_symmetry(void)
{
    Camera_Target_Extrinsic_s extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    Camera_Arm_Pose_Snapshot_s left = pose_snapshot(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 6u, 60u,
        90.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Camera_Arm_Pose_Snapshot_s right = pose_snapshot(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 7u, 70u,
        -90.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Camera_Target_Transform_Result_s left_result;
    Camera_Target_Transform_Result_s right_result;
    float point[3] = {100.0f, 5.0f, -8.0f};

    expect_true((uint8_t)(CameraTargetTransformPoint(
                    &extrinsic, &left, point, &left_result) ==
                CAMERA_TARGET_STATUS_OK),
                "left transform");
    expect_true((uint8_t)(CameraTargetTransformPoint(
                    &extrinsic, &right, point, &right_result) ==
                CAMERA_TARGET_STATUS_OK),
                "right transform");
    expect_true(near_vector(left_result.base_point_mm,
                            -5.0f, 100.0f, -8.0f),
                "left coordinates");
    expect_true(near_vector(right_result.base_point_mm,
                            5.0f, -100.0f, -8.0f),
                "right coordinates");
}

static void test_validation(void)
{
    Camera_Target_Extrinsic_s extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    Camera_Arm_Pose_Snapshot_s snapshot = pose_snapshot(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 8u, 100u,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Camera_Target_Transform_Result_s result;
    float point[3] = {1.0f, 2.0f, 3.0f};
    Camera_Target_Transform_Status_e status;

    extrinsic.calibrated = 0u;
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_NOT_CALIBRATED),
                "uncalibrated rejected");

    extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    extrinsic.rotation_e_from_c[0][0] = -1.0f;
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status ==
                          CAMERA_TARGET_STATUS_ROTATION_BAD_DETERMINANT),
                "reflection rejected");

    extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    extrinsic.rotation_e_from_c[0][1] = 0.2f;
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status ==
                          CAMERA_TARGET_STATUS_ROTATION_NOT_ORTHONORMAL),
                "nonorthogonal rotation rejected");

    extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_WRIST_PITCH_AXIS, 0.0f, 0.0f, 0.0f);
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status ==
                          CAMERA_TARGET_STATUS_REFERENCE_FRAME_MISMATCH),
                "reference mismatch rejected");

    extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    point[0] = NAN;
    status = CameraTargetTransformPoint(&extrinsic, &snapshot,
                                        point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_NONFINITE_VALUE),
                "nonfinite camera point rejected");
}

static void test_latest_snapshot_guards(void)
{
    Camera_Target_Extrinsic_s extrinsic = identity_extrinsic(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 0.0f, 0.0f, 0.0f);
    Camera_Arm_Pose_Snapshot_s snapshot = pose_snapshot(
        CAMERA_TARGET_REFERENCE_TOOL_CENTER, 9u, 1000u,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Camera_Target_Transform_Result_s result;
    float point[3] = {1.0f, 2.0f, 3.0f};
    Camera_Target_Transform_Status_e status;

    CameraTargetTransformInit();
    status = CameraTargetTransformLatest(0u, 1000u, 500u,
                                         point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_NOT_CALIBRATED),
                "default calibration disabled");
    expect_true((uint8_t)(CameraTargetSetExtrinsic(&extrinsic) ==
                          CAMERA_TARGET_STATUS_OK),
                "configure valid extrinsic");
    status = CameraTargetTransformLatest(0u, 1000u, 500u,
                                         point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_NO_POSE_SNAPSHOT),
                "missing snapshot rejected");
    expect_true((uint8_t)(CameraTargetStorePoseSnapshot(&snapshot) ==
                          CAMERA_TARGET_STATUS_OK),
                "store valid snapshot");
    status = CameraTargetTransformLatest(10u, 1100u, 500u,
                                         point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_CAPTURE_ID_MISMATCH),
                "capture id mismatch rejected");
    status = CameraTargetTransformLatest(9u, 1600u, 500u,
                                         point, &result);
    expect_true((uint8_t)(status ==
                          CAMERA_TARGET_STATUS_POSE_SNAPSHOT_EXPIRED),
                "expired snapshot rejected");
    status = CameraTargetTransformLatest(9u, 1400u, 500u,
                                         point, &result);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK &&
                          near_vector(result.base_point_mm,
                                      1.0f, 2.0f, 3.0f)),
                "latest snapshot accepted");
}

static void test_wrist_reference_without_tool_pose(void)
{
    Camera_Arm_Pose_Snapshot_s snapshot;
    float q_deg[3] = {0.0f, 90.0f, -90.0f};
    float wrist[3] = {20.0f, 30.0f, 40.0f};
    float unavailable_tool[3] = {NAN, NAN, NAN};
    Camera_Target_Transform_Status_e status;

    status = CameraTargetBuildArmPoseSnapshot(
        CAMERA_TARGET_REFERENCE_WRIST_PITCH_AXIS,
        11u, 2000u, q_deg, wrist, unavailable_tool,
        0.0f, NAN, &snapshot);
    expect_true((uint8_t)(status == CAMERA_TARGET_STATUS_OK &&
                          near_vector(snapshot.translation_b_from_e_mm,
                                      20.0f, 30.0f, 40.0f)),
                "wrist reference independent of ID1 pose");
}

int main(void)
{
    test_identity_and_translation();
    test_rotations();
    test_left_right_symmetry();
    test_validation();
    test_latest_snapshot_guards();
    test_wrist_reference_without_tool_pose();

    if (failure_count != 0u) {
        printf("camera transform: %lu/%lu checks failed\n",
               (unsigned long)failure_count,
               (unsigned long)test_count);
        return 1;
    }
    printf("camera transform: all %lu checks passed\n",
           (unsigned long)test_count);
    return 0;
}
