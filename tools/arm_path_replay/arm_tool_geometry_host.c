#include "arm_tool.h"
#include "arm_config.h"

#include <math.h>

#define HOST_PI 3.14159265358979323846f

uint8_t ArmToolGetCenterFromWrist(const Arm_Position_s *wrist,
                                  float base_yaw_deg,
                                  float tool_pitch_deg,
                                  Arm_Position_s *center)
{
    float yaw_rad;
    float pitch_rad;
    float radial_offset;

    if (wrist == NULL || center == NULL ||
        !isfinite(wrist->x_mm) || !isfinite(wrist->y_mm) ||
        !isfinite(wrist->z_mm) || !isfinite(base_yaw_deg) ||
        !isfinite(tool_pitch_deg)) {
        return 0u;
    }
    yaw_rad = base_yaw_deg * HOST_PI / 180.0f;
    pitch_rad = tool_pitch_deg * HOST_PI / 180.0f;
    radial_offset = ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * cosf(pitch_rad);
    center->x_mm = wrist->x_mm + radial_offset * cosf(yaw_rad);
    center->y_mm = wrist->y_mm + radial_offset * sinf(yaw_rad);
    center->z_mm = wrist->z_mm +
        ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * sinf(pitch_rad);
    return 1u;
}

uint8_t ArmToolGetWristFromCenter(const Arm_Position_s *center,
                                  float base_yaw_deg,
                                  float tool_pitch_deg,
                                  Arm_Position_s *wrist)
{
    float yaw_rad;
    float pitch_rad;
    float radial_offset;

    if (center == NULL || wrist == NULL ||
        !isfinite(center->x_mm) || !isfinite(center->y_mm) ||
        !isfinite(center->z_mm) || !isfinite(base_yaw_deg) ||
        !isfinite(tool_pitch_deg)) {
        return 0u;
    }
    yaw_rad = base_yaw_deg * HOST_PI / 180.0f;
    pitch_rad = tool_pitch_deg * HOST_PI / 180.0f;
    radial_offset = ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * cosf(pitch_rad);
    wrist->x_mm = center->x_mm - radial_offset * cosf(yaw_rad);
    wrist->y_mm = center->y_mm - radial_offset * sinf(yaw_rad);
    wrist->z_mm = center->z_mm -
        ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * sinf(pitch_rad);
    return isfinite(wrist->x_mm) && isfinite(wrist->y_mm) &&
           isfinite(wrist->z_mm) ? 1u : 0u;
}

float ArmToolSmallLinkPitchFromJoint(const float q_deg[3])
{
    if (q_deg == NULL || !isfinite(q_deg[1]) || !isfinite(q_deg[2])) {
        return NAN;
    }
    return q_deg[1] + (-180.0f - q_deg[2]);
}

uint8_t ArmToolPitchPositionForPose(float tool_pitch_deg,
                                     float small_link_pitch_deg,
                                     uint16_t *position)
{
    float relative_pitch_deg;
    float position_f;

    if (position == NULL || !isfinite(tool_pitch_deg) ||
        !isfinite(small_link_pitch_deg)) {
        return 0u;
    }
    relative_pitch_deg = tool_pitch_deg - small_link_pitch_deg;
    if (relative_pitch_deg < ARM_TOOL_PITCH_RELATIVE_MIN_DEG ||
        relative_pitch_deg > ARM_TOOL_PITCH_RELATIVE_MAX_DEG) {
        return 0u;
    }
    position_f = (float)ARM_TOOL_PITCH_NEUTRAL_POS +
        ARM_TOOL_PITCH_DIRECTION * relative_pitch_deg *
        (float)(ARM_TOOL_SERVO_POS_MAX - ARM_TOOL_SERVO_POS_MIN) /
        ARM_TOOL_SERVO_RANGE_DEG;
    if (!isfinite(position_f) ||
        position_f < (float)ARM_TOOL_PITCH_SERVO_MIN_POS ||
        position_f > (float)ARM_TOOL_PITCH_SERVO_MAX_POS) {
        return 0u;
    }
    *position = (uint16_t)(position_f + 0.5f);
    return *position >= ARM_TOOL_PITCH_SERVO_MIN_POS &&
           *position <= ARM_TOOL_PITCH_SERVO_MAX_POS;
}

uint8_t ArmToolPitchValidForPose(float tool_pitch_deg,
                                  const float q_deg[3])
{
    uint16_t position;

    return ArmToolPitchPositionForPose(
        tool_pitch_deg, ArmToolSmallLinkPitchFromJoint(q_deg), &position);
}
