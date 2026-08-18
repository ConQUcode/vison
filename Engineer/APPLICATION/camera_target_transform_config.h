/**
 * @file camera_target_transform_config.h
 * @brief Confirmed D435i RGB extrinsic used by AC closed-loop bring-up.
 *
 * The AC closed-loop protocol now captures the arm pose at the observation
 * point before accepting ArmTarget, so the stored transform is enabled here.
 * Re-disable it if known-point hardware validation shows an axis or offset
 * mismatch.
 */

#ifndef CAMERA_TARGET_TRANSFORM_CONFIG_H
#define CAMERA_TARGET_TRANSFORM_CONFIG_H

#define CAMERA_TARGET_DEFAULT_CALIBRATED                 1u

/* The camera is mounted after ID1 and rotates with the gripper absolute pitch. */
#define CAMERA_TARGET_DEFAULT_REFERENCE_FRAME \
    CAMERA_TARGET_REFERENCE_TOOL_CENTER

/*
 * Translation source, all expressed along the confirmed tool-frame XYZ axes:
 *   Fusion ID1 pitch axis -> D435i bottom screw S:
 *       [26.800000, 36.726000, 35.577000] mm
 *   Official D435i screw S -> RGB optical origin C:
 *       [10.144894, 32.458360, 12.615321] mm
 *   Combined ID1 pitch axis -> C:
 *       [36.944894, 69.184360, 48.192321] mm
 *
 * The selected E origin is the gripper centre, located 117 mm from the ID1
 * pitch axis along tool +X (ARM_TOOL_PITCH_AXIS_TO_CENTER_MM). Therefore
 * t_E_C = [36.944894 - 117, 69.184360, 48.192321] mm. Recompute X if the
 * physical 117 mm tool length changes.
 */
#define CAMERA_TARGET_DEFAULT_T_E_C_X_MM              (-80.055106f)
#define CAMERA_TARGET_DEFAULT_T_E_C_Y_MM                 69.184360f
#define CAMERA_TARGET_DEFAULT_T_E_C_Z_MM                 48.192321f

/*
 * R_E_C maps camera_color_optical_frame into the gripper-centre E frame.
 * Columns are RGB optical +X(right), +Y(down), +Z(forward) expressed in E.
 * Axis directions and the official D435i optical transform are confirmed.
 */
#define CAMERA_TARGET_DEFAULT_R_E_C_00                 (-0.00239767f)
#define CAMERA_TARGET_DEFAULT_R_E_C_01                    0.00237706f
#define CAMERA_TARGET_DEFAULT_R_E_C_02                    0.99999400f
#define CAMERA_TARGET_DEFAULT_R_E_C_10                 (-0.99999200f)
#define CAMERA_TARGET_DEFAULT_R_E_C_11                    0.00320883f
#define CAMERA_TARGET_DEFAULT_R_E_C_12                 (-0.00240529f)
#define CAMERA_TARGET_DEFAULT_R_E_C_20                 (-0.00321453f)
#define CAMERA_TARGET_DEFAULT_R_E_C_21                 (-0.99999200f)
#define CAMERA_TARGET_DEFAULT_R_E_C_22                    0.00236935f

/* ArmTarget may only use a recently stored capture-time pose. */
#define CAMERA_TARGET_DEFAULT_MAX_POSE_AGE_MS          5000u

#endif
