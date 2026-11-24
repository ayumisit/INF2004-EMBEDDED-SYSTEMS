/** @file imu.h
 *
 * @brief IMU Task: tilt, heading, motion classification, hump detection
 *        and impact detection (Buddy 4, Week 6 Section 6.5).
 *
 * @par
 * Status word SYS_STATUS_IMU (LSB first): [7:0] tilt (pitch and roll
 * combined, never negative) in 0.5 degree units (excess-128), the field
 * named IMU_PITCH; [16:8] heading 0..359 degrees, [19:17] motion class,
 * [21:20] hump state, [22] data valid, [26:23] impact count (saturating).
 *
 * @par
 * Status word SYS_STATUS_HUMP (LSB first): [7:0] last hump height mm,
 * [15:8] highest hump mm (FR5 running maximum), [21:16] hump count.
 *
 * @par
 * Lift detection: ROBOT_EVT_LIFTED is raised on flg_vehicle_evt when the
 * robot is picked up (lift.h).  The lift count is in no status word:
 * the USB monitor reads imu_lift_count(), and telemetry reports each
 * lift as an event.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#define IMU_PITCH_SHIFT             0u
#define IMU_PITCH_WIDTH             8u
#define IMU_HEADING_SHIFT           8u
#define IMU_HEADING_WIDTH           9u
#define IMU_CLASS_SHIFT             17u
#define IMU_CLASS_WIDTH             3u
#define IMU_HUMP_SHIFT              20u
#define IMU_HUMP_WIDTH              2u
#define IMU_VALID_SHIFT             22u
#define IMU_IMPACTS_SHIFT           23u
#define IMU_IMPACTS_WIDTH           4u
#define IMU_HUMP_LAST_SHIFT         0u
#define IMU_HUMP_MAX_SHIFT          8u
#define IMU_HUMP_MM_WIDTH           8u
#define IMU_HUMP_COUNT_SHIFT        16u
#define IMU_HUMP_COUNT_WIDTH        6u

/*!
 * @brief Motion classes (3 bits).
 */
typedef enum
{
    IMU_CLASS_STATIONARY = 0,
    IMU_CLASS_CRUISING,
    IMU_CLASS_ACCELERATING,
    IMU_CLASS_DECELERATING,
    IMU_CLASS_TURNING,
    IMU_CLASS_CLIMBING,
    IMU_CLASS_DESCENDING,
    IMU_CLASS_IMPACT
} imu_class_t;

int32_t  imu_start(void);
uint32_t imu_lift_count(void);

#endif /* IMU_H */

/*** end of file ***/

