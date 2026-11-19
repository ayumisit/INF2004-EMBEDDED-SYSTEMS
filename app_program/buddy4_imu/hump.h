/** @file hump.h
 *
 * @brief Hump detector and peak-height estimator (Buddy 4, Week 6
 *        Section 6.5).
 *
 * @par
 * Pure module (no kernel or hardware dependency), unit-tested on the host.
 * The input is the tilt: how far the robot leans from its calibrated level
 * in any direction (pitch and roll combined), so it is never negative and
 * it does not matter which wheel crosses the hump or how the IMU is turned
 * on the chassis.  A hump is LEVEL, then CLIMBING while the tilt rises,
 * DESCENDING while it falls, then LEVEL again, when it is reported.
 *
 * @par
 * Height: the course hump is crossed by one drive wheel, so the chassis
 * pivots about the line through the other wheel and the caster.  The lifted
 * wheel then rises lever x sin(peak tilt), where the lever is that wheel's
 * distance from the pivot line.  The peak tilt is never negative, so
 * neither is the height.  An invalid sample, or a tilt that lasts longer
 * than a hump can, abandons the hump, so bad data never updates the running
 * maximum (assumption A4).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef HUMP_H
#define HUMP_H

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief Hump states (2 bits).
 */
typedef enum
{
    HUMP_STATE_LEVEL = 0,
    HUMP_STATE_CLIMBING,
    HUMP_STATE_DESCENDING
} hump_state_t;

/*!
 * @brief Detector configuration and state.
 */
typedef struct
{
    int32_t         enter_mdeg;     // Tilt that starts a hump.
    int32_t         exit_mdeg;      // Tilt below which the robot is level.
    uint32_t        hold_ms;        // Time a condition must persist.
    uint32_t        lever_mm;       // Lifted wheel to pivot line.
    hump_state_t    state;
    uint32_t        timer_ms;       // Time the transition condition held.
    uint32_t        hump_ms;        // Time since the hump started.
    int32_t         peak_mdeg;      // Largest tilt on this hump.
    bool            b_blocked;      // Abandoned: wait for level first.
    uint32_t        max_mm;
    uint32_t        count;
} hump_t;

void         hump_init(hump_t * p_hump, int32_t enter_mdeg, int32_t exit_mdeg,
                       uint32_t hold_ms, uint32_t lever_mm);
bool         hump_update(hump_t * p_hump, int32_t tilt_mdeg, bool b_valid,
                         uint32_t dt_ms, uint32_t * p_height_mm);
hump_state_t hump_state(hump_t const * p_hump);
uint32_t     hump_height_mm(hump_t const * p_hump, int32_t tilt_mdeg);
uint32_t     hump_max_mm(hump_t const * p_hump);
uint32_t     hump_count(hump_t const * p_hump);

#endif /* HUMP_H */

/*** end of file ***/

