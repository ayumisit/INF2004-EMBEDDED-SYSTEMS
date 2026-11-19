/** @file lift.h
 *
 * @brief Lift detector: reports when the robot is picked up off the floor
 *        (Buddy 4, IMU requirement added in Week 12).
 *
 * @par
 * Pure module (no kernel or hardware dependency), unit-tested on the host.
 * A robot picked up and tipped in a hand leans further than anything on
 * the course tilts it: the hump under one wheel, and the lean read during
 * turns and braking, all stay well below the threshold.  The lean must
 * persist for the hold time, so a single jolt is never a lift.
 *
 * @par
 * One lift is reported once.  The detector re-arms when the lean has gone,
 * so putting the robot down and lifting it again reports again.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef LIFT_H
#define LIFT_H

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief Lift detector state.
 */
typedef struct
{
    int32_t     tilt_mdeg;      // Lean that counts as being held.
    uint32_t    hold_ms;        // Time the lean must persist.
    uint32_t    tilt_ms;        // Time the lean has persisted.
    bool        b_lifted;       // A lift has been reported and not ended.
} lift_t;

void    lift_init(lift_t * p_lift, int32_t tilt_mdeg, uint32_t hold_ms);
bool    lift_update(lift_t * p_lift, int32_t tilt_mdeg, bool b_valid,
                    uint32_t dt_ms);
bool    lift_is_lifted(lift_t const * p_lift);
void    lift_set_tilt(lift_t * p_lift, int32_t tilt_mdeg);

#endif /* LIFT_H */

/*** end of file ***/

