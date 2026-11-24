/** @file lift.c
 *
 * @brief Lift detector: reports when the robot is picked up off the floor.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "fix_math.h"
#include "lift.h"


/*!
 * @brief Configure a detector and clear its history.
 *
 * @param[out] p_lift     Detector.
 * @param[in]  tilt_mdeg  Lean from level that counts as being held.
 * @param[in]  hold_ms    How long the lean must persist.
 */
void
lift_init (lift_t * p_lift, int32_t tilt_mdeg, uint32_t hold_ms)
{
    p_lift->tilt_mdeg = tilt_mdeg;
    p_lift->hold_ms   = hold_ms;
    p_lift->tilt_ms   = 0u;
    p_lift->b_lifted  = false;
}


/*!
 * @brief Process one accelerometer sample.
 *
 * @param[in,out] p_lift     Detector.
 * @param[in]     tilt_mdeg  Lean from level in any direction.
 * @param[in]     b_valid    false if the sensor data is not trustworthy.
 * @param[in]     dt_ms      Time since the previous sample.
 *
 * @return true once, on the sample where a lift is first recognised.
 *
 * @par
 * Invalid data clears the timer and never reports a lift, so a sensor
 * fault cannot stop the robot by itself.
 */
bool
lift_update (lift_t * p_lift, int32_t tilt_mdeg, bool b_valid,
             uint32_t dt_ms)
{
    bool b_report = false;

    if (b_valid && (fix_math_abs(tilt_mdeg) >= p_lift->tilt_mdeg))
    {
        p_lift->tilt_ms += dt_ms;

        if ((p_lift->tilt_ms >= p_lift->hold_ms) && (!p_lift->b_lifted))
        {
            p_lift->b_lifted = true;
            b_report         = true;
        }
    }
    else
    {
        p_lift->tilt_ms = 0u;

        if (b_valid)
        {
            // Below the threshold: ready to report the next lift.
            p_lift->b_lifted = false;
        }
    }

    return (b_report);
}


/*!
 * @brief Whether a reported lift is still in progress.
 *
 * @param[in] p_lift  Detector.
 *
 * @return true from the report until the lean has gone.
 */
bool
lift_is_lifted (lift_t const * p_lift)
{
    return (p_lift->b_lifted);
}


/*!
 * @brief Change the lean that counts as being held, keeping the history.
 *
 * @param[in,out] p_lift     Detector.
 * @param[in]     tilt_mdeg  New threshold.
 *
 * @par
 * Lets the caller use a lower threshold while the course cannot tilt the
 * robot much (steady line following) and the normal one elsewhere.
 */
void
lift_set_tilt (lift_t * p_lift, int32_t tilt_mdeg)
{
    p_lift->tilt_mdeg = tilt_mdeg;
}

/*** end of file ***/

