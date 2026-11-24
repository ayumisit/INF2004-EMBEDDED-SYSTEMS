/** @file hump.c
 *
 * @brief Hump detector and peak-height estimator.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "fix_math.h"
#include "hump.h"

// A tilt that lasts this many hold times (5 s at 100 ms) is not a hump:
// the robot is stuck, parked on a slope, or being carried.
#define MAX_HOLD_FACTOR         50u

// While on a hump, a fall of this much below the peak counts as coming
// down (DESCENDING); a rise back to within it counts as climbing again.
#define DESCENT_MARGIN_MDEG     1000

static void restart(hump_t * p_hump);
static void abandon(hump_t * p_hump);
static void step_level(hump_t * p_hump, int32_t tilt_mdeg, uint32_t dt_ms);
static bool step_on_hump(hump_t * p_hump, int32_t tilt_mdeg, uint32_t dt_ms,
                         uint32_t * p_height_mm);


/*!
 * @brief Configure a detector and clear its history.
 *
 * @param[out] p_hump      Detector.
 * @param[in]  enter_mdeg  Tilt that starts a hump.
 * @param[in]  exit_mdeg   Tilt below which the robot counts as level.
 * @param[in]  hold_ms     How long a condition must persist.
 * @param[in]  lever_mm    Distance from the lifted wheel to the pivot line.
 */
void
hump_init (hump_t * p_hump, int32_t enter_mdeg, int32_t exit_mdeg,
           uint32_t hold_ms, uint32_t lever_mm)
{
    p_hump->enter_mdeg = enter_mdeg;
    p_hump->exit_mdeg  = exit_mdeg;
    p_hump->hold_ms    = hold_ms;
    p_hump->lever_mm   = lever_mm;
    p_hump->max_mm     = 0u;
    p_hump->count      = 0u;
    p_hump->b_blocked  = false;
    restart(p_hump);
}


/*!
 * @brief Process one sample.
 *
 * @param[in,out] p_hump       Detector.
 * @param[in]     tilt_mdeg    Tilt from level in any direction (>= 0).
 * @param[in]     b_valid      false if the sensor data is not trustworthy.
 * @param[in]     dt_ms        Time since the previous sample.
 * @param[out]    p_height_mm  Height of a completed hump.
 *
 * @return true when a hump has just been completed and *p_height_mm set.
 */
bool
hump_update (hump_t * p_hump, int32_t tilt_mdeg, bool b_valid,
             uint32_t dt_ms, uint32_t * p_height_mm)
{
    int32_t const tilt     = fix_math_abs(tilt_mdeg);
    bool          b_report = false;

    if (!b_valid)
    {
        // Invalid data abandons the hump and never touches the maximum.
        abandon(p_hump);
    }
    else if (p_hump->b_blocked)
    {
        // After an abandoned hump, re-arm only once the robot is level, so
        // the rest of that hump is not reported as a new, partial one.
        p_hump->b_blocked = (tilt >= p_hump->exit_mdeg);
    }
    else if (HUMP_STATE_LEVEL == p_hump->state)
    {
        step_level(p_hump, tilt, dt_ms);
    }
    else
    {
        b_report = step_on_hump(p_hump, tilt, dt_ms, p_height_mm);
    }

    return (b_report);
}


/*!
 * @brief Current detector state.
 *
 * @param[in] p_hump  Detector.
 *
 * @return LEVEL, CLIMBING or DESCENDING.
 */
hump_state_t
hump_state (hump_t const * p_hump)
{
    return (p_hump->state);
}


/*!
 * @brief Wheel lift for a tilt: lever x sin(tilt).
 *
 * @param[in] p_hump     Detector (for the lever length).
 * @param[in] tilt_mdeg  Tilt in millidegrees.
 *
 * @return Height in mm, never negative.
 */
uint32_t
hump_height_mm (hump_t const * p_hump, int32_t tilt_mdeg)
{
    int32_t const sine = fix_math_sin_q15(fix_math_abs(tilt_mdeg));

    // The lever is below 2^10 mm and |sine| at most 2^15, so the product
    // fits; a hump peak is far below 180 degrees, so the sine and the
    // result are not negative and the cast is exact.
    return ((uint32_t) (((int32_t) p_hump->lever_mm * sine) /
                        FIX_MATH_Q15_ONE));
}


/*!
 * @brief Highest completed hump since start-up.
 *
 * @param[in] p_hump  Detector.
 *
 * @return Height in mm.
 */
uint32_t
hump_max_mm (hump_t const * p_hump)
{
    return (p_hump->max_mm);
}


/*!
 * @brief Number of completed humps since start-up.
 *
 * @param[in] p_hump  Detector.
 *
 * @return Hump count.
 */
uint32_t
hump_count (hump_t const * p_hump)
{
    return (p_hump->count);
}


/*!
 * @brief Return to LEVEL with no hump in progress.
 *
 * @param[out] p_hump  Detector.
 */
static void
restart (hump_t * p_hump)
{
    p_hump->state     = HUMP_STATE_LEVEL;
    p_hump->timer_ms  = 0u;
    p_hump->hump_ms   = 0u;
    p_hump->peak_mdeg = 0;
}


/*!
 * @brief Drop the hump in progress and wait for level before re-arming.
 *
 * @param[out] p_hump  Detector.
 */
static void
abandon (hump_t * p_hump)
{
    restart(p_hump);
    p_hump->b_blocked = true;
}


/*!
 * @brief LEVEL: start CLIMBING once the tilt persists above the threshold.
 *
 * @param[in,out] p_hump     Detector.
 * @param[in]     tilt_mdeg  Tilt (>= 0).
 * @param[in]     dt_ms      Sample interval.
 */
static void
step_level (hump_t * p_hump, int32_t tilt_mdeg, uint32_t dt_ms)
{
    if (tilt_mdeg > p_hump->enter_mdeg)
    {
        p_hump->timer_ms  += dt_ms;
        p_hump->hump_ms   += dt_ms;
        p_hump->peak_mdeg  = (tilt_mdeg > p_hump->peak_mdeg) ?
                             tilt_mdeg : p_hump->peak_mdeg;

        if (p_hump->timer_ms >= p_hump->hold_ms)
        {
            p_hump->state    = HUMP_STATE_CLIMBING;
            p_hump->timer_ms = 0u;
        }
    }
    else
    {
        restart(p_hump);
    }
}


/*!
 * @brief CLIMBING / DESCENDING: track the peak and report once level.
 *
 * @param[in,out] p_hump       Detector.
 * @param[in]     tilt_mdeg    Tilt (>= 0).
 * @param[in]     dt_ms        Sample interval.
 * @param[out]    p_height_mm  Height of the completed hump.
 *
 * @return true when the hump has just been completed.
 */
static bool
step_on_hump (hump_t * p_hump, int32_t tilt_mdeg, uint32_t dt_ms,
              uint32_t * p_height_mm)
{
    bool b_done = false;

    p_hump->hump_ms  += dt_ms;
    p_hump->peak_mdeg = (tilt_mdeg > p_hump->peak_mdeg) ?
                        tilt_mdeg : p_hump->peak_mdeg;
    p_hump->state     = (tilt_mdeg < (p_hump->peak_mdeg -
                                      DESCENT_MARGIN_MDEG)) ?
                        HUMP_STATE_DESCENDING : HUMP_STATE_CLIMBING;
    p_hump->timer_ms  = (tilt_mdeg < p_hump->exit_mdeg) ?
                        (p_hump->timer_ms + dt_ms) : 0u;

    if (p_hump->hump_ms >= (p_hump->hold_ms * MAX_HOLD_FACTOR))
    {
        // Tilted for too long to be a hump.
        abandon(p_hump);
    }
    else if (p_hump->timer_ms >= p_hump->hold_ms)
    {
        uint32_t const height = hump_height_mm(p_hump, p_hump->peak_mdeg);

        *p_height_mm   = height;
        p_hump->max_mm = (height > p_hump->max_mm) ? height : p_hump->max_mm;
        p_hump->count++;
        restart(p_hump);
        b_done = true;
    }
    else
    {
        // Still on the hump.
    }

    return (b_done);
}

/*** end of file ***/

