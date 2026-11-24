/** @file test_main.c
 *
 * @brief Host unit tests for the hardware-independent robot modules.
 *
 * @par
 * Build and run on a PC (see tests/Makefile):  make -C tests
 * The modules under test are compiled unchanged from app_program/.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "bitpack.h"
#include "fix_math.h"
#include "code39.h"
#include "hump.h"
#include "lift.h"
#include "obs_profile.h"
#include "tlm_codec.h"
#include "tune.h"
#include "nav_map.h"
#include "pid_ctrl.h"
#include "sys_status.h"
#include "robot_cfg.h"

#define NARROW_TICKS            40u
#define WIDE_TICKS              120u
#define FRAME_CHARS             3u
#define LETTERS                 26u
#define STRAY_ELEMENTS          3u

static uint32_t g_checks   = 0u;
static uint32_t g_failures = 0u;

static void check(bool b_ok, char const * p_what, int32_t line);
static bool near(int32_t value, int32_t expected, int32_t tolerance);
static uint32_t build_frame(uint8_t data_symbol, uint32_t * p_widths);
static void test_bitpack(void);
static void test_fix_math(void);
static void test_code39(void);
static uint32_t run_hump(hump_t * p_hump, int32_t peak_mdeg,
                         uint32_t ramp_steps, uint32_t hold_steps,
                         uint32_t bad_step, uint32_t * p_height);
static void test_hump(void);
static uint32_t run_lift(lift_t * p_lift, int32_t tilt_mdeg, uint32_t count,
                         bool b_valid);
static void test_lift(void);
static void test_obs_profile(void);
static void test_obs_beam(void);
static void test_tlm_codec(void);
static void test_nav_map(void);
static void test_pid(void);

#define CHECK(COND)             check((COND), #COND, __LINE__)


/*!
 * @brief Run every test and report.
 *
 * @return 0 if every check passed, 1 otherwise.
 */
int
main (void)
{
    test_bitpack();
    test_fix_math();
    test_code39();
    test_hump();
    test_lift();
    test_obs_profile();
    test_obs_beam();
    test_tlm_codec();
    test_nav_map();
    test_pid();

    printf("%lu checks, %lu failures\n", (unsigned long) g_checks,
           (unsigned long) g_failures);

    return ((0u == g_failures) ? 0 : 1);
}


/*!
 * @brief Record one check.
 *
 * @param[in] b_ok    Outcome.
 * @param[in] p_what  Expression text.
 * @param[in] line    Source line.
 */
static void
check (bool b_ok, char const * p_what, int32_t line)
{
    g_checks++;

    if (!b_ok)
    {
        g_failures++;
        printf("FAIL line %ld: %s\n", (long) line, p_what);
    }
}


/*!
 * @brief Absolute-tolerance comparison.
 *
 * @param[in] value      Measured value.
 * @param[in] expected   Expected value.
 * @param[in] tolerance  Allowed difference.
 *
 * @return true if within tolerance.
 */
static bool
near (int32_t value, int32_t expected, int32_t tolerance)
{
    return (fix_math_abs(value - expected) <= tolerance);
}


/*!
 * @brief Element widths of "*X*" for a data symbol, with small jitter.
 *
 * @param[in]  data_symbol  ASCII letter.
 * @param[out] p_widths     29 widths.
 *
 * @return Number of elements written.
 */
static uint32_t
build_frame (uint8_t data_symbol, uint32_t * p_widths)
{
    uint8_t const symbols[FRAME_CHARS] = { '*', data_symbol, '*' };
    uint32_t      count = 0u;

    for (uint32_t chr = 0u; chr < FRAME_CHARS; chr++)
    {
        uint32_t pattern = 0u;

        for (uint32_t candidate = 0u; candidate < 512u; candidate++)
        {
            if (symbols[chr] == code39_symbol_for(candidate))
            {
                pattern = candidate;
            }
        }

        for (uint32_t idx = 0u; idx < CODE39_ELEMENTS_PER_CHAR; idx++)
        {
            bool const     b_wide = (0u != ((pattern >> (8u - idx)) & 1u));
            uint32_t const jitter = (count % 3u) * 3u;

            p_widths[count] = (b_wide ? WIDE_TICKS : NARROW_TICKS) + jitter;
            count++;
        }

        if ((chr + 1u) < FRAME_CHARS)
        {
            p_widths[count] = NARROW_TICKS;
            count++;
        }
    }

    return (count);
}


/*!
 * @brief bitpack: word fields, excess-K, saturation and bit streams.
 */
static void
test_bitpack (void)
{
    uint8_t   buf[8];
    bitpack_t pack;
    uint32_t  word = 0u;

    word = bitpack_word_set(word, 3u, 5u, 0x1Fu);
    CHECK(0xF8u == word);
    CHECK(0x1Fu == bitpack_word_get(word, 3u, 5u));
    word = bitpack_word_set(word, 4u, 2u, 0u);
    CHECK(0x1Fu != bitpack_word_get(word, 3u, 5u));
    CHECK(1024u == bitpack_from_signed(0, 11u));
    CHECK(0u == bitpack_from_signed(-5000, 11u));
    CHECK(2047u == bitpack_from_signed(5000, 11u));
    CHECK(-300 == bitpack_to_signed(bitpack_from_signed(-300, 11u), 11u));
    CHECK(255u == bitpack_saturate(1000u, 8u));
    CHECK(0xFFFFFFFFu == bitpack_word_get(0xFFFFFFFFu, 0u, 32u));

    bitpack_init(&pack, buf, sizeof(buf));
    bitpack_put(&pack, 0x5u, 3u);
    bitpack_put(&pack, 0x1ABu, 9u);
    bitpack_put(&pack, 0xDEADBEEFu, 32u);
    CHECK(6u == bitpack_bytes_used(&pack));
    bitpack_put(&pack, 0xFFFFu, 32u);
    CHECK(bitpack_is_overflow(&pack));

    bitpack_init(&pack, buf, sizeof(buf));
    bitpack_put(&pack, 0x5u, 3u);
    bitpack_put(&pack, 0x1ABu, 9u);
    bitpack_put(&pack, 0xDEADBEEFu, 32u);
    pack.bit_pos = 0u;
    CHECK(0x5u == bitpack_get(&pack, 3u));
    CHECK(0x1ABu == bitpack_get(&pack, 9u));
    CHECK(0xDEADBEEFu == bitpack_get(&pack, 32u));
}


/*!
 * @brief fix_math: square root and trigonometry accuracy.
 */
static void
test_fix_math (void)
{
    CHECK(0u == fix_math_isqrt(0u));
    CHECK(1000u == fix_math_isqrt(1000000u));
    CHECK(65535u == fix_math_isqrt(0xFFFFFFFFu));
    CHECK(near(fix_math_sin_q15(30000), 16384, 40));
    CHECK(near(fix_math_sin_q15(90000), 32768, 1));
    CHECK(near(fix_math_sin_q15(-90000), -32768, 1));
    CHECK(near(fix_math_sin_q15(210000), -16384, 40));
    CHECK(near(fix_math_cos_q15(60000), 16384, 40));
    CHECK(near(fix_math_cos_q15(180000), -32768, 1));
    CHECK(near(fix_math_atan2_mdeg(1000, 1000), 45000, 300));
    CHECK(near(fix_math_atan2_mdeg(1000, -1000), 135000, 300));
    CHECK(near(fix_math_atan2_mdeg(-1000, -1000), -135000, 300));
    CHECK(near(fix_math_atan2_mdeg(500, 866), 30000, 300));
    CHECK(near(fix_math_atan2_mdeg(0, -1000), 180000, 300));
    CHECK(near(fix_math_asin_mdeg(16384), 30000, 300));
    CHECK(near(fix_math_asin_mdeg(-32768), -90000, 300));
    CHECK(-170000 == fix_math_wrap_mdeg(190000));
    CHECK(near(fix_math_abs(INT32_MIN), INT32_MAX, 0));
}


/*!
 * @brief code39: every letter, both directions, and rejections.
 */
static void
test_code39 (void)
{
    uint32_t widths[CODE39_FRAME_ELEMENTS];
    uint32_t reversed[CODE39_FRAME_ELEMENTS];
    uint8_t  symbol     = 0u;
    bool     b_reversed = false;
    uint32_t passed     = 0u;

    for (uint32_t letter = 0u; letter < LETTERS; letter++)
    {
        uint32_t const count = build_frame((uint8_t) ('A' + letter), widths);

        for (uint32_t idx = 0u; idx < count; idx++)
        {
            reversed[idx] = widths[(count - 1u) - idx];
        }

        if ((CODE39_OK == code39_decode(widths, count, &symbol,
                                        &b_reversed)) &&
            (('A' + letter) == symbol) && (!b_reversed) &&
            (CODE39_OK == code39_decode(reversed, count, &symbol,
                                        &b_reversed)) &&
            (('A' + letter) == symbol) && b_reversed)
        {
            passed++;
        }
    }

    CHECK(LETTERS == passed);
    CHECK(CODE39_BAD_COUNT == code39_decode(widths, 28u, &symbol,
                                            &b_reversed));

    (void) build_frame('B', widths);
    widths[3] = NARROW_TICKS + 30u;    // A narrow element nearly wide.
    widths[1] = WIDE_TICKS - 50u;      // And a wide one nearly narrow.
    CHECK(CODE39_OK != code39_decode(widths, CODE39_FRAME_ELEMENTS, &symbol,
                                     &b_reversed));

    (void) build_frame('1', widths);
    CHECK(CODE39_BAD_FRAME == code39_decode(widths, CODE39_FRAME_ELEMENTS,
                                            &symbol, &b_reversed));

    // Stray edges around the frame (paper edges): every letter, both ways.
    passed = 0u;

    for (uint32_t letter = 0u; letter < LETTERS; letter++)
    {
        uint32_t padded[CODE39_FRAME_ELEMENTS + STRAY_ELEMENTS];

        (void) build_frame((uint8_t) ('A' + letter), widths);
        padded[0] = WIDE_TICKS * 5u;
        padded[CODE39_FRAME_ELEMENTS + 1u] = WIDE_TICKS * 4u;
        padded[CODE39_FRAME_ELEMENTS + 2u] = NARROW_TICKS * 7u;

        for (uint32_t idx = 0u; idx < CODE39_FRAME_ELEMENTS; idx++)
        {
            padded[idx + 1u] = widths[idx];
        }

        if ((CODE39_OK == code39_decode(padded,
                                        CODE39_FRAME_ELEMENTS + STRAY_ELEMENTS,
                                        &symbol, &b_reversed)) &&
            (('A' + letter) == symbol) && (!b_reversed))
        {
            passed++;
        }

        for (uint32_t idx = 0u; idx < CODE39_FRAME_ELEMENTS; idx++)
        {
            padded[idx + 1u] = widths[(CODE39_FRAME_ELEMENTS - 1u) - idx];
        }

        if ((CODE39_OK == code39_decode(padded,
                                        CODE39_FRAME_ELEMENTS + STRAY_ELEMENTS,
                                        &symbol, &b_reversed)) &&
            (('A' + letter) == symbol) && b_reversed)
        {
            passed++;
        }
    }

    CHECK((2u * LETTERS) == passed);
}


/*!
 * @brief Feed one tilt profile (20 ms samples) to a fresh hump detector.
 *
 * @param[out] p_hump      Detector, left as the profile ends.
 * @param[in]  peak_mdeg   Peak tilt of a triangular hump (0: none).
 * @param[in]  ramp_steps  Samples up to the peak (and as many down).
 * @param[in]  hold_steps  Samples at the peak.
 * @param[in]  bad_step    Sample marked invalid (0: none).
 * @param[out] p_height    Height of the last completed hump.
 *
 * @return Number of humps reported.
 */
static uint32_t
run_hump (hump_t * p_hump, int32_t peak_mdeg, uint32_t ramp_steps,
          uint32_t hold_steps, uint32_t bad_step, uint32_t * p_height)
{
    uint32_t const start   = 10u;
    uint32_t const top     = start + ramp_steps;
    uint32_t const down    = top + hold_steps;
    uint32_t const end     = down + ramp_steps;
    uint32_t       reports = 0u;

    hump_init(p_hump, HUMP_ENTER_MDEG, HUMP_EXIT_MDEG, HUMP_HOLD_MS, 100u);

    for (uint32_t step = 0u; step < (end + 20u); step++)
    {
        int32_t tilt = 0;

        if ((step >= start) && (step < top))
        {
            // Casts: step counts are small, so the conversions are exact.
            tilt = (peak_mdeg * (int32_t) (step - start + 1u)) /
                   (int32_t) ramp_steps;
        }
        else if ((step >= top) && (step < down))
        {
            tilt = peak_mdeg;
        }
        else if ((step >= down) && (step < end))
        {
            // Casts: step counts are small, so the conversions are exact.
            tilt = (peak_mdeg * (int32_t) (end - step - 1u)) /
                   (int32_t) ramp_steps;
        }
        else
        {
            tilt = 0;
        }

        if (hump_update(p_hump, tilt, (0u == bad_step) || (step != bad_step),
                        20u, p_height))
        {
            reports++;
        }
    }

    return (reports);
}


/*!
 * @brief hump: one-wheel humps from the combined tilt, height = lever x
 *        sin(peak), never negative.
 */
static void
test_hump (void)
{
    hump_t   hump;
    uint32_t height = 0u;
    bool     b_seen = false;

    // A 12-degree lean over 0.4 s up and 0.4 s down: 100 x sin(12) = 20 mm.
    CHECK(1u == run_hump(&hump, 12000, 20u, 0u, 0u, &height));
    CHECK(near((int32_t) height, 20, 1));
    CHECK(near((int32_t) hump_max_mm(&hump), 20, 1));
    CHECK(1u == hump_count(&hump));
    CHECK(HUMP_STATE_LEVEL == hump_state(&hump));

    // A flat top (1 s at the peak) is still one hump of the same height.
    CHECK(1u == run_hump(&hump, 12000, 20u, 50u, 0u, &height));
    CHECK(near((int32_t) height, 20, 1));

    // A negative input (the other wheel, or a flipped axis) is the same.
    CHECK(1u == run_hump(&hump, -12000, 20u, 0u, 0u, &height));
    CHECK(near((int32_t) height, 20, 1));

    // Below the 3-degree threshold, or too brief: not a hump.
    CHECK(0u == run_hump(&hump, 2500, 20u, 0u, 0u, &height));
    CHECK(0u == run_hump(&hump, 8000, 1u, 2u, 0u, &height));
    CHECK(0u == hump_max_mm(&hump));

    // An invalid sample in the middle abandons the hump.
    CHECK(0u == run_hump(&hump, 12000, 20u, 0u, 30u, &height));
    CHECK(0u == hump_max_mm(&hump));

    // Tilted for 6 s (stuck or carried; the limit is 50 hold times): not a
    // hump.
    CHECK(0u == run_hump(&hump, 12000, 5u, 300u, 0u, &height));

    // CLIMBING while the lean grows, DESCENDING while it shrinks.
    hump_init(&hump, HUMP_ENTER_MDEG, HUMP_EXIT_MDEG, HUMP_HOLD_MS, 100u);

    for (int32_t tilt = 0; tilt <= 12000; tilt += 600)
    {
        (void) hump_update(&hump, tilt, true, 20u, &height);
    }

    CHECK(HUMP_STATE_CLIMBING == hump_state(&hump));

    for (int32_t tilt = 12000; tilt >= 6000; tilt -= 600)
    {
        (void) hump_update(&hump, tilt, true, 20u, &height);
    }

    CHECK(HUMP_STATE_DESCENDING == hump_state(&hump));

    for (uint32_t step = 0u; step < 10u; step++)
    {
        b_seen = hump_update(&hump, 0, true, 20u, &height) || b_seen;
    }

    CHECK(b_seen);
    CHECK(near((int32_t) height, 20, 1));
}


/*!
 * @brief Feed constant samples (20 ms apart) to a lift detector.
 *
 * @param[in,out] p_lift     Detector, left as the samples end.
 * @param[in]     tilt_mdeg  Lean of every sample.
 * @param[in]     count      Number of samples.
 * @param[in]     b_valid    Whether the samples are valid.
 *
 * @return Number of lifts reported.
 */
static uint32_t
run_lift (lift_t * p_lift, int32_t tilt_mdeg, uint32_t count, bool b_valid)
{
    uint32_t lifts = 0u;

    for (uint32_t i = 0u; i < count; i++)
    {
        if (lift_update(p_lift, tilt_mdeg, b_valid, 20u))
        {
            lifts++;
        }
    }

    return (lifts);
}


/*!
 * @brief lift: a held lean beyond the threshold is a lift; the floor, the
 *        hump and the lean read in turns are not.
 */
static void
test_lift (void)
{
    lift_t lift;

    // Level driving, the course hump (7.7 degrees) for a long time and the
    // lean read during turns and braking (17 degrees) are not lifts.
    lift_init(&lift, LIFT_TILT_MDEG, LIFT_HOLD_MS);
    CHECK(0u == run_lift(&lift, 0, 100u, true));
    CHECK(0u == run_lift(&lift, 7700, 1000u, true));
    CHECK(0u == run_lift(&lift, 17000, 1000u, true));
    CHECK(!lift_is_lifted(&lift));

    // A lean beyond the threshold is a lift once it has lasted the hold,
    // and is reported once however long it lasts.
    CHECK(0u == run_lift(&lift, 30000, 14u, true));
    CHECK(1u == run_lift(&lift, 30000, 1u, true));
    CHECK(lift_is_lifted(&lift));
    CHECK(0u == run_lift(&lift, 30000, 500u, true));

    // Level again re-arms it; the opposite lean counts the same.
    CHECK(0u == run_lift(&lift, 0, 10u, true));
    CHECK(!lift_is_lifted(&lift));
    CHECK(1u == run_lift(&lift, -30000, 20u, true));

    // A lean shorter than the hold (one jolt) is not a lift.
    lift_init(&lift, LIFT_TILT_MDEG, LIFT_HOLD_MS);
    CHECK(0u == run_lift(&lift, 40000, 3u, true));
    CHECK(0u == run_lift(&lift, 0, 10u, true));

    // Invalid samples never report and restart the hold time.
    CHECK(0u == run_lift(&lift, 30000, 100u, false));
    CHECK(0u == run_lift(&lift, 30000, 14u, true));
    CHECK(0u == run_lift(&lift, 30000, 1u, false));
    CHECK(0u == run_lift(&lift, 30000, 14u, true));
    CHECK(1u == run_lift(&lift, 30000, 1u, true));

    // Steady following: the course hump is still not a lift, but a lean
    // too big for any hump (20 degrees) is, after the same hold.
    lift_init(&lift, LIFT_TILT_MDEG, LIFT_HOLD_MS);
    lift_set_tilt(&lift, LIFT_TILT_STEADY_MDEG);
    CHECK(0u == run_lift(&lift, 7700, 1000u, true));
    CHECK(0u == run_lift(&lift, 0, 10u, true));
    CHECK(0u == run_lift(&lift, 20000, 14u, true));
    CHECK(1u == run_lift(&lift, 20000, 1u, true));

    // Back to the normal threshold (a turn): 20 degrees is not a lift.
    lift_init(&lift, LIFT_TILT_MDEG, LIFT_HOLD_MS);
    lift_set_tilt(&lift, LIFT_TILT_MDEG);
    CHECK(0u == run_lift(&lift, 20000, 1000u, true));
}


/*!
 * @brief obs_profile: plans for clear, one-side and blocked scenes.
 */
static void
test_obs_profile (void)
{
    obs_profile_point_t   points[OBS_PROFILE_MAX_POINTS];
    obs_profile_t profile;
    obs_profile_t decoded;
    uint8_t       msg[OBS_PROFILE_MSG_BYTES];
    uint32_t      count = 0u;

    // Scene 1: nothing within range.
    for (uint32_t angle = 30u; angle <= 150u; angle += 30u)
    {
        points[count].angle_deg   = angle;
        points[count].distance_mm = 1500u;
        points[count].b_echo      = true;
        count++;
    }

    obs_profile_compute(points, count, &profile);
    CHECK(OBS_PLAN_CONTINUE == profile.plan);

    // Scene 2: a 100 mm box 250 mm ahead, open on both sides.
    count = 0u;

    for (uint32_t angle = 30u; angle <= 150u; angle += 5u)
    {
        points[count].angle_deg   = angle;
        points[count].distance_mm = ((angle >= 80u) && (angle <= 100u)) ?
                                    250u : 1500u;
        points[count].b_echo      = ((angle >= 80u) && (angle <= 100u));
        count++;
    }

    // Ideal pencil-beam samples: no beam correction for these scenes
    // (test_obs_beam covers it).  Cast: enumeration value, exact.
    CHECK(tune_set((uint32_t) TUNE_OBS_BEAM_DEG, 0));
    obs_profile_compute(points, count, &profile);
    CHECK((OBS_PLAN_PASS_LEFT == profile.plan) ||
          (OBS_PLAN_PASS_RIGHT == profile.plan));
    CHECK(near((int32_t) profile.width_mm, 87, 30));

    obs_profile_pack(&profile, msg);
    obs_profile_unpack(msg, &decoded);
    CHECK(decoded.plan == profile.plan);
    CHECK((profile.distance_mm / 10u) == (decoded.distance_mm / 10u));

    // Scene 3: the same box with a wall close on the left: pass right.
    for (uint32_t idx = 0u; idx < count; idx++)
    {
        if (points[idx].angle_deg >= 110u)
        {
            points[idx].distance_mm = 260u;
            points[idx].b_echo      = true;
        }
    }

    obs_profile_compute(points, count, &profile);
    CHECK(OBS_PLAN_PASS_RIGHT == profile.plan);

    // Scene 4: walls on both sides: reverse and retry.
    for (uint32_t idx = 0u; idx < count; idx++)
    {
        if (points[idx].angle_deg <= 70u)
        {
            points[idx].distance_mm = 260u;
            points[idx].b_echo      = true;
        }
    }

    obs_profile_compute(points, count, &profile);
    CHECK(OBS_PLAN_REVERSE == profile.plan);

    // No usable samples at all: stop.
    obs_profile_compute(points, 0u, &profile);
    CHECK(OBS_PLAN_STOP == profile.plan);
}


/*!
 * @brief obs_profile: the sonar beam correction of the obstacle width.
 *
 * @par
 * A real sonar beam (about 15 degrees each side) still hears a 100 mm box
 * 250 mm ahead out to 65 and 115 degrees.  Uncorrected that reads about
 * 210 mm; the default correction brings it near the true 100 mm.
 */
static void
test_obs_beam (void)
{
    obs_profile_point_t spread[OBS_PROFILE_MAX_POINTS];
    obs_profile_t       profile;
    uint32_t            used = 0u;

    for (uint32_t angle = 30u; angle <= 150u; angle += 5u)
    {
        bool const b_hit  = (angle >= 65u) && (angle <= 115u);
        bool const b_face = (angle >= 80u) && (angle <= 100u);
        // Cast: angle is at most 150, so the conversion is exact.
        int32_t const sine = fix_math_sin_q15((int32_t) angle * 1000);

        // A flat face 250 mm ahead reads 250 / sin(angle); the cone hears
        // the corners (255 mm) out to 65 and 115 degrees.  Cast: 250..254.
        spread[used].angle_deg   = angle;
        spread[used].distance_mm = b_face ?
                                   (uint32_t) ((250 * FIX_MATH_Q15_ONE) /
                                               sine) :
                                   (b_hit ? 255u : 1500u);
        spread[used].b_echo      = b_hit;
        used++;
    }

    // Casts: enumeration value and small constants, all exact.
    CHECK(tune_set((uint32_t) TUNE_OBS_BEAM_DEG, 0));
    // From the angle alone it read about 215 mm (capped at
    // OBSTACLE_MAX_WIDTH_MM).  The range limit (2026-11-24) puts each edge
    // at the corner's range, so it is near the true 100 mm even uncorrected.
    obs_profile_compute(spread, used, &profile);
    CHECK(near((int32_t) profile.width_mm, 100, 15));  // Cast: < 2^16.

    // Casts: enumeration value and small constants, all exact.
    CHECK(tune_set((uint32_t) TUNE_OBS_BEAM_DEG,
                   (int32_t) OBS_BEAM_HALF_DEG));
    obs_profile_compute(spread, used, &profile);
    CHECK(near((int32_t) profile.width_mm, 100, 40));  // Cast: < 2^16.

    // Something 90 mm behind the box, beside it (60 and 120 degrees), is
    // not part of the obstacle (OBSTACLE_DEPTH_MM): the width is unchanged.
    for (uint32_t idx = 0u; idx < used; idx++)
    {
        if ((60u == spread[idx].angle_deg) ||
            (120u == spread[idx].angle_deg))
        {
            spread[idx].b_echo      = true;
            spread[idx].distance_mm = 340u;
        }
    }

    obs_profile_compute(spread, used, &profile);
    CHECK(near((int32_t) profile.width_mm, 100, 40));  // Cast: < 2^16.

    // A 150 mm box only 100 mm ahead (the closer stop, 2026-11-24): its
    // corners are 125 mm away at 53 and 127 degrees, and the 15-degree cone
    // still hears them out to 40 and 140 degrees.  The face in between
    // reads 100 / sin(angle).
    used = 0u;

    for (uint32_t angle = 20u; angle <= 160u; angle += 5u)
    {
        bool const b_hit  = (angle >= 40u) && (angle <= 140u);
        bool const b_face = (angle >= 55u) && (angle <= 125u);
        // Cast: angle is at most 160, so the conversion is exact.
        int32_t const sine = fix_math_sin_q15((int32_t) angle * 1000);

        spread[used].angle_deg   = angle;
        // Cast: 100 / sin is 100..122 on the face, so it is exact.
        spread[used].distance_mm = b_face ?
                                   (uint32_t) ((100 * FIX_MATH_Q15_ONE) /
                                               sine) :
                                   (b_hit ? 125u : 1500u);
        spread[used].b_echo      = b_hit;
        used++;
    }

    obs_profile_compute(spread, used, &profile);
    CHECK(near((int32_t) profile.width_mm, 150, 20));  // Cast: < 2^16.
    CHECK(near((int32_t) profile.distance_mm, 100, 5));  // Cast: < 2^16.
}


/*!
 * @brief tlm_codec: frame sizes and command validation.
 */
static void
test_tlm_codec (void)
{
    uint32_t      words[SYS_STATUS_WORD_COUNT];
    uint8_t       frame[TLM_CODEC_MAX_FRAME];
    uint32_t      word = 0u;
    tlm_codec_summary_t summary = { 17u, 2u, 5u, 1u, 1u, 1234u, 95u, 0u };
    uint8_t const start[]   = { 0x18u, 0x00u };
    uint8_t const scan[]    = { 0x18u, 0x06u };
    uint8_t const map_set[] = { 0x18u, 0x34u, 0x22u };
    uint8_t const bad_ver[] = { 0x28u, 0x00u };
    uint8_t const bad_map[] = { 0x18u, 0x04u, 0x1Fu };
    uint8_t const tune_edge[] = { 0x18u, 0x75u, 0xF1u, 0x13u };
    uint8_t const tune_dir[]  = { 0x18u, 0x65u, 0xFFu, 0xFFu };

    for (uint32_t idx = 0u; idx < (uint32_t) SYS_STATUS_WORD_COUNT; idx++)
    {
        words[idx] = 0xFFFFFFFFu;
    }

    CHECK(288u == (16u + sys_status_total_bits()));
    CHECK(36u == tlm_codec_telemetry(7u, words, frame, sizeof(frame)));
    CHECK(0x10u == frame[0]);
    CHECK(7u == frame[1]);
    CHECK(6u == tlm_codec_heartbeat(1u, 60u, 3u, 0u, frame, sizeof(frame)));
    CHECK(6u == tlm_codec_event(2u, 0x12345678u, frame, sizeof(frame)));
    CHECK(11u == tlm_codec_summary(&summary, frame, sizeof(frame)));
    CHECK(2u == tlm_codec_ack(4u, true, frame, sizeof(frame)));
    CHECK(0u == tlm_codec_telemetry(0u, words, frame, 10u));

    CHECK(TLM_CMD_START == tlm_codec_parse_command(start, 2u, &word));
    CHECK(TLM_CMD_SCAN == tlm_codec_parse_command(scan, 2u, &word));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(scan, 1u, &word));
    CHECK(TLM_CMD_MAP_SET == tlm_codec_parse_command(map_set, 3u, &word));
    CHECK(2u == bitpack_word_get(word, TLM_CMD_LETTER_SHIFT,
                                 TLM_CMD_LETTER_WIDTH));
    CHECK(1u == bitpack_word_get(word, TLM_CMD_ACTION_SHIFT,
                                 TLM_CMD_ACTION_WIDTH));
    CHECK(3u == bitpack_word_get(word, TLM_CMD_VERSION_SHIFT,
                                 TLM_CMD_VERSION_WIDTH));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(bad_ver, 2u, &word));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(bad_map, 3u, &word));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(start, 1u, &word));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(map_set, 2u, &word));

    // TUNE: setting 7 (encoder edge) = 5105, and setting 6 (steering) = -1.
    CHECK(TLM_CMD_TUNE == tlm_codec_parse_command(tune_edge, 4u, &word));
    CHECK(7u == bitpack_word_get(word, TLM_CMD_PARAM_SHIFT,
                                 TLM_CMD_PARAM_WIDTH));
    CHECK(5105 == bitpack_to_signed(bitpack_word_get(word,
                                        TLM_CMD_VALUE_SHIFT,
                                        TLM_CMD_VALUE_WIDTH),
                                    TLM_CMD_VALUE_WIDTH));
    CHECK(TLM_CMD_TUNE == tlm_codec_parse_command(tune_dir, 4u, &word));
    CHECK(-1 == bitpack_to_signed(bitpack_word_get(word,
                                      TLM_CMD_VALUE_SHIFT,
                                      TLM_CMD_VALUE_WIDTH),
                                  TLM_CMD_VALUE_WIDTH));
    CHECK(TLM_CMD_INVALID == tlm_codec_parse_command(tune_edge, 3u, &word));

    // Settings accept only in-range values.
    CHECK(tune_set(TUNE_MOTOR_MAX, 350));
    CHECK(350 == tune_get(TUNE_MOTOR_MAX));
    CHECK(!tune_set(TUNE_MOTOR_MAX, 100));
    CHECK(350 == tune_get(TUNE_MOTOR_MAX));
    CHECK(!tune_set(TUNE_STEER_DIR, 0));
    CHECK(tune_set(TUNE_STEER_DIR, -1));
    CHECK(!tune_set(TUNE_COUNT, 1));
}


/*!
 * @brief nav_map: default actions and updates.
 */
static void
test_nav_map (void)
{
    nav_map_t map;

    // Nothing is hard-coded: every letter goes straight until mapped.
    nav_map_init(&map);
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 0u));
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 1u));
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 3u));
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 25u));
    CHECK(0u == nav_map_version(&map));

    // A map of four letters, one version, as sent over MQTT.
    CHECK(nav_map_set(&map, 6u, NAV_ACTION_STRAIGHT, 1u));     // G.
    CHECK(nav_map_set(&map, 10u, NAV_ACTION_UTURN, 1u));       // K.
    CHECK(nav_map_set(&map, 16u, NAV_ACTION_LEFT, 1u));        // Q.
    CHECK(nav_map_set(&map, 25u, NAV_ACTION_RIGHT, 1u));       // Z.
    CHECK(NAV_ACTION_UTURN == nav_map_get(&map, 10u));
    CHECK(NAV_ACTION_LEFT == nav_map_get(&map, 16u));
    CHECK(NAV_ACTION_RIGHT == nav_map_get(&map, 25u));
    CHECK(1u == nav_map_version(&map));

    // A new version replaces the whole map: Q and Z no longer turn.
    CHECK(nav_map_set(&map, 0u, NAV_ACTION_LEFT, 2u));         // A.
    CHECK(NAV_ACTION_LEFT == nav_map_get(&map, 0u));
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 16u));
    CHECK(NAV_ACTION_STRAIGHT == nav_map_get(&map, 25u));
    CHECK(2u == nav_map_version(&map));

    // An invalid letter changes nothing, not even the version.
    CHECK(!nav_map_set(&map, 26u, NAV_ACTION_LEFT, 8u));
    CHECK(2u == nav_map_version(&map));
    CHECK(NAV_ACTION_LEFT == nav_map_get(&map, 0u));
}


/*!
 * @brief pid_ctrl: sign, clamping and anti-windup.
 */
static void
test_pid (void)
{
    pid_ctrl_t pid;
    int32_t    output = 0;

    pid_ctrl_init(&pid, 1000, 500, 0, 300);
    CHECK(pid_ctrl_update(&pid, 100, 0) > 0);
    CHECK(pid_ctrl_update(&pid, 0, 100) < 0);

    for (uint32_t step = 0u; step < 100u; step++)
    {
        output = pid_ctrl_update(&pid, 1000, 0);
    }

    CHECK(300 == output);
    CHECK(pid.integral <= pid.integral_limit);

    // After saturation, a reversed error must pull the output back quickly.
    output = pid_ctrl_update(&pid, 0, 50);
    CHECK(output < 300);
}

/*** end of file ***/

