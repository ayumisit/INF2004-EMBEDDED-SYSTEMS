/** @file imu.c
 *
 * @brief IMU Task: tilt, heading, motion class, humps and impacts.
 *
 * @par
 * The LSM303DLHC has no gyroscope, so pitch comes from the direction of
 * gravity (asin of the forward acceleration over the total), smoothed with
 * a first-order low-pass filter, and turn rate comes from the wheel speed
 * difference (FR15, assumption A6).  Roll comes the same way from the side
 * axis; the tilt (pitch and roll combined, never negative) drives the hump
 * detector, so a hump under either wheel is seen.  Heading is the level
 * magnetometer
 * angle; it is not tilt-compensated and the motors disturb it, which is why
 * it is reported but not used for control.
 *
 * @par
 * Lift detection (Week 12 requirement): the same tilt feeds the lift
 * detector.  When the robot is picked up and tipped, ROBOT_EVT_LIFTED
 * tells Vehicle Control to stop the motors.
 *
 * @par
 * CALIBRATING averages IMU_CALIB_SAMPLES readings while the robot is still
 * to remove the mounting pitch and record 1 g.  Three consecutive I2C
 * failures enter SENSOR FAULT: the data is marked invalid, ROBOT_FAULT_IMU
 * is raised and the sensor is re-initialised once a second.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "robot_ipc.h"
#include "period_tick.h"
#include "sys_status.h"
#include "bitpack.h"
#include "fix_math.h"
#include "telemetry.h"
#include "buzzer.h"
#include "encoder.h"
#include "motion.h"
#include "vehicle.h"
#include "lsm303_hw.h"
#include "hump.h"
#include "lift.h"
#include "imu.h"

#define MS_PER_PERIOD           (PERIOD_BASE_MS * PERIOD_IMU_TICKS)
#define PERIODS_PER_SECOND      (1000u / MS_PER_PERIOD)
#define MAG_EVERY_PERIODS       3u
#define FILTER_DIVISOR          4
#define ONE_G_MG                1000
#define MDEG_PER_HALF_DEG       500
#define MDEG_PER_DEG            1000
#define MDEG_PER_CDEG           10
#define MDEG_PER_RAD            57296
#define IMPACT_UNIT_MG          16
#define MAX_COUNT_4BIT          15u
#define STILL_MM_S              20
#define IMPACT_DATA_WIDTH       8u
#define FULL_TURN_DEG           360

/*!
 * @brief IMU Task working state.
 */
typedef struct
{
    bool        b_sensor_ok;
    uint32_t    errors;
    uint32_t    retry_countdown;
    uint32_t    frozen_count;
    uint32_t    follow_ms;
    int32_t     prev_accel[LSM303_HW_AXES];
    uint32_t    calib_count;
    int32_t     calib_pitch_sum;
    int32_t     calib_roll_sum;
    int32_t     calib_mag_sum;
    uint32_t    relevel_count;      // Samples in this READY block.
    int32_t     relevel_pitch_sum;
    int32_t     relevel_roll_sum;
    int32_t     relevel_mag_sum;
    int32_t     relevel_low_mdeg;   // Lowest raw pitch in the block.
    int32_t     relevel_high_mdeg;  // Highest raw pitch in the block.
    int32_t     pitch_bias_mdeg;
    int32_t     roll_bias_mdeg;
    int32_t     one_g_mg;
    int32_t     pitch_mdeg;
    int32_t     roll_mdeg;
    int32_t     tilt_mdeg;
    int32_t     heading_mdeg;
    int32_t     prev_speed_mm_s;
    uint32_t    mag_countdown;
    uint32_t    cooldown_ms;
    uint32_t    impacts;
    uint32_t    last_hump_mm;
    imu_class_t motion_class;
    hump_t      hump;
    lift_t      lift;
} imu_ctx_t;

static imu_ctx_t g_ctx;

// Read by the USB monitor.  Each has one writer (the IMU task) and is one
// aligned word, so plain loads and stores are atomic.
static uint32_t volatile g_lift_count   = 0u;
static uint64_t  g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void    imu_task(INT start_code, void * p_ext_info);
static bool    sample(int32_t * p_accel);
static int32_t raw_angle(int32_t const * p_accel, uint32_t axis,
                         int32_t magnitude);
static int32_t combine_tilt(int32_t pitch_mdeg, int32_t roll_mdeg);
static void    check_impact(int32_t magnitude);
static void    read_heading(void);
static void    report_init(bool b_ok);
static void    classify(encoder_state_t const * p_enc);
static void    track_hump(bool b_valid);
static void    track_lift(bool b_valid);
static bool    is_frozen(int32_t const * p_accel);
static void    relevel(int32_t pitch, int32_t roll, int32_t magnitude);
static bool    is_following_steadily(encoder_state_t const * p_enc);
static void    publish(bool b_valid);


/*!
 * @brief Start the IMU Task (the sensor is initialised by the task).
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
imu_start (void)
{
    int32_t result = 0;
    int32_t h_task = 0;

    g_ctx.b_sensor_ok     = false;
    g_ctx.retry_countdown = 0u;
    g_ctx.one_g_mg        = ONE_G_MG;
    g_ctx.motion_class    = IMU_CLASS_STATIONARY;
    hump_init(&g_ctx.hump, HUMP_ENTER_MDEG, HUMP_EXIT_MDEG, HUMP_HOLD_MS,
              HUMP_LEVER_MM);
    lift_init(&g_ctx.lift, LIFT_TILT_MDEG, LIFT_HOLD_MS);

    h_task = robot_ipc_start_task(imu_task, PRIO_IMU, g_stack,
                                  sizeof(g_stack));

    if (!period_tick_register(h_task, PERIOD_IMU_TICKS))
    {
        result = -1;
    }

    return (result);
}


/*!
 * @brief Number of lifts reported since start-up.
 *
 * @return Count, for the USB monitor.
 */
uint32_t
imu_lift_count (void)
{
    return (g_lift_count);
}


/*!
 * @brief IMU Task, released every 20 ms.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
imu_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    // Leave the chip alone while it starts up.  Seen 2026-11-17: after
    // three of four power-ons from the battery alone the accelerometer
    // gave the same reading for the whole session, and restarting it did
    // not help; it never happened with the board already powered over USB.
    robot_ipc_sleep_ms(IMU_START_DELAY_MS);

    for (;;)
    {
        encoder_state_t enc;
        bool            b_valid  = false;
        bool            b_steady = false;

        (void) period_tick_wait();  // Missed periods are counted centrally.

        if (!g_ctx.b_sensor_ok)
        {
            // SENSOR FAULT: re-initialise once a second.
            if (0u == g_ctx.retry_countdown)
            {
                g_ctx.b_sensor_ok     = lsm303_hw_init();
                g_ctx.errors          = 0u;
                g_ctx.calib_count     = 0u;
                g_ctx.calib_pitch_sum = 0;
                g_ctx.calib_roll_sum  = 0;
                g_ctx.calib_mag_sum   = 0;
                g_ctx.retry_countdown = PERIODS_PER_SECOND;
                report_init(g_ctx.b_sensor_ok);
            }

            g_ctx.retry_countdown--;
        }
        else
        {
            int32_t accel[LSM303_HW_AXES] = { 0, 0, 0 };

            b_valid = sample(accel);
        }

        encoder_get_state(&enc);
        classify(&enc);

        // Turning, braking and pulling away tilt the IMU by themselves
        // (11.5 and 17 degrees measured on 2026-11-04; 13.5 and 19.5 on
        // 2026-11-17), which read as false humps.  A hump is only on the
        // line, so samples count only in steady line following; all others
        // abandon any hump, like invalid data.  So does a lift, so being
        // picked up is never recorded as a hump.
        b_steady = is_following_steadily(&enc);

        // A lean no course hump can give, in steady following, is a lift.
        lift_set_tilt(&g_ctx.lift, b_steady ? LIFT_TILT_STEADY_MDEG :
                                              LIFT_TILT_MDEG);
        track_lift(b_valid);
        track_hump(b_valid && b_steady && (!lift_is_lifted(&g_ctx.lift)));
        publish(b_valid);
    }
}


/*!
 * @brief Read the sensor and update pitch, heading and impact detection.
 *
 * @param[out] p_accel  Acceleration in milli-g.
 *
 * @return true if this sample is valid (sensor read and calibrated).
 */
static bool
sample (int32_t * p_accel)
{
    bool       b_valid = false;
    bool const b_read  = lsm303_hw_read_accel(p_accel);

    if (b_read && (is_frozen(p_accel)))
    {
        // The chip answers but its readings never change: the
        // accelerometer is switched off.  Start it again.
        g_ctx.frozen_count    = 0u;
        g_ctx.b_sensor_ok     = false;
        g_ctx.retry_countdown = PERIODS_PER_SECOND;
        robot_ipc_raise_fault(ROBOT_FAULT_IMU);
    }
    else if (!b_read)
    {
        g_ctx.errors++;

        if (g_ctx.errors >= IMU_FAULT_ERRORS)
        {
            g_ctx.b_sensor_ok     = false;
            g_ctx.retry_countdown = PERIODS_PER_SECOND;
            robot_ipc_raise_fault(ROBOT_FAULT_IMU);
        }
    }
    else
    {
        // Each axis is within +/-2048 mg, so the squares sum below 2^24
        // and the conversions are exact.
        uint32_t const sum_sq    = (uint32_t) ((p_accel[0] * p_accel[0]) +
                                               (p_accel[1] * p_accel[1]) +
                                               (p_accel[2] * p_accel[2]));
        int32_t const  magnitude = (int32_t) fix_math_isqrt(sum_sq);  // <4096.
        int32_t const  pitch     = raw_angle(p_accel, IMU_FORWARD_AXIS,
                                             magnitude) * IMU_FORWARD_SIGN;
        int32_t const  roll      = raw_angle(p_accel, IMU_SIDE_AXIS,
                                             magnitude);

        g_ctx.errors = 0u;

        if (g_ctx.calib_count < IMU_CALIB_SAMPLES)
        {
            // CALIBRATING: the robot is held still in READY.
            g_ctx.calib_pitch_sum += pitch;
            g_ctx.calib_roll_sum  += roll;
            g_ctx.calib_mag_sum   += magnitude;
            g_ctx.calib_count++;
            g_ctx.pitch_bias_mdeg  = g_ctx.calib_pitch_sum /
                                     (int32_t) g_ctx.calib_count;  // <= 50.
            g_ctx.roll_bias_mdeg   = g_ctx.calib_roll_sum /
                                     (int32_t) g_ctx.calib_count;  // <= 50.
            g_ctx.one_g_mg         = g_ctx.calib_mag_sum /
                                     (int32_t) g_ctx.calib_count;  // <= 50.
            g_ctx.pitch_mdeg       = 0;
            g_ctx.roll_mdeg        = 0;
            g_ctx.tilt_mdeg        = 0;
        }
        else
        {
            g_ctx.pitch_mdeg += ((pitch - g_ctx.pitch_bias_mdeg) -
                                 g_ctx.pitch_mdeg) / FILTER_DIVISOR;
            g_ctx.roll_mdeg  += ((roll - g_ctx.roll_bias_mdeg) -
                                 g_ctx.roll_mdeg) / FILTER_DIVISOR;
            g_ctx.tilt_mdeg   = combine_tilt(g_ctx.pitch_mdeg,
                                             g_ctx.roll_mdeg);
            check_impact(magnitude);
            relevel(pitch, roll, magnitude);
            b_valid = true;
        }

        read_heading();
    }

    return (b_valid);
}


/*!
 * @brief Re-learn level while the robot waits in READY.
 *
 * @param[in] pitch      Raw pitch of this sample, millidegrees.
 * @param[in] roll       Raw roll of this sample, millidegrees.
 * @param[in] magnitude  Acceleration magnitude of this sample, mg.
 *
 * @par
 * 2026-11-24: level was only learnt at power-on.  Powered on in a hand,
 * the robot later read 16.5 degrees sitting flat on the track, which is
 * over LIFT_TILT_STEADY_MDEG, so every run stopped as "picked up".  Now
 * every block of IMU_CALIB_SAMPLES samples taken in READY whose raw pitch
 * stays within IMU_RELEVEL_SPAN_MDEG (held still, not in a hand) replaces
 * the level, so the robot is level wherever it sat still before Start.
 */
static void
relevel (int32_t pitch, int32_t roll, int32_t magnitude)
{
    uint32_t const state = bitpack_word_get(sys_status_read(
                                                SYS_STATUS_VEHICLE),
                                            VEHICLE_STATE_SHIFT,
                                            VEHICLE_STATE_WIDTH);
    // Cast: enumeration value, exact.
    bool const     b_ready = ((uint32_t) VEHICLE_READY == state);
    bool const     b_first = (0u == g_ctx.relevel_count);

    g_ctx.relevel_pitch_sum = b_first ? pitch :
                              (g_ctx.relevel_pitch_sum + pitch);
    g_ctx.relevel_roll_sum  = b_first ? roll :
                              (g_ctx.relevel_roll_sum + roll);
    g_ctx.relevel_mag_sum   = b_first ? magnitude :
                              (g_ctx.relevel_mag_sum + magnitude);
    g_ctx.relevel_low_mdeg  = (b_first || (pitch < g_ctx.relevel_low_mdeg)) ?
                              pitch : g_ctx.relevel_low_mdeg;
    g_ctx.relevel_high_mdeg = (b_first ||
                               (pitch > g_ctx.relevel_high_mdeg)) ?
                              pitch : g_ctx.relevel_high_mdeg;
    g_ctx.relevel_count     = b_ready ? (g_ctx.relevel_count + 1u) : 0u;

    if ((g_ctx.relevel_count >= IMU_CALIB_SAMPLES) &&
        ((g_ctx.relevel_high_mdeg - g_ctx.relevel_low_mdeg) <=
         IMU_RELEVEL_SPAN_MDEG))
    {
        // Casts: the count is IMU_CALIB_SAMPLES (50), exact.
        g_ctx.pitch_bias_mdeg = g_ctx.relevel_pitch_sum /
                                (int32_t) g_ctx.relevel_count;  // 50.
        g_ctx.roll_bias_mdeg  = g_ctx.relevel_roll_sum /
                                (int32_t) g_ctx.relevel_count;  // 50.
        g_ctx.one_g_mg        = g_ctx.relevel_mag_sum /
                                (int32_t) g_ctx.relevel_count;  // 50.
        g_ctx.pitch_mdeg      = 0;
        g_ctx.roll_mdeg       = 0;
        g_ctx.tilt_mdeg       = 0;
    }

    if (g_ctx.relevel_count >= IMU_CALIB_SAMPLES)
    {
        g_ctx.relevel_count = 0u;  // Start the next block.
    }
}


/*!
 * @brief Notice an accelerometer whose readings have stopped changing.
 *
 * @param[in] p_accel  Latest acceleration in milli-g.
 *
 * @return true once the same reading has repeated IMU_FROZEN_SAMPLES times.
 *
 * @par
 * A working accelerometer has a few milli-g of noise on every axis, even
 * at rest, so three identical axes for a whole second means the data is
 * not being updated.  Seen 2026-11-17 after a power-on: the tilt stayed at
 * exactly zero, so neither a hump nor a lift could be detected.
 */
static bool
is_frozen (int32_t const * p_accel)
{
    bool b_same = true;

    for (uint32_t axis = 0u; axis < LSM303_HW_AXES; axis++)
    {
        if (p_accel[axis] != g_ctx.prev_accel[axis])
        {
            b_same = false;
        }

        g_ctx.prev_accel[axis] = p_accel[axis];
    }

    g_ctx.frozen_count = b_same ? (g_ctx.frozen_count + 1u) : 0u;

    return (g_ctx.frozen_count >= IMU_FROZEN_SAMPLES);
}


/*!
 * @brief Tilt of one horizontal axis from the gravity direction.
 *
 * @param[in] p_accel    Acceleration in milli-g.
 * @param[in] axis       LSM303_HW_AXIS_* of the horizontal axis.
 * @param[in] magnitude  Total acceleration in milli-g.
 *
 * @return Angle in millidegrees, positive when that axis points up,
 *         mounting bias included.
 */
static int32_t
raw_angle (int32_t const * p_accel, uint32_t axis, int32_t magnitude)
{
    int32_t ratio = 0;

    if (magnitude > 0)
    {
        ratio = (p_accel[axis] * FIX_MATH_Q15_ONE) / magnitude;
    }

    return (fix_math_asin_mdeg(ratio));
}


/*!
 * @brief Total lean from level, in any direction.
 *
 * @param[in] pitch_mdeg  Nose-up angle, bias removed.
 * @param[in] roll_mdeg   Side angle, bias removed.
 *
 * @return sqrt(pitch^2 + roll^2) in millidegrees, never negative.  Within
 *         1% of the true angle between the two gravity vectors up to 20
 *         degrees, well beyond any course hump.
 *
 * @par
 * Both inputs are worked in 0.01-degree steps, so each square is below
 * 9000^2 and the sum stays far below 2^32.
 */
static int32_t
combine_tilt (int32_t pitch_mdeg, int32_t roll_mdeg)
{
    int32_t const pitch_cdeg = pitch_mdeg / MDEG_PER_CDEG;
    int32_t const roll_cdeg  = roll_mdeg / MDEG_PER_CDEG;

    // Each square is non-negative and below 2^27: exact conversions.
    uint32_t const sum_sq    = (uint32_t) (pitch_cdeg * pitch_cdeg) +
                               (uint32_t) (roll_cdeg * roll_cdeg);

    // The root is below 2^15, so the conversion is exact.
    return ((int32_t) fix_math_isqrt(sum_sq) * MDEG_PER_CDEG);
}


/*!
 * @brief Detect a sudden impact from the total acceleration.
 *
 * @param[in] magnitude  Total acceleration in milli-g.
 *
 * @par
 * IMPACT is raised when the total departs from 1 g by more than IMPACT_MG,
 * followed by a cooldown so one collision is reported once.
 */
static void
check_impact (int32_t magnitude)
{
    int32_t const deviation = fix_math_abs(magnitude - g_ctx.one_g_mg);

    if (g_ctx.cooldown_ms > 0u)
    {
        g_ctx.cooldown_ms = (g_ctx.cooldown_ms > MS_PER_PERIOD) ?
                            (g_ctx.cooldown_ms - MS_PER_PERIOD) : 0u;
    }
    else if (deviation > (int32_t) IMPACT_MG)  // 1500: exact conversion.
    {
        // deviation is positive here, so the conversion is exact.
        uint32_t const peak = (uint32_t) (deviation / IMPACT_UNIT_MG);

        g_ctx.cooldown_ms = IMPACT_COOLDOWN_MS;
        g_ctx.impacts     = (g_ctx.impacts < MAX_COUNT_4BIT) ?
                            (g_ctx.impacts + 1u) : MAX_COUNT_4BIT;
        robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_IMPACT);
        telemetry_post_event(TELEMETRY_EVT_IMPACT,
                             bitpack_saturate(peak, IMPACT_DATA_WIDTH));
    }
    else
    {
        // No impact.
    }
}


/*!
 * @brief Raise or clear ROBOT_FAULT_IMU after an initialisation attempt.
 *
 * @param[in] b_ok  true if the sensor answered and accepted its settings.
 */
static void
report_init (bool b_ok)
{
    if (b_ok)
    {
        robot_ipc_flag_clear(ROBOT_IPC_FLAG_FAULT, ROBOT_FAULT_IMU);
    }
    else
    {
        robot_ipc_raise_fault(ROBOT_FAULT_IMU);
    }
}


/*!
 * @brief Update the heading from the magnetometer every third sample.
 */
static void
read_heading (void)
{
    if (0u == g_ctx.mag_countdown)
    {
        int32_t field[LSM303_HW_AXES] = { 0, 0, 0 };

        g_ctx.mag_countdown = MAG_EVERY_PERIODS;

        if (lsm303_hw_read_mag(field))
        {
            g_ctx.heading_mdeg = fix_math_atan2_mdeg(field[LSM303_HW_AXIS_Y],
                                                     field[LSM303_HW_AXIS_X]);
        }
    }

    g_ctx.mag_countdown--;
}


/*!
 * @brief Classify the vehicle's motion.
 *
 * @param[in] p_enc  Latest encoder state.
 */
static void
classify (encoder_state_t const * p_enc)
{
    int32_t const      speed = (p_enc->left_speed_mm_s +
                                p_enc->right_speed_mm_s) / 2;
    int32_t const      accel = ((speed - g_ctx.prev_speed_mm_s) *
                                MDEG_PER_DEG) /
                               (int32_t) MS_PER_PERIOD;  // 20 ms: exact.
    int32_t const      turn  = ((p_enc->right_speed_mm_s -
                                 p_enc->left_speed_mm_s) * MDEG_PER_RAD) /
                               (int32_t) TRACK_WIDTH_MM;  // 120: exact.
    hump_state_t const hump  = hump_state(&g_ctx.hump);

    g_ctx.prev_speed_mm_s = speed;

    if (g_ctx.cooldown_ms > 0u)
    {
        g_ctx.motion_class = IMU_CLASS_IMPACT;
    }
    else if (HUMP_STATE_LEVEL != hump)
    {
        g_ctx.motion_class = (HUMP_STATE_CLIMBING == hump) ?
                             IMU_CLASS_CLIMBING : IMU_CLASS_DESCENDING;
    }
    else if (fix_math_abs(turn) > TURNING_MDEG_S)
    {
        g_ctx.motion_class = IMU_CLASS_TURNING;
    }
    else if (fix_math_abs(accel) > ACCEL_EVENT_MM_S2)
    {
        g_ctx.motion_class = (accel > 0) ? IMU_CLASS_ACCELERATING :
                                           IMU_CLASS_DECELERATING;
    }
    else
    {
        g_ctx.motion_class = (fix_math_abs(speed) < STILL_MM_S) ?
                             IMU_CLASS_STATIONARY : IMU_CLASS_CRUISING;
    }
}


/*!
 * @brief Feed the hump detector and report completed humps.
 *
 * @param[in] b_valid  Whether this IMU sample is valid.
 */
static void
track_hump (bool b_valid)
{
    uint32_t height_mm = 0u;

    if (hump_update(&g_ctx.hump, g_ctx.tilt_mdeg, b_valid, MS_PER_PERIOD,
                    &height_mm))
    {
        uint32_t data = 0u;

        g_ctx.last_hump_mm = height_mm;
        data = bitpack_word_set(data, IMU_HUMP_LAST_SHIFT, IMU_HUMP_MM_WIDTH,
                                bitpack_saturate(height_mm,
                                                 IMU_HUMP_MM_WIDTH));
        data = bitpack_word_set(data, IMU_HUMP_MAX_SHIFT, IMU_HUMP_MM_WIDTH,
                                bitpack_saturate(hump_max_mm(&g_ctx.hump),
                                                 IMU_HUMP_MM_WIDTH));
        telemetry_post_event(TELEMETRY_EVT_HUMP, data);
        buzzer_play(BUZZER_CUE_HUMP);  // Once: peak height calculated.
    }
}


/*!
 * @brief Feed the lift detector and tell Vehicle Control about a lift.
 *
 * @param[in] b_valid  Whether this IMU sample is valid.
 */
static void
track_lift (bool b_valid)
{
    if (lift_update(&g_ctx.lift, g_ctx.tilt_mdeg, b_valid, MS_PER_PERIOD))
    {
        g_lift_count = g_lift_count + 1u;  // Single writer: this task.
        robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_LIFTED);
        telemetry_post_event(TELEMETRY_EVT_LIFT, 0u);
    }
}


/*!
 * @brief Whether the robot is line following at a steady pace.
 *
 * @param[in] p_enc  Latest encoder state.
 *
 * @return true once MOTION_MODE_FOLLOWING has lasted HUMP_SETTLE_MS and
 *         the robot is not in a sharp turn.
 *
 * @par
 * The settle time covers the pull-away and the speed-up after a start, a
 * junction or a bypass.  A sharp turn is the same turn rate that classes
 * the motion as turning; after one, HUMP_AFTER_TURN_MS must pass before
 * the robot counts as steady again.
 */
static bool
is_following_steadily (encoder_state_t const * p_enc)
{
    uint32_t const mode = bitpack_word_get(sys_status_read(SYS_STATUS_MOTION),
                                           MOTION_MODE_SHIFT,
                                           MOTION_MODE_WIDTH);
    int32_t const  turn = ((p_enc->right_speed_mm_s -
                            p_enc->left_speed_mm_s) * MDEG_PER_RAD) /
                          (int32_t) TRACK_WIDTH_MM;  // 120: exact.

    if (MOTION_MODE_FOLLOWING != mode)
    {
        g_ctx.follow_ms = 0u;
    }
    else if (fix_math_abs(turn) > TURNING_MDEG_S)
    {
        // In a sharp turn: once it ends, wait HUMP_AFTER_TURN_MS more.
        if (g_ctx.follow_ms > (HUMP_SETTLE_MS - HUMP_AFTER_TURN_MS))
        {
            g_ctx.follow_ms = HUMP_SETTLE_MS - HUMP_AFTER_TURN_MS;
        }
    }
    else if (g_ctx.follow_ms < HUMP_SETTLE_MS)
    {
        g_ctx.follow_ms += MS_PER_PERIOD;
    }
    else
    {
        // Settled: nothing to count.
    }

    return ((g_ctx.follow_ms >= HUMP_SETTLE_MS) &&
            (fix_math_abs(turn) <= TURNING_MDEG_S));
}


/*!
 * @brief Publish the IMU and HUMP status words.
 *
 * @param[in] b_valid  Whether the latest sample was valid.
 */
static void
publish (bool b_valid)
{
    int32_t const  heading_deg = fix_math_wrap_mdeg(g_ctx.heading_mdeg) /
                                 MDEG_PER_DEG;

    // Heading in [0, 360) after the wrap correction: exact conversion.
    uint32_t const heading     = (uint32_t) ((heading_deg < 0) ?
                                             (heading_deg + FULL_TURN_DEG) :
                                             heading_deg);
    uint32_t       imu         = 0u;
    uint32_t       hump        = 0u;

    // The enumeration values are below 8, so the casts are exact.  The
    // pitch field carries the total tilt (pitch and roll combined, >= 0),
    // which is what the hump detector uses.
    imu  = bitpack_word_set(imu, IMU_PITCH_SHIFT, IMU_PITCH_WIDTH,
                            bitpack_from_signed(g_ctx.tilt_mdeg /
                                                MDEG_PER_HALF_DEG,
                                                IMU_PITCH_WIDTH));
    imu  = bitpack_word_set(imu, IMU_HEADING_SHIFT, IMU_HEADING_WIDTH,
                            heading);
    imu  = bitpack_word_set(imu, IMU_CLASS_SHIFT, IMU_CLASS_WIDTH,
                            (uint32_t) g_ctx.motion_class);  // Enum < 8.
    imu  = bitpack_word_set(imu, IMU_HUMP_SHIFT, IMU_HUMP_WIDTH,
                            (uint32_t) hump_state(&g_ctx.hump));  // Enum < 4.
    imu  = bitpack_word_set(imu, IMU_VALID_SHIFT, 1u, b_valid ? 1u : 0u);
    imu  = bitpack_word_set(imu, IMU_IMPACTS_SHIFT, IMU_IMPACTS_WIDTH,
                            g_ctx.impacts);
    hump = bitpack_word_set(hump, IMU_HUMP_LAST_SHIFT, IMU_HUMP_MM_WIDTH,
                            bitpack_saturate(g_ctx.last_hump_mm,
                                             IMU_HUMP_MM_WIDTH));
    hump = bitpack_word_set(hump, IMU_HUMP_MAX_SHIFT, IMU_HUMP_MM_WIDTH,
                            bitpack_saturate(hump_max_mm(&g_ctx.hump),
                                             IMU_HUMP_MM_WIDTH));
    hump = bitpack_word_set(hump, IMU_HUMP_COUNT_SHIFT, IMU_HUMP_COUNT_WIDTH,
                            bitpack_saturate(hump_count(&g_ctx.hump),
                                             IMU_HUMP_COUNT_WIDTH));
    sys_status_write(SYS_STATUS_IMU, imu);
    sys_status_write(SYS_STATUS_HUMP, hump);
}

/*** end of file ***/

