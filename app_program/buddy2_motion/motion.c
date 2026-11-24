/** @file motion.c
 *
 * @brief Motion Control Task: closed-loop wheel speed, line steering,
 *        encoder-based moves and turns, braking and stall detection.
 *
 * @par
 * Statechart: Week 6 Section 6.2 (IDLE, FOLLOWING, MOVING DISTANCE, TURNING,
 * BRAKING, STALLED).  Each wheel runs its own speed PID with a feed-forward
 * term that covers the motor dead band; FOLLOW adds a PD steering
 * correction from the line offset.
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
#include "motor_hw.h"
#include "encoder.h"
#include "pid_ctrl.h"
#include "line_follow.h"
#include "barcode.h"
#include "motion.h"
#include "tune.h"

#define WHEEL_LEFT              0u
#define WHEEL_RIGHT             1u
#define WHEEL_COUNT             2u
#define GAIN_SCALE              1000
#define PERMILLE_SCALE          1000
#define PERCENT_SCALE           100u
#define UM_PER_MM               1000u
#define SPEED_FIELD_WIDTH       10u
#define PID_TRIM_PERMILLE       500
#define BRAKE_DECAY_DIVISOR     2
#define BRAKE_TIMEOUT_MS        500u
#define STALL_GAP_FACTOR        3u
#define DECEL_GAIN              4u
#define DECEL_FLOOR_MM_S        80u
#define MS_PER_PERIOD           (PERIOD_BASE_MS * PERIOD_MOTION_TICKS)
#define ARG_MAGNITUDE_BITS      15u

// Signed copies of the unsigned tuning constants.  Every value is below
// 2^16, so each cast is exact.
#define MIN_DRIVE               ((int32_t) MOTOR_MIN_PERMILLE)
#define FEED_PER_M_S            ((int32_t) MOTOR_FF_PERMILLE_PER_M_S)

#define STOPPED_SPEED           ((int32_t) STOPPED_SPEED_MM_S)

// Wheel travel per degree of an on-the-spot turn is pi * track / 360, in
// um, using pi = 355/113 (120 mm track: 1047 um per degree).  The track is
// tunable live, so this is computed in turn_um_per_deg().
#define PI_NUM                  355u
#define PI_DEN                  113u
#define DEG_PER_REV             360u

/*!
 * @brief Motion Control working state (owned by the task).
 */
typedef struct
{
    motion_mode_t   mode;
    uint32_t        speed_mm_s;
    int32_t         direction;
    uint32_t        goal_um;
    int32_t         start_count[WHEEL_COUNT];
    uint32_t        mode_age_ms;
    int32_t         target[WHEEL_COUNT];
    int32_t         output[WHEEL_COUNT];
    int32_t         prev_offset;
    int32_t         steer_percent;  // Steering limit, % of the speed.
    uint32_t        straight_ms;    // Steering held for a barcode, ms.
    pid_ctrl_t      pid[WHEEL_COUNT];
} motion_ctx_t;

static motion_ctx_t g_ctx;

// DIAGNOSTIC BUILD ONLY: ring of line-following samples.
#define REC_SLOTS               800u
static motion_rec_t g_rec[REC_SLOTS];
static uint32_t     g_rec_next  = 0u;
static uint32_t     g_rec_count = 0u;
static uint32_t     g_rec_total = 0u;

// Lowest line-following speed of each wheel, in WHEEL_LEFT, WHEEL_RIGHT
// order.  Casts: small unsigned constants, exact.
static int32_t const g_wheel_min[WHEEL_COUNT] =
{
    (int32_t) FOLLOW_LEFT_MIN_MM_S, (int32_t) FOLLOW_RIGHT_MIN_MM_S
};
static uint64_t     g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void     motion_task(INT start_code, void * p_ext_info);
static bool     send_command(motion_cmd_type_t type, uint32_t speed_mm_s,
                             int32_t argument);
static void     take_commands(encoder_state_t const * p_enc);
static void     begin_command(uint32_t word, encoder_state_t const * p_enc);
static void     start_mode(uint32_t type, int32_t arg);
static void     set_targets(encoder_state_t const * p_enc);
static void     follow_targets(void);
static uint32_t travelled_um(encoder_state_t const * p_enc);
static uint32_t turn_um_per_deg(void);
static uint32_t turn_goal_um(int32_t angle_deg);
static void     drive_wheels(encoder_state_t const * p_enc);
static void     check_stall(encoder_state_t const * p_enc);
static bool     is_wheel_stalled(uint32_t wheel, uint32_t edge_age_ms);
static void     enter_mode(motion_mode_t mode);
static void     publish_status(encoder_state_t const * p_enc);
static int32_t  sign_of(int32_t value);


/*!
 * @brief Initialise the motor driver and encoders and start the task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
motion_start (void)
{
    int32_t result = 0;

    motor_hw_init();

    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        pid_ctrl_init(&g_ctx.pid[wheel], PID_KP_MILLI, PID_KI_MILLI,
                      PID_KD_MILLI, PID_TRIM_PERMILLE);
        g_ctx.target[wheel]      = 0;
        g_ctx.output[wheel]      = 0;
        g_ctx.start_count[wheel] = 0;
    }

    g_ctx.mode        = MOTION_MODE_IDLE;
    g_ctx.prev_offset = 0;
    g_ctx.steer_percent = (int32_t) STEER_LIMIT_PERCENT;  // < 100: exact.

    result = encoder_start();

    if (0 == result)
    {
        int32_t const h_task = robot_ipc_start_task(motion_task, PRIO_MOTION,
                                                    g_stack,
                                                    sizeof(g_stack));

        if (!period_tick_register(h_task, PERIOD_MOTION_TICKS))
        {
            result = -1;
        }
    }

    return (result);
}


/*!
 * @brief Drive forward a set distance (brief API moveForward).
 *
 * @param[in] distance_mm  Distance to travel.
 * @param[in] speed_mm_s   Cruise speed.
 *
 * @return true if the command was queued.
 */
bool
motion_move_forward (uint32_t distance_mm, uint32_t speed_mm_s)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const distance = (int32_t) bitpack_saturate(distance_mm,
                                                      ARG_MAGNITUDE_BITS);

    return (send_command(MOTION_CMD_MOVE, speed_mm_s, distance));
}


/*!
 * @brief Drive backward a set distance (brief API moveBackward).
 *
 * @param[in] distance_mm  Distance to travel.
 * @param[in] speed_mm_s   Cruise speed.
 *
 * @return true if the command was queued.
 */
bool
motion_move_backward (uint32_t distance_mm, uint32_t speed_mm_s)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const distance = (int32_t) bitpack_saturate(distance_mm,
                                                      ARG_MAGNITUDE_BITS);

    return (send_command(MOTION_CMD_MOVE, speed_mm_s, -distance));
}


/*!
 * @brief Turn on the spot to the left (brief API turnLeft).
 *
 * @param[in] angle_deg  Angle in degrees.
 *
 * @return true if the command was queued.
 */
bool
motion_turn_left (uint32_t angle_deg)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const angle = (int32_t) bitpack_saturate(angle_deg,
                                                   ARG_MAGNITUDE_BITS);

    // The tuned turn speed is 30..600, so the cast is exact.
    return (send_command(MOTION_CMD_TURN,
                         (uint32_t) tune_get(TUNE_SPEED_TURN), angle));
}


/*!
 * @brief Turn on the spot to the right (brief API turnRight).
 *
 * @param[in] angle_deg  Angle in degrees.
 *
 * @return true if the command was queued.
 */
bool
motion_turn_right (uint32_t angle_deg)
{
    // Clamped below 2^15 before the conversion, so it is exact.
    int32_t const angle = (int32_t) bitpack_saturate(angle_deg,
                                                   ARG_MAGNITUDE_BITS);

    // The tuned turn speed is 30..600, so the cast is exact.
    return (send_command(MOTION_CMD_TURN,
                         (uint32_t) tune_get(TUNE_SPEED_TURN), -angle));
}


/*!
 * @brief Follow the line at a cruise speed until another command arrives.
 *
 * @param[in] speed_mm_s  Base speed.
 *
 * @return true if the command was queued.
 */
bool
motion_follow_line (uint32_t speed_mm_s)
{
    return (send_command(MOTION_CMD_FOLLOW, speed_mm_s, 0));
}


/*!
 * @brief Follow the line with the steering limited to a smaller share of
 *        the speed than STEER_LIMIT_PERCENT.
 *
 * @param[in] speed_mm_s     Base speed.
 * @param[in] steer_percent  Steering limit, % of the speed (1 to
 *                           STEER_LIMIT_PERCENT).
 *
 * @return true if the command was queued.
 *
 * @par
 * Used for the slow start after rejoining the line, so a heading error is
 * pulled out gradually instead of swinging the wheels (2026-11-24).
 */
bool
motion_follow_line_gently (uint32_t speed_mm_s, uint32_t steer_percent)
{
    // steer_percent is a small setting below 2^31, so the cast is exact.
    int32_t const percent = fix_math_clamp((int32_t) steer_percent, 1,
                                           (int32_t) STEER_LIMIT_PERCENT);

    return (send_command(MOTION_CMD_FOLLOW, speed_mm_s, percent));
}


/*!
 * @brief Ramp down and stop (brief API stop).
 *
 * @return true if the command was queued.
 */
bool
motion_stop (void)
{
    return (send_command(MOTION_CMD_STOP, 0u, 0));
}


/*!
 * @brief DIAGNOSTIC BUILD ONLY: number of recorded samples.
 *
 * @return Count, at most REC_SLOTS.
 */
uint32_t
motion_rec_count (void)
{
    return (g_rec_count);
}


/*!
 * @brief DIAGNOSTIC BUILD ONLY: samples recorded since start-up.
 *
 * @return Total, which changes whenever the record has changed.
 */
uint32_t
motion_rec_serial (void)
{
    return (g_rec_total);
}


/*!
 * @brief DIAGNOSTIC BUILD ONLY: whether the record is not being written.
 *
 * @return true when the robot is not line following.
 */
bool
motion_rec_is_idle (void)
{
    return (MOTION_MODE_FOLLOWING != g_ctx.mode);
}


/*!
 * @brief DIAGNOSTIC BUILD ONLY: one recorded sample, oldest first.
 *
 * @param[in]  index  0 for the oldest sample.
 * @param[out] p_rec  The sample.
 */
void
motion_rec_get (uint32_t index, motion_rec_t * p_rec)
{
    uint32_t const first = (g_rec_next + REC_SLOTS - g_rec_count) % REC_SLOTS;

    *p_rec = g_rec[(first + index) % REC_SLOTS];
}


/*!
 * @brief Pack a motion command into its 29-bit word.
 *
 * @param[in] type        Command type.
 * @param[in] speed_mm_s  Speed, saturated to 10 bits.
 * @param[in] argument    Signed distance (mm) or angle (degrees).
 *
 * @return The command word.
 */
uint32_t
motion_pack_command (motion_cmd_type_t type, uint32_t speed_mm_s,
                     int32_t argument)
{
    uint32_t word = 0u;

    // The enumeration value is below 8, so the cast is exact.
    word = bitpack_word_set(word, MOTION_CMD_TYPE_SHIFT,
                            MOTION_CMD_TYPE_WIDTH, (uint32_t) type);
    word = bitpack_word_set(word, MOTION_CMD_SPEED_SHIFT,
                            MOTION_CMD_SPEED_WIDTH,
                            bitpack_saturate(speed_mm_s, SPEED_FIELD_WIDTH));
    word = bitpack_word_set(word, MOTION_CMD_ARG_SHIFT, MOTION_CMD_ARG_WIDTH,
                            bitpack_from_signed(argument,
                                                MOTION_CMD_ARG_WIDTH));

    return (word);
}


/*!
 * @brief Motion Control Task, released every 10 ms.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
motion_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        encoder_state_t enc;

        (void) period_tick_wait();  // Missed periods are counted centrally.

        encoder_get_state(&enc);
        take_commands(&enc);
        set_targets(&enc);
        drive_wheels(&enc);

        if (MOTION_MODE_FOLLOWING == g_ctx.mode)
        {
            motion_rec_t * const p_rec = &g_rec[g_rec_next];

            p_rec->line_word = line_follow_read();
            p_rec->target[0] = (int16_t) g_ctx.target[WHEEL_LEFT];  // Fits.
            p_rec->target[1] = (int16_t) g_ctx.target[WHEEL_RIGHT];  // Fits.
            p_rec->speed[0]  = (int16_t) enc.left_speed_mm_s;  // Fits.
            p_rec->speed[1]  = (int16_t) enc.right_speed_mm_s;  // Fits.
            p_rec->output[0] = (int16_t) g_ctx.output[WHEEL_LEFT];  // Fits.
            p_rec->output[1] = (int16_t) g_ctx.output[WHEEL_RIGHT];  // Fits.
            g_rec_next       = (g_rec_next + 1u) % REC_SLOTS;
            g_rec_count      = (g_rec_count < REC_SLOTS) ?
                               (g_rec_count + 1u) : REC_SLOTS;
            g_rec_total++;
        }

        check_stall(&enc);
        publish_status(&enc);
        g_ctx.mode_age_ms += MS_PER_PERIOD;
    }
}


/*!
 * @brief Pack a command and queue it without blocking the caller.
 *
 * @param[in] type        Command type.
 * @param[in] speed_mm_s  Speed.
 * @param[in] argument    Signed argument.
 *
 * @return true if queued.
 */
static bool
send_command (motion_cmd_type_t type, uint32_t speed_mm_s, int32_t argument)
{
    uint32_t const word = motion_pack_command(type, speed_mm_s, argument);
    uint8_t        msg[ROBOT_IPC_MOTION_MSG_BYTES];

    for (uint32_t idx = 0u; idx < ROBOT_IPC_MOTION_MSG_BYTES; idx++)
    {
        // Little-endian byte order; each byte is masked to 8 bits.
        msg[idx] = (uint8_t) ((word >> (idx * 8u)) & 0xFFu);
    }

    return (robot_ipc_send(ROBOT_IPC_MBF_MOTION_CMD, msg, sizeof(msg),
                           ROBOT_IPC_POLL));
}


/*!
 * @brief Apply every queued command; the newest one wins.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
take_commands (encoder_state_t const * p_enc)
{
    uint8_t  msg[ROBOT_IPC_MOTION_MSG_BYTES];
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_MOTION_CMD, msg,
                                        sizeof(msg), ROBOT_IPC_POLL);

    while (ROBOT_IPC_MOTION_MSG_BYTES == length)
    {
        uint32_t word = 0u;

        for (uint32_t idx = 0u; idx < ROBOT_IPC_MOTION_MSG_BYTES; idx++)
        {
            word |= ((uint32_t) msg[idx]) << (idx * 8u);  // Widening cast.
        }

        begin_command(word, p_enc);
        length = robot_ipc_receive(ROBOT_IPC_MBF_MOTION_CMD, msg,
                                   sizeof(msg), ROBOT_IPC_POLL);
    }
}


/*!
 * @brief Start executing one command word.
 *
 * @param[in] word   Packed command.
 * @param[in] p_enc  Current encoder state (start of travel).
 */
static void
begin_command (uint32_t word, encoder_state_t const * p_enc)
{
    uint32_t const type  = bitpack_word_get(word, MOTION_CMD_TYPE_SHIFT,
                                            MOTION_CMD_TYPE_WIDTH);
    int32_t const  arg   = bitpack_to_signed(
                               bitpack_word_get(word, MOTION_CMD_ARG_SHIFT,
                                                MOTION_CMD_ARG_WIDTH),
                               MOTION_CMD_ARG_WIDTH);
    uint32_t const speed = bitpack_word_get(word, MOTION_CMD_SPEED_SHIFT,
                                            MOTION_CMD_SPEED_WIDTH);

    // Only STOP may leave STALLED (Week 6 Section 6.2).
    if ((MOTION_MODE_STALLED != g_ctx.mode) || (MOTION_CMD_STOP == type))
    {
        g_ctx.speed_mm_s               = speed;
        g_ctx.direction                = (arg < 0) ? -1 : 1;
        g_ctx.start_count[WHEEL_LEFT]  = p_enc->left_count;
        g_ctx.start_count[WHEEL_RIGHT] = p_enc->right_count;
        start_mode(type, arg);
    }
}


/*!
 * @brief Enter the mode that executes a command.
 *
 * @param[in] type  Command type field.
 * @param[in] arg   Signed argument (distance in mm or angle in degrees).
 */
static void
start_mode (uint32_t type, int32_t arg)
{
    // |arg| is below 2^15, so the conversion to unsigned is exact.
    switch (type)
    {
        case MOTION_CMD_FOLLOW:
            // A positive argument is a lower steering limit (percent).
            // STEER_LIMIT_PERCENT is below 100, so the cast is exact.
            g_ctx.steer_percent = (arg > 0) ? arg :
                                  (int32_t) STEER_LIMIT_PERCENT;
            g_ctx.prev_offset = 0;
            enter_mode(MOTION_MODE_FOLLOWING);
        break;

        case MOTION_CMD_MOVE:
            g_ctx.goal_um = (uint32_t) fix_math_abs(arg) * UM_PER_MM;  // >=0.
            enter_mode(MOTION_MODE_MOVING);
        break;

        case MOTION_CMD_TURN:
            g_ctx.goal_um = turn_goal_um(arg);
            enter_mode(MOTION_MODE_TURNING);
        break;

        default:
            enter_mode(MOTION_MODE_BRAKING);
        break;
    }
}


/*!
 * @brief Work out each wheel's target speed for the current mode.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
set_targets (encoder_state_t const * p_enc)
{
    // The speed field is at most 1023, so it converts to int32_t exactly.
    int32_t const speed = (int32_t) g_ctx.speed_mm_s;

    switch (g_ctx.mode)
    {
        case MOTION_MODE_FOLLOWING:
            follow_targets();
        break;

        case MOTION_MODE_MOVING:
        {
            uint32_t const done_um = travelled_um(p_enc);
            uint32_t const left_mm = (g_ctx.goal_um > done_um) ?
                                     ((g_ctx.goal_um - done_um) / UM_PER_MM) :
                                     0u;
            uint32_t const ramp    = DECEL_FLOOR_MM_S + (left_mm * DECEL_GAIN);

            // ramp and speed are below 2^16: the conversion is exact.
            int32_t const  cruise  = (ramp < g_ctx.speed_mm_s) ?
                                     (int32_t) ramp : speed;

            g_ctx.target[WHEEL_LEFT]  = cruise * g_ctx.direction;
            g_ctx.target[WHEEL_RIGHT] = cruise * g_ctx.direction;

            if (0u == left_mm)
            {
                enter_mode(MOTION_MODE_BRAKING);
            }
        }
        break;

        case MOTION_MODE_TURNING:
            g_ctx.target[WHEEL_LEFT]  = -speed * g_ctx.direction;
            g_ctx.target[WHEEL_RIGHT] = speed * g_ctx.direction;

            if (travelled_um(p_enc) >= g_ctx.goal_um)
            {
                enter_mode(MOTION_MODE_BRAKING);
            }
        break;

        default:
            g_ctx.target[WHEEL_LEFT]  = 0;
            g_ctx.target[WHEEL_RIGHT] = 0;
        break;
    }
}


/*!
 * @brief Line-following targets: base speed plus PD steering.
 */
static void
follow_targets (void)
{
    uint32_t const word      = line_follow_read();
    int32_t const  base      = (int32_t) g_ctx.speed_mm_s;  // At most 1023.
    // Cast: the enumeration value is below 8, so the conversion is exact.
    bool const     b_reading = (((uint32_t) BARCODE_STATE_CAPTURING) ==
                                bitpack_word_get(
                                    sys_status_read(SYS_STATUS_BARCODE),
                                    BARCODE_STATE_SHIFT,
                                    BARCODE_STATE_WIDTH));
    int32_t        goal[WHEEL_COUNT];

    // While the barcode sensor is crossing bars, hold the steering so it
    // crosses them square and at a steady speed (the line is straight at
    // the barcodes), for at most BARCODE_STRAIGHT_MS in case it misfires.
    g_ctx.straight_ms = b_reading ? (g_ctx.straight_ms + MS_PER_PERIOD) : 0u;

    if (b_reading && (g_ctx.straight_ms <= BARCODE_STRAIGHT_MS))
    {
        goal[WHEEL_LEFT]  = base;
        goal[WHEEL_RIGHT] = base;
        g_ctx.prev_offset = tune_get(TUNE_STEER_DIR) *
                            line_follow_offset(word);  // No D kick after.
    }
    else if (line_follow_age_ms(word) > LINE_STALE_MS)
    {
        // Stale steering data: go straight and slowly (Week 6 negotiation
        // point with Buddy 3 on steering-data age).
        goal[WHEEL_LEFT]  = tune_get(TUNE_SPEED_SLOW);
        goal[WHEEL_RIGHT] = tune_get(TUNE_SPEED_SLOW);
    }
    else
    {
        int32_t const offset = tune_get(TUNE_STEER_DIR) *
                               line_follow_offset(word);
        int32_t const raw    = ((tune_get(TUNE_STEER_KP) * offset) +
                                (tune_get(TUNE_STEER_KD) *
                                 (offset - g_ctx.prev_offset))) /
                               GAIN_SCALE;
        // A negative steer slows the left wheel (a left curve); it may go
        // further (STEER_LEFT_LIMIT_PERCENT) unless following gently.
        // Cast: the percentage is at most 100, so the conversion is exact.
        int32_t const left   = (g_ctx.steer_percent <
                                (int32_t) STEER_LIMIT_PERCENT) ?
                               g_ctx.steer_percent :
                               (int32_t) STEER_LEFT_LIMIT_PERCENT;
        int32_t const limit  = (base * g_ctx.steer_percent) / 100;
        int32_t const steer  = fix_math_clamp(raw, -((base * left) / 100),
                                              limit);

        goal[WHEEL_LEFT]  = base + steer;
        goal[WHEEL_RIGHT] = base - steer;
        g_ctx.prev_offset = offset;

        // Keep each wheel above the speed where its motor sticks, and
        // give the other wheel what was added, so the turn rate is kept.
        for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
        {
            int32_t const lack = g_wheel_min[wheel] - goal[wheel];

            if (lack > 0)
            {
                goal[wheel]                           += lack;
                goal[(WHEEL_COUNT - 1u) - wheel]      += lack;
            }
        }
    }

    // Slew-limit each wheel so steering changes and the start from rest
    // are spread over several periods instead of one step.
    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        g_ctx.target[wheel] = fix_math_clamp(goal[wheel],
                                             g_ctx.target[wheel] -
                                             FOLLOW_SLEW_MM_S,
                                             g_ctx.target[wheel] +
                                             FOLLOW_SLEW_MM_S);
    }
}


/*!
 * @brief Wheel travel per degree of an on-the-spot turn, in um.
 *
 * @return pi * track / 360 for the current (tunable) track width.
 */
static uint32_t
turn_um_per_deg (void)
{
    // The tuned track is 50..300 mm, so the cast is exact.
    uint32_t const track_mm = (uint32_t) tune_get(TUNE_TRACK_MM);

    return ((track_mm * UM_PER_MM * PI_NUM) / (DEG_PER_REV * PI_DEN));
}


/*!
 * @brief Wheel travel for an on-the-spot turn, with the side's size
 *        correction applied.
 *
 * @param[in] angle_deg  Signed angle; positive is left (motion_turn_left).
 *
 * @return Travel of each wheel in um.
 *
 * @par
 * The robot coasts on after the wheels reach their distance, by a
 * different amount on each side, so each side has its own percentage
 * (TUNE_TURN_LEFT_PERCENT, TUNE_TURN_RIGHT_PERCENT).
 */
static uint32_t
turn_goal_um (int32_t angle_deg)
{
    // The tuned percentages are 50..150 and |angle| is below 2^15, so both
    // casts are exact and the product stays far below 2^32.
    uint32_t const percent = (uint32_t) tune_get((angle_deg >= 0) ?
                                                 TUNE_TURN_LEFT_PERCENT :
                                                 TUNE_TURN_RIGHT_PERCENT);
    uint32_t const angle   = (uint32_t) fix_math_abs(angle_deg);

    return ((angle * turn_um_per_deg() * percent) / PERCENT_SCALE);
}


/*!
 * @brief Mean absolute wheel travel since the current command started.
 *
 * @param[in] p_enc  Current encoder state.
 *
 * @return Travel in micrometres.
 */
static uint32_t
travelled_um (encoder_state_t const * p_enc)
{
    int32_t const left  = fix_math_abs(p_enc->left_count -
                                       g_ctx.start_count[WHEEL_LEFT]);
    int32_t const right = fix_math_abs(p_enc->right_count -
                                       g_ctx.start_count[WHEEL_RIGHT]);

    // Both counts are non-negative, so the conversion is exact.
    // The tuned edge length is 500..30000, so its cast is exact too.
    return (((uint32_t) (left + right) *
             (uint32_t) tune_get(TUNE_ENC_UM_PER_EDGE)) / 2u);
}


/*!
 * @brief Run the wheel speed controllers and drive the motors.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
drive_wheels (encoder_state_t const * p_enc)
{
    int32_t const measured[WHEEL_COUNT] =
    {
        p_enc->left_speed_mm_s, p_enc->right_speed_mm_s
    };

    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        int32_t const target = g_ctx.target[wheel];
        int32_t       drive  = 0;

        if (MOTION_MODE_BRAKING == g_ctx.mode)
        {
            // Ramp the last output down rather than reversing the motor.
            drive = g_ctx.output[wheel] / BRAKE_DECAY_DIVISOR;
        }
        else if ((0 != target) && (MOTION_MODE_STALLED != g_ctx.mode))
        {
            int32_t const feed = sign_of(target) *
                                 (MIN_DRIVE +
                                  ((fix_math_abs(target) * FEED_PER_M_S) /
                                   PERMILLE_SCALE));

            drive = feed + pid_ctrl_update(&g_ctx.pid[wheel], target,
                                           measured[wheel]);
        }
        else
        {
            pid_ctrl_reset(&g_ctx.pid[wheel]);
        }

        g_ctx.output[wheel] = fix_math_clamp(drive, -MOTOR_HW_FULL_PERMILLE,
                                             MOTOR_HW_FULL_PERMILLE);
    }

    encoder_set_direction(sign_of(g_ctx.output[WHEEL_LEFT]),
                          sign_of(g_ctx.output[WHEEL_RIGHT]));
    motor_hw_set(g_ctx.output[WHEEL_LEFT], g_ctx.output[WHEEL_RIGHT]);

    // Braking ends once both wheels have (almost) stopped, or after a time
    // limit, so encoder noise on a stopped wheel cannot hold up the next step.
    if ((MOTION_MODE_BRAKING == g_ctx.mode) &&
        (((fix_math_abs(p_enc->left_speed_mm_s) < STOPPED_SPEED) &&
          (fix_math_abs(p_enc->right_speed_mm_s) < STOPPED_SPEED)) ||
         (g_ctx.mode_age_ms >= BRAKE_TIMEOUT_MS)))
    {
        motor_hw_set(0, 0);
        enter_mode(MOTION_MODE_IDLE);
        robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_MOVE_DONE);
    }
}


/*!
 * @brief Detect a stalled wheel: drive applied but no encoder edges.
 *
 * @param[in] p_enc  Current encoder state.
 *
 * @par
 * Week 6: if PWM is above its minimum but no encoder pulse arrives for
 * STALL_TIMEOUT_MS, stop the motors and raise STALL.
 */
static void
check_stall (encoder_state_t const * p_enc)
{
    bool const b_driving = (MOTION_MODE_FOLLOWING == g_ctx.mode) ||
                           (MOTION_MODE_MOVING == g_ctx.mode) ||
                           (MOTION_MODE_TURNING == g_ctx.mode);
    bool const b_left    = is_wheel_stalled(WHEEL_LEFT,
                                            p_enc->left_edge_age_ms);
    bool const b_right   = is_wheel_stalled(WHEEL_RIGHT,
                                            p_enc->right_edge_age_ms);

    if (b_driving && (g_ctx.mode_age_ms > STALL_TIMEOUT_MS) &&
        (b_left || b_right))
    {
        motor_hw_set(0, 0);
        g_ctx.output[WHEEL_LEFT]  = 0;
        g_ctx.output[WHEEL_RIGHT] = 0;
        enter_mode(MOTION_MODE_STALLED);
        robot_ipc_raise_fault(ROBOT_FAULT_MOTOR_STALL);
        robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_STALL);
    }
}


/*!
 * @brief Change mode and restart the mode timer.
 *
 * @param[in] mode  New mode.
 */
static void
enter_mode (motion_mode_t mode)
{
    g_ctx.mode        = mode;
    g_ctx.mode_age_ms = 0u;

    // A new mode starts clean: trim wound up in a turn must not carry into
    // the next move, where it would keep the robot turning.
    for (uint32_t wheel = 0u; wheel < WHEEL_COUNT; wheel++)
    {
        pid_ctrl_reset(&g_ctx.pid[wheel]);
    }
}


/*!
 * @brief Decide whether one driven wheel has stopped producing edges.
 *
 * @param[in] wheel        WHEEL_LEFT or WHEEL_RIGHT.
 * @param[in] edge_age_ms  Time since the wheel's last edge.
 *
 * @return true if the wheel is driven and its edges are overdue.
 *
 * @par
 * A slowed inner wheel is still moving even when its edges are far apart,
 * so the limit grows with the edge interval expected at the target speed.
 */
static bool
is_wheel_stalled (uint32_t wheel, uint32_t edge_age_ms)
{
    int32_t const target    = fix_math_abs(g_ctx.target[wheel]);
    bool          b_stalled = false;

    if ((0 != target) && (fix_math_abs(g_ctx.output[wheel]) >= MIN_DRIVE))
    {
        // um per edge / mm per s = ms per edge.  Both are positive here, so
        // the casts are exact.
        uint32_t const gap_ms = (uint32_t) tune_get(TUNE_ENC_UM_PER_EDGE) /
                                (uint32_t) target;
        uint32_t const limit  = gap_ms * STALL_GAP_FACTOR;

        b_stalled = edge_age_ms > ((limit > STALL_TIMEOUT_MS) ?
                                   limit : STALL_TIMEOUT_MS);
    }

    return (b_stalled);
}


/*!
 * @brief Publish the SYS_STATUS_MOTION word.
 *
 * @param[in] p_enc  Current encoder state.
 */
static void
publish_status (encoder_state_t const * p_enc)
{
    uint32_t word = 0u;

    // The enumeration value is below 8, so the cast is exact.
    word = bitpack_word_set(word, MOTION_MODE_SHIFT, MOTION_MODE_WIDTH,
                            (uint32_t) g_ctx.mode);
    word = bitpack_word_set(word, MOTION_LEFT_SHIFT, MOTION_SPEED_WIDTH,
                            bitpack_from_signed(p_enc->left_speed_mm_s,
                                                MOTION_SPEED_WIDTH));
    word = bitpack_word_set(word, MOTION_RIGHT_SHIFT, MOTION_SPEED_WIDTH,
                            bitpack_from_signed(p_enc->right_speed_mm_s,
                                                MOTION_SPEED_WIDTH));
    word = bitpack_word_set(word, MOTION_STALLED_SHIFT, 1u,
                            (MOTION_MODE_STALLED == g_ctx.mode) ? 1u : 0u);
    sys_status_write(SYS_STATUS_MOTION, word);
}


/*!
 * @brief Sign of a value.
 *
 * @param[in] value  Any value.
 *
 * @return -1, 0 or +1.
 */
static int32_t
sign_of (int32_t value)
{
    int32_t sign = 0;

    if (value > 0)
    {
        sign = 1;
    }
    else if (value < 0)
    {
        sign = -1;
    }
    else
    {
        // Zero has no sign.
    }

    return (sign);
}


/*** end of file ***/