/** @file vehicle.c
 *
 * @brief Vehicle Control Task: system statechart and mission sequencing.
 *
 * @par
 * Multi-step manoeuvres (calibration sweep, junction action, bypass, line
 * search, reverse) are short lists of Motion Control steps.  Each step is
 * issued, and the next one starts when Motion Control reports MOVE_DONE.
 * MOVE_DONE is cleared before each step is issued; because this task has a
 * higher priority than Motion Control, the new command is always queued
 * before Motion Control can report completion of the previous one.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "board_hw.h"
#include "robot_ipc.h"
#include "sys_status.h"
#include "bitpack.h"
#include "fix_math.h"
#include "telemetry.h"
#include "tlm_codec.h"
#include "motion.h"
#include "encoder.h"
#include "line_follow.h"
#include "barcode.h"
#include "obstacle.h"
#include "nav_map.h"
#include "vehicle.h"
#include "tune.h"

#define WAIT_MS                 50
#define US_PER_MS               1000u
#define MAX_STEPS               8u
#define CALIB_SWEEP_DEG         25
#define SWEEP_UNIT_DEG          30
#define BITS_PER_BYTE           8u
#define TURN_90_DEG             90
#define TURN_180_DEG            180
#define CROSS_JUNCTION_MM       100
#define BYPASS_ANGLE_DEG        45
// Bypass step indices: 0 back up, 1 turn out, 2 out leg, 3 turn parallel,
// 4 pass leg, 5 turn in, 6 approach leg.
#define PASS_STEP               4u
#define APPROACH_LEG_STEP       6u
#define SQRT2_MILLI             1414
#define MILLI                   1000
#define MIN_PASS_MM             100
#define STATE_EVT_WIDTH         4u
#define MAP_ACK_LETTER_WIDTH    5u
#define MAP_ACK_ACTION_SHIFT    5u
#define MAP_ACK_VERSION_SHIFT   7u
#define MAP_ACK_OK_SHIFT        11u
// Signed copies of the unsigned geometry constants.  Each is below 2^16,
// so the casts are exact.
#define HALF_VEHICLE_MM         ((int32_t) (VEHICLE_WIDTH_MM / 2u))
#define MARGIN_MM               ((int32_t) CLEARANCE_MARGIN_MM)
#define DEPTH_EST_MM            ((int32_t) OBSTACLE_DEPTH_EST_MM)
#define LENGTH_MM               ((int32_t) VEHICLE_LENGTH_MM)
#define JUNCTION_ADVANCE        ((int32_t) JUNCTION_ADVANCE_MM)
#define BACKUP_MM               ((int32_t) BYPASS_BACKUP_MM)
#define MIN_LATERAL_MM          ((int32_t) BYPASS_LATERAL_MM)
#define LATERAL_PERCENT         ((int32_t) BYPASS_LATERAL_PERCENT)
#define LATERAL_EXTRA_MM        ((int32_t) BYPASS_LATERAL_EXTRA_MM)
#define REJOIN_GUARD_PERCENT    ((int32_t) BYPASS_REJOIN_GUARD_PERCENT)
#define PERCENT                 100
#define APPROACH_MAX_MM         ((int32_t) BYPASS_APPROACH_MAX_MM)
#define CENTRE_MM               ((int32_t) BYPASS_CENTRE_MM)
#define FATAL_FAULTS            (ROBOT_FAULT_MOTOR_STALL |                 \
                                 ROBOT_FAULT_ENCODER |                     \
                                 ROBOT_FAULT_NO_ECHO |                     \
                                 ROBOT_FAULT_IR_CALIB)
// NOTE: Collision detection is out of scope.  An impact is counted and
// reported by the IMU task but does not stop the run.
#define SAFETY_EVENTS           (ROBOT_EVT_STALL | ROBOT_EVT_ECHO_FAULT | \
                                 ROBOT_EVT_LIFTED)

// Bench build only (make ... BENCH_NO_IMU=1): start without a working IMU so
// the other subsystems can be tested.  The IMU fault stays raised and is
// still reported over USB and MQTT; only the start-up check ignores it.
// Hump height, lift and impact detection do not work in this build.  Never
// use it for the mission.
#ifndef BENCH_NO_IMU
#define BENCH_NO_IMU            0
#endif

#if BENCH_NO_IMU
#define START_FAULTS            0u
#else
#define START_FAULTS            ROBOT_FAULT_IMU
#endif

/*!
 * @brief Kinds of manoeuvre step.
 */
typedef enum
{
    STEP_FORWARD = 0,
    STEP_BACKWARD,
    STEP_TURN
} step_kind_t;

/*!
 * @brief Stages of the bypass.
 */
typedef enum
{
    BYPASS_DETOUR = 0,  // Around the obstacle, then towards the line.
    BYPASS_CENTRING,    // Line met: drive on until the axle is over it.
    BYPASS_REALIGN      // Turn back until the sensor meets the line.
} bypass_phase_t;

/*!
 * @brief One manoeuvre step: a distance in mm, or an angle in degrees
 *        (positive = left).
 */
typedef struct
{
    step_kind_t kind;
    int32_t     amount;
} step_t;

/*!
 * @brief Vehicle Control working state.
 */
typedef struct
{
    vehicle_state_t  state;
    uint32_t         entered_us;
    step_t           steps[MAX_STEPS];
    uint32_t         step_count;
    uint32_t         step_index;
    bool             b_steps_running;
    bool             b_scan_requested;
    bool             b_follow_slow;     // Still in the slow start of following.
    uint32_t         follow_slow_ms;    // Length of this slow start.
    bool             b_lost_pending;    // Line lost soon after resuming.
    uint32_t         lost_us;           // When that loss was reported.
    uint32_t         reacquire_slow_ms; // Slow start once the search ends.
    bypass_phase_t   phase;
    bool             b_searching;       // LINE_FOUND has been asked for.
    bool             b_approaching;     // The approach leg has started.
    int32_t          approach_from_mm;  // Odometer when it started.
    int32_t          rejoin_guard_mm;   // Travel before a line counts.
    bool             b_button_down;
    bool             b_latched;
    nav_map_action_t latched;
    nav_map_t        map;
    obs_profile_t    profile;
    uint32_t         junctions;
    uint32_t         bypasses;
    uint32_t         retries;
} vehicle_ctx_t;

static vehicle_ctx_t g_ctx;
static uint64_t      g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void vehicle_task(INT start_code, void * p_ext_info);
static void enter(vehicle_state_t state);
static bool is_running(void);
static void check_safety(uint32_t events);
static void take_commands(void);
static void apply_command(uint32_t word);
static void take_barcodes(void);
static void dispatch(uint32_t events);
static void on_line_following(uint32_t events);
static void on_obstacle_scan(uint32_t events);
static void on_bypass(uint32_t events, bool b_finished);
static void bypass_blocked(void);
static void bypass_detour(bool b_found, bool b_finished);
static void bypass_centre(void);
static void bypass_realign(void);
static void on_reacquire(uint32_t events, bool b_finished);
static void poll_button(void);
static void check_timeouts(void);
static void start_run(void);
static void follow(uint32_t slow_ms);
static bool relost_expired(void);
static void begin_junction(bool b_at_junction);
static void begin_reacquire(uint32_t then_slow_ms);
static void begin_bypass(void);
static void begin_plan(void);
static void run_steps(step_t const * p_steps, uint32_t count);
static void issue_step(void);
static bool advance_steps(uint32_t events);
static void publish_status(void);


/*!
 * @brief Start the Vehicle Control Task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
vehicle_start (void)
{
    g_ctx.state           = VEHICLE_INITIALISING;
    g_ctx.entered_us      = board_hw_time_us();
    g_ctx.step_count      = 0u;
    g_ctx.step_index      = 0u;
    g_ctx.b_steps_running = false;
    g_ctx.b_latched       = false;
    g_ctx.latched         = NAV_ACTION_STRAIGHT;
    g_ctx.b_button_down   = false;
    g_ctx.phase           = BYPASS_DETOUR;
    g_ctx.b_searching     = false;
    g_ctx.junctions       = 0u;
    g_ctx.bypasses        = 0u;
    g_ctx.retries         = 0u;
    nav_map_init(&g_ctx.map);

    return ((robot_ipc_start_task(vehicle_task, PRIO_VEHICLE, g_stack,
                                  sizeof(g_stack)) > 0) ? 0 : -1);
}


/*!
 * @brief Vehicle Control Task: wait for events and run the statechart.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
vehicle_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        uint32_t const events = robot_ipc_flag_wait(ROBOT_IPC_FLAG_VEHICLE,
                                                    ROBOT_EVT_ALL, WAIT_MS);

        check_safety(events);

        if (0u != (events & ROBOT_EVT_CTRL_CMD))
        {
            take_commands();
        }

        if (0u != (events & ROBOT_EVT_NAV_CMD))
        {
            take_barcodes();
        }

        dispatch(events);
        poll_button();
        check_timeouts();
        publish_status();
    }
}


/*!
 * @brief Change state, report it and restart the state timer.
 *
 * @param[in] state  New state.
 */
static void
enter (vehicle_state_t state)
{
    uint32_t data = 0u;

    // The enumeration values are below 16, so the casts are exact.
    data = bitpack_word_set(data, 0u, STATE_EVT_WIDTH, (uint32_t) state);
    data = bitpack_word_set(data, STATE_EVT_WIDTH, STATE_EVT_WIDTH,
                            (uint32_t) g_ctx.state);
    telemetry_post_event(TELEMETRY_EVT_STATE, data);  // Counted.

    g_ctx.state           = state;
    g_ctx.entered_us      = board_hw_time_us();
    g_ctx.b_steps_running = false;

    if ((VEHICLE_SAFE_STOP == state) || (VEHICLE_MISSION_COMPLETE == state))
    {
        (void) motion_stop();  // Queue drained every 10 ms: not full.
    }

    if (VEHICLE_SAFE_STOP == state)
    {
        telemetry_post_event(TELEMETRY_EVT_FAULT, robot_ipc_faults());
    }

    if (VEHICLE_MISSION_COMPLETE == state)
    {
        // Publish the run summary (FR5): Telemetry answers this request.
        robot_ipc_flag_set(ROBOT_IPC_FLAG_COMM, ROBOT_COMM_SUMMARY_REQ);
    }
}


/*!
 * @brief Report whether the robot is on a mission (motors may move).
 *
 * @return true from CALIBRATING to REVERSE_RETRY.
 */
static bool
is_running (void)
{
    return ((g_ctx.state >= VEHICLE_CALIBRATING) &&
            (g_ctx.state <= VEHICLE_REVERSE_RETRY));
}


/*!
 * @brief SAFE STOP on stall, echo fault, a lift or a fatal fault.
 *
 * @param[in] events  Events received this cycle.
 */
static void
check_safety (uint32_t events)
{
    bool const b_fault = (0u != (events & ROBOT_EVT_FAULT)) &&
                         (0u != (robot_ipc_faults() & FATAL_FAULTS));

    if ((is_running()) && ((0u != (events & SAFETY_EVENTS)) || b_fault))
    {
        enter(VEHICLE_SAFE_STOP);
    }
}


/*!
 * @brief Apply every queued MQTT command.
 */
static void
take_commands (void)
{
    uint8_t  msg[ROBOT_IPC_CTRL_MSG_BYTES];
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_CTRL_CMD, msg,
                                        sizeof(msg), ROBOT_IPC_POLL);

    while (ROBOT_IPC_CTRL_MSG_BYTES == length)
    {
        uint32_t word = 0u;

        // Least significant byte first; each byte is widened before the
        // shift.
        for (uint32_t idx = 0u; idx < ROBOT_IPC_CTRL_MSG_BYTES; idx++)
        {
            word |= ((uint32_t) msg[idx]) << (idx * BITS_PER_BYTE);  // Wide.
        }

        apply_command(word);
        length = robot_ipc_receive(ROBOT_IPC_MBF_CTRL_CMD, msg, sizeof(msg),
                                   ROBOT_IPC_POLL);
    }
}


/*!
 * @brief Act on one command word (tlm_codec.h layout).
 *
 * @param[in] word  Command word.
 */
static void
apply_command (uint32_t word)
{
    uint32_t const command = bitpack_word_get(word, TLM_CMD_TYPE_SHIFT,
                                              TLM_CMD_TYPE_WIDTH);
    uint32_t const letter  = bitpack_word_get(word, TLM_CMD_LETTER_SHIFT,
                                              TLM_CMD_LETTER_WIDTH);
    uint32_t const action  = bitpack_word_get(word, TLM_CMD_ACTION_SHIFT,
                                              TLM_CMD_ACTION_WIDTH);
    uint32_t const version = bitpack_word_get(word, TLM_CMD_VERSION_SHIFT,
                                              TLM_CMD_VERSION_WIDTH);
    uint32_t       ack     = 0u;

    switch (command)
    {
        case TLM_CMD_START:
            if (VEHICLE_READY == g_ctx.state)
            {
                start_run();
            }
        break;

        case TLM_CMD_STOP:
            if (is_running())
            {
                enter(VEHICLE_MISSION_COMPLETE);
            }
        break;

        case TLM_CMD_RESET:
            if (!is_running())
            {
                robot_ipc_flag_clear(ROBOT_IPC_FLAG_FAULT, FATAL_FAULTS);
                (void) motion_stop();  // Leaves STALLED (Week 6 6.2).
                enter(VEHICLE_READY);
            }
        break;

        case TLM_CMD_MAP_SET:
            // Applied immediately; affects the next barcode only.
            ack = bitpack_word_set(ack, 0u, MAP_ACK_LETTER_WIDTH, letter);
            ack = bitpack_word_set(ack, MAP_ACK_ACTION_SHIFT,
                                   NAV_MAP_ACTION_WIDTH, action);
            ack = bitpack_word_set(ack, MAP_ACK_VERSION_SHIFT,
                                   NAV_MAP_VERSION_WIDTH, version);
            ack = bitpack_word_set(ack, MAP_ACK_OK_SHIFT, 1u,
                                   nav_map_set(&g_ctx.map, letter, action,
                                               version) ? 1u : 0u);
            telemetry_post_event(TELEMETRY_EVT_MAP_ACK, ack);
        break;

        case TLM_CMD_SCAN:
            // Profile the obstacle in front on request, for checking the
            // width on the bench.  Only while stopped: during a run the
            // scans belong to OBSTACLE SCAN.  The PROFILE event reports the
            // result.  The buffer holds two profiles and OBSTACLE SCAN takes
            // the newest, so after one bench scan the run's own profile is
            // used; after two, the run's profile is dropped.
            if (!is_running())
            {
                obstacle_request_scan();
            }
        break;

        default:
            // Validated by Communication; nothing else can arrive.
        break;
    }
}


/*!
 * @brief Latch the action of the newest decoded barcode (FR3).
 *
 * @par
 * The letter is resolved through the map now, at decode time, so a later
 * map update cannot change the action already latched for the coming
 * junction.  A newer barcode replaces an older latched action.
 */
static void
take_barcodes (void)
{
    uint8_t  msg = 0u;
    uint32_t length = robot_ipc_receive(ROBOT_IPC_MBF_NAV_CMD, &msg,
                                        sizeof(msg), ROBOT_IPC_POLL);

    while (ROBOT_IPC_NAV_MSG_BYTES == length)
    {
        g_ctx.latched   = nav_map_get(&g_ctx.map, barcode_msg_letter(msg));
        g_ctx.b_latched = true;
        length          = robot_ipc_receive(ROBOT_IPC_MBF_NAV_CMD, &msg,
                                            sizeof(msg), ROBOT_IPC_POLL);
    }
}


/*!
 * @brief Run the handler of the current state.
 *
 * @param[in] events  Events received this cycle.
 */
static void
dispatch (uint32_t events)
{
    bool const b_finished = advance_steps(events);

    switch (g_ctx.state)
    {
        case VEHICLE_INITIALISING:
            // Wait for the IMU to calibrate while the robot is still.
            if ((board_hw_time_us() - g_ctx.entered_us) >=
                (INIT_SETTLE_MS * US_PER_MS))
            {
                enter((0u != (robot_ipc_faults() & START_FAULTS)) ?
                      VEHICLE_SAFE_STOP : VEHICLE_READY);
            }
        break;

        case VEHICLE_CALIBRATING:
            if (b_finished)
            {
                if (line_follow_end_calibration())
                {
                    follow(FOLLOW_SLOW_SHORT_MS);  // After calibration.
                }
                else
                {
                    enter(VEHICLE_SAFE_STOP);
                }
            }
        break;

        case VEHICLE_LINE_FOLLOWING:
            if ((g_ctx.b_follow_slow) &&
                ((board_hw_time_us() - g_ctx.entered_us) >=
                 (g_ctx.follow_slow_ms * US_PER_MS)))
            {
                // Settled on the edge: up to the full following speed.
                // The tuned speed is 30..600, so the cast is exact.
                g_ctx.b_follow_slow = false;
                (void) motion_follow_line(
                           (uint32_t) tune_get(TUNE_SPEED_FOLLOW));
            }

            if (!relost_expired())
            {
                on_line_following(events);
            }
        break;

        case VEHICLE_JUNCTION_ACTION:
            if (b_finished)
            {
                g_ctx.junctions++;
                follow(FOLLOW_SLOW_SHORT_MS);  // After a junction turn.
            }
        break;

        case VEHICLE_OBSTACLE_SCAN:
            on_obstacle_scan(events);
        break;

        case VEHICLE_BYPASS:
            on_bypass(events, b_finished);
        break;

        case VEHICLE_LINE_REACQUIRE:
            on_reacquire(events, b_finished);
        break;

        case VEHICLE_REVERSE_RETRY:
            if (b_finished)
            {
                // Already stopped: request the new profile straight away.
                enter(VEHICLE_OBSTACLE_SCAN);
                g_ctx.b_scan_requested = true;
                obstacle_request_scan();
            }
        break;

        default:
            // READY, SAFE STOP and MISSION COMPLETE wait for a command.
        break;
    }
}


/*!
 * @brief LINE FOLLOWING: obstacle first, then junction, then line loss.
 *
 * @param[in] events  Events received this cycle.
 */
static void
on_line_following (uint32_t events)
{
    if (0u != (events & ROBOT_EVT_OBSTACLE))
    {
        // The latched barcode action is kept for the next junction.
        g_ctx.b_scan_requested = false;
        g_ctx.retries          = 0u;
        enter(VEHICLE_OBSTACLE_SCAN);
        (void) motion_stop();  // Its MOVE_DONE starts the scan.
    }
    else if (0u != (events & ROBOT_EVT_JUNCTION))
    {
        if (g_ctx.b_latched)
        {
            begin_junction(true);
        }
        else
        {
            // No barcode action saved: every turn on the track has a
            // barcode before it, so this is a tight curve (the left sensor
            // reaching the line), not a junction.  Keep following.
            line_follow_clear_junction();
        }
    }
    else if (0u != (events & ROBOT_EVT_LINE_LOST))
    {
        if ((g_ctx.b_latched) && (NAV_ACTION_UTURN == g_ctx.latched))
        {
            // The line ended with a U-turn saved: run it here.
            begin_junction(false);
        }
        else if ((!g_ctx.b_lost_pending) &&
                 ((board_hw_time_us() - g_ctx.entered_us) <
                  (LINE_RELOST_MS * US_PER_MS)))
        {
            // Lost again just after resuming (a sharp bend): keep
            // following, steering hard to where the line went, before
            // searching (relost_expired).
            g_ctx.b_lost_pending = true;
            g_ctx.lost_us        = board_hw_time_us();
        }
        else
        {
            // Back to full following once found: a short loss needs the
            // full steering, not the gentle slow start.
            begin_reacquire(0u);
        }
    }
    else
    {
        // Keep following.
    }
}


/*!
 * @brief OBSTACLE SCAN: wait until stopped, request a profile, then plan.
 *
 * @param[in] events  Events received this cycle.
 */
static void
on_obstacle_scan (uint32_t events)
{
    if ((!g_ctx.b_scan_requested) && (0u != (events & ROBOT_EVT_MOVE_DONE)))
    {
        g_ctx.b_scan_requested = true;
        obstacle_request_scan();
    }
    else if ((g_ctx.b_scan_requested) &&
             (0u != (events & ROBOT_EVT_OBS_PROFILE)) &&
             (obstacle_get_profile(&g_ctx.profile)))
    {
        begin_plan();
    }
    else
    {
        // Still stopping or scanning.
    }
}


/*!
 * @brief BYPASS: run the detour, then rejoin the line in three stages.
 *
 * @param[in] events      Events received this cycle.
 * @param[in] b_finished  true if the step list has just finished.
 *
 * @par
 * DETOUR ends on the approach leg, which drives towards the line until the
 * centre sensor finds it.  CENTRING then drives on until the axle is over
 * the line, and REALIGN turns back towards the original heading until the
 * sensor meets the line again, so the robot ends up on the line and
 * pointing along it whatever angle it arrived at.
 *
 * @par
 * LINE_LOST and JUNCTION are ignored here: leaving the line is intended,
 * and rejoining it at an angle can look like a junction (Week 6 row 6).
 */
static void
on_bypass (uint32_t events, bool b_finished)
{
    // A LINE_FOUND counts only after the search for it was requested, so a
    // report left over from crossing another line is never used.
    bool const b_found   = (g_ctx.b_searching) &&
                           (0u != (events & ROBOT_EVT_LINE_FOUND));

    // Only the pass and approach legs are checked: while backing up and on
    // the way out, the obstacle being avoided is still in front.
    bool const b_blocked = (0u != (events & ROBOT_EVT_OBSTACLE)) &&
                           (BYPASS_DETOUR == g_ctx.phase) &&
                           (g_ctx.step_index >= PASS_STEP) &&
                           (STEP_FORWARD ==
                            g_ctx.steps[g_ctx.step_index].kind);

    if (b_blocked)
    {
        bypass_blocked();
    }
    else if (BYPASS_DETOUR == g_ctx.phase)
    {
        bypass_detour(b_found, b_finished);
    }
    else if (BYPASS_CENTRING == g_ctx.phase)
    {
        if (b_finished)
        {
            bypass_realign();
        }
    }
    else if (b_found)
    {
        // REALIGN met the line: back on it and pointing along it.
        g_ctx.bypasses++;
        follow(FOLLOW_SLOW_LONG_MS);   // After a bypass.
    }
    else if (b_finished)
    {
        // REALIGN turned its full angle without meeting the line.
        begin_reacquire(FOLLOW_SLOW_LONG_MS);
    }
    else
    {
        // Still turning back towards the line.
    }
}


/*!
 * @brief BYPASS: something else is in the way of the detour.
 *
 * @par
 * Stop and scan again from here, as many times as a reverse-and-retry
 * allows.
 */
static void
bypass_blocked (void)
{
    g_ctx.retries++;
    g_ctx.b_steps_running = false;

    if (g_ctx.retries > BYPASS_RETRIES_MAX)
    {
        enter(VEHICLE_SAFE_STOP);
    }
    else
    {
        g_ctx.b_scan_requested = false;
        enter(VEHICLE_OBSTACLE_SCAN);
        (void) motion_stop();  // Its MOVE_DONE starts the scan.
    }
}


/*!
 * @brief BYPASS, DETOUR stage: ask for the line on the approach leg.
 *
 * @param[in] b_found     true if the requested LINE_FOUND has arrived.
 * @param[in] b_finished  true if the step list has just finished.
 */
static void
bypass_detour (bool b_found, bool b_finished)
{
    if (b_found)
    {
        bypass_centre();
    }
    else if (b_finished)
    {
        // The approach leg reached its limit without finding the line.
        begin_reacquire(FOLLOW_SLOW_LONG_MS);
    }
    else if ((!g_ctx.b_searching) &&
             (g_ctx.step_index >= APPROACH_LEG_STEP))
    {
        encoder_state_t enc;

        encoder_get_state(&enc);

        if (!g_ctx.b_approaching)
        {
            // The approach leg has just started: note where.
            g_ctx.b_approaching    = true;
            g_ctx.approach_from_mm = enc.distance_mm;
        }
        else if ((enc.distance_mm - g_ctx.approach_from_mm) >=
                 g_ctx.rejoin_guard_mm)
        {
            // Far enough back: from now on the next line under the centre
            // sensor is the one being rejoined.
            g_ctx.b_searching = true;
            line_follow_begin_search();
        }
        else
        {
            // Still too far out: a line here is a neighbouring one.
        }
    }
    else
    {
        // Detour in progress.
    }
}


/*!
 * @brief BYPASS, CENTRING stage: the sensor is on the line, so drive on
 *        until the axle (the turning centre) is over it.
 *
 * @par
 * The new move replaces the approach leg without stopping.
 */
static void
bypass_centre (void)
{
    step_t const centre[] = { { STEP_FORWARD, CENTRE_MM } };

    g_ctx.phase       = BYPASS_CENTRING;
    g_ctx.b_searching = false;
    run_steps(centre, 1u);
}


/*!
 * @brief BYPASS, REALIGN stage: turn back towards the original heading
 *        until the centre sensor meets the line.
 *
 * @par
 * The sensor is now past the line, so the turn swings it back onto the
 * line exactly when the robot points along it.  The angle is only a limit.
 */
static void
bypass_realign (void)
{
    int32_t const angle     = (OBS_PLAN_PASS_LEFT == g_ctx.profile.plan) ?
                              BYPASS_REALIGN_MAX_DEG :
                              -BYPASS_REALIGN_MAX_DEG;
    step_t const  realign[] = { { STEP_TURN, angle } };

    g_ctx.phase       = BYPASS_REALIGN;
    g_ctx.b_searching = true;
    line_follow_begin_search();
    run_steps(realign, 1u);
}


/*!
 * @brief LINE REACQUIRE: sweep until LINE_FOUND.
 *
 * @param[in] events      Events received this cycle.
 * @param[in] b_finished  true if the sweep has finished.
 */
static void
on_reacquire (uint32_t events, bool b_finished)
{
    if (0u != (events & ROBOT_EVT_LINE_FOUND))
    {
        follow(g_ctx.reacquire_slow_ms);  // Set by begin_reacquire().
    }
    else if (b_finished)
    {
        enter(VEHICLE_SAFE_STOP);
    }
    else
    {
        // Still sweeping.
    }
}


/*!
 * @brief Robo Pico START button: start, stop or reset on each press.
 */
static void
poll_button (void)
{
    bool const b_down = board_hw_is_start_pressed();

    if (b_down && (!g_ctx.b_button_down))
    {
        if (VEHICLE_READY == g_ctx.state)
        {
            start_run();
        }
        else if (is_running())
        {
            // START during a run ends the course manually.
            enter(VEHICLE_MISSION_COMPLETE);
        }
        else if (VEHICLE_INITIALISING != g_ctx.state)
        {
            robot_ipc_flag_clear(ROBOT_IPC_FLAG_FAULT, FATAL_FAULTS);
            (void) motion_stop();  // Leaves STALLED.
            enter(VEHICLE_READY);
        }
        else
        {
            // Ignore presses while initialising.
        }
    }

    g_ctx.b_button_down = b_down;
}


/*!
 * @brief Abandon states that have run too long.
 */
static void
check_timeouts (void)
{
    uint32_t const age_ms = (board_hw_time_us() - g_ctx.entered_us) /
                            US_PER_MS;
    bool           b_late = false;

    switch (g_ctx.state)
    {
        case VEHICLE_LINE_REACQUIRE:
            b_late = (age_ms > REACQUIRE_TIMEOUT_MS);
        break;

        case VEHICLE_OBSTACLE_SCAN:
            b_late = (age_ms > PROFILE_TIMEOUT_MS);
        break;

        // These four step-list states share one limit (labels fall through).
        case VEHICLE_CALIBRATING:
        case VEHICLE_JUNCTION_ACTION:
        case VEHICLE_BYPASS:
        case VEHICLE_REVERSE_RETRY:
            b_late = (age_ms > SEQUENCE_TIMEOUT_MS);
        break;

        default:
            b_late = false;
        break;
    }

    if (b_late)
    {
        enter(VEHICLE_SAFE_STOP);
    }
}


/*!
 * @brief START: calibrate the IR sensors with a sweep across the line.
 */
static void
start_run (void)
{
    step_t const sweep[] =
    {
        { STEP_TURN, CALIB_SWEEP_DEG },
        { STEP_TURN, -2 * CALIB_SWEEP_DEG },
        { STEP_TURN, CALIB_SWEEP_DEG }
    };

    g_ctx.b_latched = false;
    enter(VEHICLE_CALIBRATING);
    line_follow_begin_calibration();
    run_steps(sweep, sizeof(sweep) / sizeof(sweep[0]));
}


/*!
 * @brief Return to LINE FOLLOWING.
 *
 * @param[in] slow_ms  How long to follow slowly and gently first:
 *                     FOLLOW_SLOW_LONG_MS after a bypass (or the search
 *                     that ends one), FOLLOW_SLOW_SHORT_MS otherwise, 0
 *                     for none (back on the line after a line loss).
 *
 * @par
 * The junction hold is released here as well, because rejoining the line
 * at an angle can leave the line task holding a false junction.
 */
static void
follow (uint32_t slow_ms)
{
    line_follow_clear_junction();
    enter(VEHICLE_LINE_FOLLOWING);
    // Following begins slowly and with gentle steering, because the robot
    // may still be carrying rotation and a heading error from a turn, or be
    // off the line edge after a stop.  dispatch() raises it to the full
    // speed and steering after slow_ms.
    // The 4-slot command queue cannot fill: this task issues only a few
    // commands per cycle and Motion Control drains the queue every 10 ms.
    g_ctx.b_lost_pending = false;
    g_ctx.b_follow_slow  = (slow_ms > 0u);
    g_ctx.follow_slow_ms = slow_ms;

    if (g_ctx.b_follow_slow)
    {
        // Cast: the queue cannot be full (see above), so it is ignored.
        (void) motion_follow_line_gently(FOLLOW_SLOW_SPEED_MM_S,
                                         FOLLOW_SLOW_STEER_PERCENT);
    }
    else
    {
        // The tuned speed is 30..600, so the cast is exact.
        (void) motion_follow_line((uint32_t) tune_get(TUNE_SPEED_FOLLOW));
    }
}


/*!
 * @brief LINE FOLLOWING: end the wait for a line lost just after resuming.
 *
 * @return true if the line search has started (the state has changed).
 *
 * @par
 * The wait ends when the line is under the sensors again, or after
 * LINE_RELOST_WAIT_MS without it, when the search starts as usual.
 */
static bool
relost_expired (void)
{
    line_follow_state_t const line    = line_follow_state(line_follow_read());
    bool                      b_start = false;

    if (!g_ctx.b_lost_pending)
    {
        // Nothing lost.
    }
    else if ((LINE_STATE_TRACKING == line) || (LINE_STATE_JUNCTION == line))
    {
        g_ctx.b_lost_pending = false;
    }
    else if ((board_hw_time_us() - g_ctx.lost_us) >=
             (LINE_RELOST_WAIT_MS * US_PER_MS))
    {
        g_ctx.b_lost_pending = false;
        begin_reacquire(0u);
        b_start = true;
    }
    else
    {
        // Still waiting for the line.
    }

    return (b_start);
}


/*!
 * @brief JUNCTION ACTION: execute the latched action (straight if none).
 *
 * @param[in] b_at_junction  true at a junction; false where the line ended.
 *
 * @par
 * Every turn, the U-turn included, is made on the spot after the axle has
 * reached the junction centre.  Where the line simply ended there is no
 * junction centre to drive to, so the saved U-turn is made where the robot
 * stands.
 */
static void
begin_junction (bool b_at_junction)
{
    nav_map_action_t const action = g_ctx.b_latched ? g_ctx.latched :
                                                      NAV_ACTION_STRAIGHT;
    step_t                 steps[2];
    uint32_t               count  = 2u;

    // The axle must reach the junction centre before turning.
    steps[0].kind   = STEP_FORWARD;
    steps[0].amount = JUNCTION_ADVANCE;
    steps[1].kind   = STEP_TURN;

    switch (action)
    {
        case NAV_ACTION_LEFT:
            steps[1].amount = TURN_90_DEG;
        break;

        case NAV_ACTION_RIGHT:
            steps[1].amount = -TURN_90_DEG;
        break;

        case NAV_ACTION_UTURN:
            steps[1].amount = TURN_180_DEG;
        break;

        default:
            steps[0].amount = CROSS_JUNCTION_MM;
            count           = 1u;
        break;
    }

    if ((NAV_ACTION_UTURN == action) && (!b_at_junction))
    {
        steps[0] = steps[1];
        count    = 1u;
    }

    // The action enumeration is below 4, so the cast is exact.
    telemetry_post_event(TELEMETRY_EVT_JUNCTION, (uint32_t) action);
    g_ctx.b_latched = false;
    enter(VEHICLE_JUNCTION_ACTION);
    run_steps(steps, count);
}


/*!
 * @brief LINE REACQUIRE: sweep left and right around the current heading.
 *
 * @param[in] then_slow_ms  Slow start once the line is found (see follow):
 *                          FOLLOW_SLOW_LONG_MS after a bypass, 0 after a
 *                          line lost while following.
 */
static void
begin_reacquire (uint32_t then_slow_ms)
{
    g_ctx.reacquire_slow_ms = then_slow_ms;

    // Start towards the side the line was last seen: the published offset
    // is positive when it went right (positive turn = left).  Then 1, 2,
    // 3, 4 and 2 sweep units alternating: the sweep ends on the heading it
    // started from.
    int32_t const side    = (line_follow_offset(line_follow_read()) > 0) ?
                            -1 : 1;
    step_t const  sweep[] =
    {
        { STEP_TURN, side * SWEEP_UNIT_DEG },
        { STEP_TURN, -side * 2 * SWEEP_UNIT_DEG },
        { STEP_TURN, side * 3 * SWEEP_UNIT_DEG },
        { STEP_TURN, -side * 4 * SWEEP_UNIT_DEG },
        { STEP_TURN, side * 2 * SWEEP_UNIT_DEG }
    };

    enter(VEHICLE_LINE_REACQUIRE);
    run_steps(sweep, sizeof(sweep) / sizeof(sweep[0]));
}


/*!
 * @brief Act on the obstacle profile's plan (brief stage 4).
 */
static void
begin_plan (void)
{
    int32_t const reverse_mm = tune_get(TUNE_REVERSE_MM);  // Live setting.
    step_t const  back[]     = { { STEP_BACKWARD, reverse_mm } };

    switch (g_ctx.profile.plan)
    {
        case OBS_PLAN_CONTINUE:
            follow(FOLLOW_SLOW_SHORT_MS);  // Nothing in the way.
        break;

        case OBS_PLAN_PASS_LEFT:
        case OBS_PLAN_PASS_RIGHT:
            begin_bypass();
        break;

        case OBS_PLAN_REVERSE:
            g_ctx.retries++;

            if (g_ctx.retries > BYPASS_RETRIES_MAX)
            {
                enter(VEHICLE_SAFE_STOP);
            }
            else
            {
                enter(VEHICLE_REVERSE_RETRY);
                run_steps(back, 1u);
            }
        break;

        default:
            enter(VEHICLE_SAFE_STOP);
        break;
    }
}


/*!
 * @brief BYPASS: back away a little, leave the line at 45 degrees, pass
 *        the obstacle, and come back at 45 degrees until LINE_FOUND.
 *
 * @par
 * The approach leg has no planned length: it ends when the line is found
 * (on_bypass), and APPROACH_MAX_MM only bounds a miss.
 */
static void
begin_bypass (void)
{
    int32_t const side    = (OBS_PLAN_PASS_LEFT == g_ctx.profile.plan) ?
                            1 : -1;

    // Profile distances are at most 2550 mm (8-bit cm fields), so both
    // conversions to int32_t are exact.
    int32_t const width   = (int32_t) g_ctx.profile.width_mm;
    int32_t const ahead   = (int32_t) g_ctx.profile.distance_mm + BACKUP_MM;
    int32_t const needed  = (((width / 2) + HALF_VEHICLE_MM + MARGIN_MM) *
                             LATERAL_PERCENT) / PERCENT;
    int32_t const lateral = ((needed > MIN_LATERAL_MM) ? needed :
                                                         MIN_LATERAL_MM) +
                            LATERAL_EXTRA_MM;
    int32_t const leg     = (lateral * SQRT2_MILLI) / MILLI;
    int32_t const pass    = fix_math_clamp(ahead + DEPTH_EST_MM +
                                           LENGTH_MM - lateral,
                                           MIN_PASS_MM, INT32_MAX);
    step_t const  detour[] =
    {
        { STEP_BACKWARD, BACKUP_MM },
        { STEP_TURN, side * BYPASS_ANGLE_DEG },
        { STEP_FORWARD, leg },
        { STEP_TURN, -side * BYPASS_ANGLE_DEG },
        { STEP_FORWARD, pass },
        { STEP_TURN, -side * BYPASS_ANGLE_DEG },
        { STEP_FORWARD, APPROACH_MAX_MM }
    };

    // The way back to the line is as long as the way out.  A line met in
    // the first part of it is not the one that was left.
    g_ctx.rejoin_guard_mm = (leg * REJOIN_GUARD_PERCENT) / PERCENT;
    g_ctx.b_approaching   = false;
    g_ctx.phase           = BYPASS_DETOUR;
    g_ctx.b_searching     = false;
    enter(VEHICLE_BYPASS);
    run_steps(detour, sizeof(detour) / sizeof(detour[0]));
}


/*!
 * @brief Load a step list and issue its first step.
 *
 * @param[in] p_steps  Steps (copied).
 * @param[in] count    Number of steps, at most MAX_STEPS.
 */
static void
run_steps (step_t const * p_steps, uint32_t count)
{
    g_ctx.step_count = (count < MAX_STEPS) ? count : MAX_STEPS;

    for (uint32_t idx = 0u; idx < g_ctx.step_count; idx++)
    {
        g_ctx.steps[idx] = p_steps[idx];
    }

    g_ctx.step_index      = 0u;
    g_ctx.b_steps_running = (g_ctx.step_count > 0u);

    if (g_ctx.b_steps_running)
    {
        issue_step();
    }
}


/*!
 * @brief Send the current step to Motion Control.
 */
static void
issue_step (void)
{
    step_t const   step   = g_ctx.steps[g_ctx.step_index];

    // |amount| is below 2^15 for every step, so the conversion is exact.
    uint32_t const amount = (uint32_t) fix_math_abs(step.amount);

    robot_ipc_flag_clear(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_MOVE_DONE);

    switch (step.kind)
    {
        case STEP_FORWARD:
            // Queued.  The tuned speed is 30..600: the cast is exact.
            (void) motion_move_forward(amount,
                                       (uint32_t) tune_get(TUNE_SPEED_SLOW));
        break;

        case STEP_BACKWARD:
            // Queued.  The tuned speed is 30..600: the cast is exact.
            (void) motion_move_backward(amount,
                                        (uint32_t) tune_get(TUNE_SPEED_SLOW));
        break;

        default:
            if (step.amount >= 0)
            {
                (void) motion_turn_left(amount);  // Queued.
            }
            else
            {
                (void) motion_turn_right(amount);  // Queued.
            }
        break;
    }
}


/*!
 * @brief Advance the step list on MOVE_DONE.
 *
 * @param[in] events  Events received this cycle.
 *
 * @return true when the last step has just completed.
 */
static bool
advance_steps (uint32_t events)
{
    bool b_finished = false;

    if ((g_ctx.b_steps_running) && (0u != (events & ROBOT_EVT_MOVE_DONE)))
    {
        g_ctx.step_index++;

        if (g_ctx.step_index < g_ctx.step_count)
        {
            issue_step();
        }
        else
        {
            g_ctx.b_steps_running = false;
            b_finished            = true;
        }
    }

    return (b_finished);
}


/*!
 * @brief Publish the SYS_STATUS_VEHICLE word.
 */
static void
publish_status (void)
{
    uint32_t word = 0u;

    // The enumeration values are below 16, so the casts are exact.
    word = bitpack_word_set(word, VEHICLE_STATE_SHIFT, VEHICLE_STATE_WIDTH,
                            (uint32_t) g_ctx.state);  // Enum < 16.
    word = bitpack_word_set(word, VEHICLE_ACTION_SHIFT, VEHICLE_ACTION_WIDTH,
                            (uint32_t) g_ctx.latched);  // Enum < 4.
    word = bitpack_word_set(word, VEHICLE_LATCHED_SHIFT, 1u,
                            g_ctx.b_latched ? 1u : 0u);
    word = bitpack_word_set(word, VEHICLE_MAP_SHIFT, VEHICLE_MAP_WIDTH,
                            nav_map_version(&g_ctx.map));
    word = bitpack_word_set(word, VEHICLE_JUNCTIONS_SHIFT,
                            VEHICLE_JUNCTIONS_WIDTH,
                            bitpack_saturate(g_ctx.junctions,
                                             VEHICLE_JUNCTIONS_WIDTH));
    word = bitpack_word_set(word, VEHICLE_BYPASSES_SHIFT,
                            VEHICLE_BYPASSES_WIDTH,
                            bitpack_saturate(g_ctx.bypasses,
                                             VEHICLE_BYPASSES_WIDTH));
    word = bitpack_word_set(word, VEHICLE_RETRIES_SHIFT,
                            VEHICLE_RETRIES_WIDTH,
                            bitpack_saturate(g_ctx.retries,
                                             VEHICLE_RETRIES_WIDTH));
    sys_status_write(SYS_STATUS_VEHICLE, word);
}

/*** end of file ***/

