/** @file line_follow.c
 *
 * @brief Line Following Task: offset estimation and junction, line-lost and
 *        line-found detection.
 *
 * @par
 * Statechart: Week 6 Section 6.3.  Debounce intervals are defined in time
 * (milliseconds), not sample counts, so they do not change if the task
 * period changes (Tribunal 1 decision on assumption A8).
 *
 * @par
 * Calibration: Vehicle Control sweeps the robot across the line while this
 * task records the darkest and brightest centre readings.  The range then
 * keeps widening during the run if a reading falls outside it, which absorbs
 * slow lighting changes (NFR7).
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
#include "period_tick.h"
#include "sys_status.h"
#include "bitpack.h"
#include "fix_math.h"
#include "telemetry.h"
#include "ir_hw.h"
#include "line_follow.h"

#define NORM_FULL               1000
#define NORM_HALF               500
#define NORM_BLACK              700
#define NORM_WHITE              200
#define NORM_FOUND              500

// Off-edge steering ramps, in offset units per 10 ms period.  The robot
// follows the line's right edge.  A white centre usually means the line has
// moved to the left, but if the centre saw black within CROSSED_MS it has
// crossed the whole line (a right curve taken too wide) and the line is to
// the right until the centre reaches black again.  A centre deep in black
// means the line is curving right under it.  In each case the offset ramps
// towards the line's side instead of jumping or sitting at +/-500, so the
// turn tightens the longer the condition lasts and the D term sees no
// step.  The left sensor seeing the line speeds the left ramp up and always
// means left.  Back on the edge, the offset returns towards the sensor
// value at most RELEASE per period.  Rates chosen in a host simulation
// (notes/robot-turn-simulation-2026-11-04.md).
// 2026-11-04: ramps roughly halved (were 100 / 200 / 200 / 30) for smoother
// recovery at the slower follow speed, which covers half the distance in
// the same time.
// 2026-11-17 (build 8): 30 / 60 was tried and put back to 50 / 100; the
// logs showed no less shake (the follow speed was lowered instead).
#define WHITE_RAMP              50
#define WHITE_RAMP_LEFT         100
#define WHITE_RELEASE           80
#define CROSSED_MS              150u
#define BLACK_RAMP              15
#define US_PER_STAMP_TICK       10000u
#define STAMP_MASK              0x1Fu
#define MS_PER_STAMP_TICK       10u
#define MS_PER_PERIOD           (PERIOD_BASE_MS * PERIOD_LINE_TICKS)

// Default range used until a calibration succeeds.  The ADC limit is 4095,
// so the conversions to int32_t are exact.
#define DEFAULT_WHITE_RAW       300
#define DEFAULT_BLACK_RAW       3000
#define ADC_MAX                 ((int32_t) IR_HW_ADC_MAX)

/*!
 * @brief Line Following working state (owned by the task).
 */
typedef struct
{
    line_follow_state_t state;
    int32_t             white_raw;
    int32_t             black_raw;
    uint32_t            junction_ms;
    uint32_t            lost_ms;
    uint32_t            found_ms;
    int32_t             prev_offset;
    uint32_t            since_black_ms;
    int32_t             white_side;
} line_ctx_t;

static line_ctx_t       g_ctx;

// Hand-shake with Vehicle Control.  Every variable has exactly one writer
// task and is one aligned word, so plain loads and stores are atomic.  A
// request is a sequence number the requester increments; the line task
// acts when the number differs from the last one it handled.
static bool volatile     gb_calibrating = false;   // Writer: Vehicle.
static uint32_t volatile g_calib_seq    = 0u;      // Writer: Vehicle.
static int32_t volatile  g_calib_white  = DEFAULT_WHITE_RAW;  // Vehicle.
static int32_t volatile  g_calib_black  = DEFAULT_BLACK_RAW;  // Vehicle.
static uint32_t volatile g_clear_seq    = 0u;      // Writer: Vehicle.
static uint32_t volatile g_search_seq   = 0u;      // Writer: Vehicle.
static int32_t volatile  g_calib_min    = ADC_MAX; // Writer: line task.
static int32_t volatile  g_calib_max    = 0;       // Writer: line task.
static uint32_t          g_calib_done   = 0u;      // Line task only.
static uint32_t          g_clear_done   = 0u;      // Line task only.
static uint32_t          g_search_done  = 0u;      // Line task only.

static uint64_t         g_stack[STACK_SMALL / sizeof(uint64_t)];

static void    line_follow_task(INT start_code, void * p_ext_info);
static void    track_calibration(int32_t sample);
static int32_t normalise(int32_t raw);
static void    update_events(int32_t norm, bool b_left_black);
static void    publish(int32_t offset, uint32_t raw, bool b_left_black);
static int32_t shape_offset(int32_t offset, int32_t norm,
                            bool b_left_black);


/*!
 * @brief Configure the IR inputs and start the Line Following Task.
 *
 * @return 0 on success, or a negative error code.
 */
int32_t
line_follow_start (void)
{
    int32_t result = 0;
    int32_t h_task = 0;

    ir_hw_init();
    g_ctx.state       = LINE_STATE_TRACKING;
    g_ctx.white_raw   = DEFAULT_WHITE_RAW;
    g_ctx.black_raw   = DEFAULT_BLACK_RAW;
    g_ctx.junction_ms = 0u;
    g_ctx.lost_ms     = 0u;
    g_ctx.found_ms    = 0u;
    g_ctx.prev_offset    = 0;
    g_ctx.since_black_ms = CROSSED_MS;
    g_ctx.white_side     = -1;

    h_task = robot_ipc_start_task(line_follow_task, PRIO_LINE, g_stack,
                                  sizeof(g_stack));

    if (!period_tick_register(h_task, PERIOD_LINE_TICKS))
    {
        result = -1;
    }

    return (result);
}


/*!
 * @brief Latest line_state word (one atomic load).
 *
 * @return Packed line_state.
 */
uint32_t
line_follow_read (void)
{
    return (sys_status_read(SYS_STATUS_LINE));
}


/*!
 * @brief Extract the signed offset from a line_state word.
 *
 * @param[in] word  Packed line_state.
 *
 * @return Offset, -1000 to +1000 (positive: steer right).
 */
int32_t
line_follow_offset (uint32_t word)
{
    return (bitpack_to_signed(bitpack_word_get(word, LINE_OFFSET_SHIFT,
                                               LINE_OFFSET_WIDTH),
                              LINE_OFFSET_WIDTH));
}


/*!
 * @brief Extract the state from a line_state word.
 *
 * @param[in] word  Packed line_state.
 *
 * @return Line Following state.
 */
line_follow_state_t
line_follow_state (uint32_t word)
{
    uint32_t const field = bitpack_word_get(word, LINE_STATE_SHIFT,
                                            LINE_STATE_WIDTH);
    line_follow_state_t state = LINE_STATE_CALIBRATING;

    switch (field)
    {
        case LINE_STATE_TRACKING:
            state = LINE_STATE_TRACKING;
        break;

        case LINE_STATE_JUNCTION:
            state = LINE_STATE_JUNCTION;
        break;

        case LINE_STATE_LOST:
            state = LINE_STATE_LOST;
        break;

        case LINE_STATE_SEARCHING:
            state = LINE_STATE_SEARCHING;
        break;

        default:
            state = LINE_STATE_CALIBRATING;
        break;
    }

    return (state);
}


/*!
 * @brief Age of a line_state word, from its 5-bit timestamp.
 *
 * @param[in] word  Packed line_state.
 *
 * @return Age in milliseconds (wraps every 320 ms, which is far longer than
 *         the 50 ms staleness limit Motion Control applies).
 */
uint32_t
line_follow_age_ms (uint32_t word)
{
    uint32_t const now   = (board_hw_time_us() / US_PER_STAMP_TICK) &
                           STAMP_MASK;
    uint32_t const stamp = bitpack_word_get(word, LINE_STAMP_SHIFT,
                                            LINE_STAMP_WIDTH);

    return (((now - stamp) & STAMP_MASK) * MS_PER_STAMP_TICK);
}


/*!
 * @brief Start recording the darkest and brightest centre readings.
 */
void
line_follow_begin_calibration (void)
{
    gb_calibrating = true;
}


/*!
 * @brief Stop recording and apply the calibration if the contrast is good.
 *
 * @return true if the contrast reached LINE_MIN_CONTRAST; false raises
 *         ROBOT_FAULT_IR_CALIB and keeps the previous range.
 */
bool
line_follow_end_calibration (void)
{
    int32_t const lowest  = g_calib_min;
    int32_t const highest = g_calib_max;

    // LINE_MIN_CONTRAST is 400, so the conversion is exact.
    bool const    b_ok    = (highest - lowest) >= (int32_t) LINE_MIN_CONTRAST;

    gb_calibrating = false;

    if (b_ok)
    {
        // Values first, sequence number last: the line task reads them in
        // the opposite order.
        g_calib_white = (0u != IR_AO_BLACK_IS_HIGH) ? lowest : highest;
        g_calib_black = (0u != IR_AO_BLACK_IS_HIGH) ? highest : lowest;
        g_calib_seq++;
    }
    else
    {
        robot_ipc_raise_fault(ROBOT_FAULT_IR_CALIB);
    }

    return (b_ok);
}


/*!
 * @brief Tell the line task that Vehicle Control is following again.
 *
 * @par
 * Releases a held JUNCTION and restarts line-loss detection.
 */
void
line_follow_clear_junction (void)
{
    g_clear_seq++;
}


/*!
 * @brief Ask for LINE_FOUND the next time the centre sensor meets a line.
 *
 * @par
 * Used when Vehicle Control drives or turns towards a line it has left on
 * purpose (the bypass return).  The line task moves to SEARCHING at once,
 * whatever it was doing, so a junction held from crossing another line
 * during the detour cannot block the report, and a LINE_FOUND raised after
 * this call always belongs to this search.
 */
void
line_follow_begin_search (void)
{
    g_search_seq++;
}


/*!
 * @brief Line Following Task, released every 10 ms.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
line_follow_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        uint32_t raw          = 0u;
        int32_t  sample       = 0;
        int32_t  norm         = 0;
        int32_t  offset       = 0;
        bool     b_left_black = false;

        (void) period_tick_wait();  // Missed periods are counted centrally.

        raw          = ir_hw_read_centre();
        sample       = (int32_t) raw;  // 12-bit reading: exact conversion.
        b_left_black = ir_hw_is_left_black();

        track_calibration(sample);
        norm   = normalise(sample);
        offset = shape_offset(norm - NORM_HALF, norm, b_left_black);

        if (LINE_STATE_CALIBRATING != g_ctx.state)
        {
            update_events(norm, b_left_black);
        }

        publish(offset, raw, b_left_black);
    }
}


/*!
 * @brief Ramp the offset while the centre is off the line.
 *
 * @param[in] offset        Raw offset, norm - NORM_HALF.
 * @param[in] norm          Normalised centre darkness.
 * @param[in] b_left_black  Left sensor sees black.
 *
 * @return Offset to publish.
 *
 * @par
 * On entering white the side is chosen once: right if the centre saw black
 * within CROSSED_MS, otherwise left; the left sensor forces left.  The
 * offset then moves towards that side's limit by a fixed step each period.
 * On the edge it follows the sensor, except that after a white spell it
 * may not move faster than WHITE_RELEASE towards it.
 */
static int32_t
shape_offset (int32_t offset, int32_t norm, bool b_left_black)
{
    int32_t const prev    = g_ctx.prev_offset;
    bool const    b_white = (norm < NORM_WHITE);
    int32_t       result  = offset;

    if (b_white && (g_ctx.white_side < 0) && (prev >= -NORM_HALF) &&
        (prev <= NORM_HALF))
    {
        // Just left the line: it crossed to the far side if the centre saw
        // black just before.
        g_ctx.white_side = (g_ctx.since_black_ms < CROSSED_MS) ? 1 : -1;
    }

    if (b_left_black || (norm > NORM_BLACK))
    {
        // Back on the line (or the left sensor sees it): normal edge rules.
        g_ctx.white_side = -1;
    }

    if (norm > NORM_BLACK)
    {
        g_ctx.since_black_ms = 0u;
    }
    else if (g_ctx.since_black_ms < CROSSED_MS)
    {
        g_ctx.since_black_ms += MS_PER_PERIOD;
    }
    else
    {
        // Saturated: black is long past.
    }

    if (g_ctx.white_side > 0)
    {
        // Crossed: the centre is on the line's far (left) side.  Keep
        // turning right until it reaches black, even over the grey edge,
        // where the raw offset would point the wrong way.
        int32_t const start = (prev > 0) ? prev : 0;

        result = fix_math_clamp(start + WHITE_RAMP, -LINE_OFFSET_LIMIT,
                                LINE_OFFSET_LIMIT);
    }
    else if (b_white)
    {
        // Line to the left: turn left harder the longer it stays white.
        int32_t const step  = b_left_black ? WHITE_RAMP_LEFT : WHITE_RAMP;
        int32_t const start = (prev < offset) ? prev : offset;

        result = fix_math_clamp(start - step, -LINE_OFFSET_LIMIT,
                                LINE_OFFSET_LIMIT);
    }
    else if ((norm > NORM_BLACK) && (!b_left_black))
    {
        // Centre deep in black: the line is curving right under it.  Turn
        // harder the longer it stays there, as on the white side.
        int32_t const start = (prev > offset) ? prev : offset;

        result = fix_math_clamp(start + BLACK_RAMP, -LINE_OFFSET_LIMIT,
                                LINE_OFFSET_LIMIT);
    }
    else if ((prev < -NORM_HALF) && (offset > (prev + WHITE_RELEASE)))
    {
        result = prev + WHITE_RELEASE;
    }
    else if ((prev > NORM_HALF) && (offset < (prev - WHITE_RELEASE)))
    {
        result = prev - WHITE_RELEASE;
    }
    else
    {
        // On the edge: use the sensor value.
    }

    g_ctx.prev_offset = result;

    return (result);
}


/*!
 * @brief Calibration steps for one sample.
 *
 * @param[in] sample  Raw centre reading.
 *
 * @par
 * While Vehicle Control sweeps, record the reading extremes.  When it
 * publishes a new calibration, adopt the range.
 */
static void
track_calibration (int32_t sample)
{
    uint32_t const request = g_calib_seq;

    if (gb_calibrating)
    {
        if (LINE_STATE_CALIBRATING != g_ctx.state)
        {
            g_ctx.state = LINE_STATE_CALIBRATING;
            g_calib_min = ADC_MAX;
            g_calib_max = 0;
        }

        g_calib_min = (sample < g_calib_min) ? sample : g_calib_min;
        g_calib_max = (sample > g_calib_max) ? sample : g_calib_max;
    }
    else if (LINE_STATE_CALIBRATING == g_ctx.state)
    {
        g_ctx.state = LINE_STATE_TRACKING;
    }
    else
    {
        // Not calibrating.
    }

    if (request != g_calib_done)
    {
        g_calib_done    = request;
        g_ctx.white_raw = g_calib_white;
        g_ctx.black_raw = g_calib_black;
    }
}


/*!
 * @brief Map a raw reading onto 0 (white) .. 1000 (black).
 *
 * @param[in] raw  Raw ADC reading.
 *
 * @return Normalised darkness, clamped to 0..1000.
 *
 * @par
 * Readings outside the calibrated range widen it, absorbing slow lighting
 * drift.
 */
static int32_t
normalise (int32_t raw)
{
    int32_t span = 0;

    if (g_ctx.black_raw > g_ctx.white_raw)
    {
        g_ctx.black_raw = (raw > g_ctx.black_raw) ? raw : g_ctx.black_raw;
        g_ctx.white_raw = (raw < g_ctx.white_raw) ? raw : g_ctx.white_raw;
    }
    else
    {
        g_ctx.black_raw = (raw < g_ctx.black_raw) ? raw : g_ctx.black_raw;
        g_ctx.white_raw = (raw > g_ctx.white_raw) ? raw : g_ctx.white_raw;
    }

    span = g_ctx.black_raw - g_ctx.white_raw;

    if (0 == span)
    {
        span = 1;
    }

    return (fix_math_clamp(((raw - g_ctx.white_raw) * NORM_FULL) / span, 0,
                           NORM_FULL));
}


/*!
 * @brief Run the Week 6 Section 6.3 statechart for one sample.
 *
 * @param[in] norm          Normalised centre darkness.
 * @param[in] b_left_black  Left sensor sees black.
 */
static void
update_events (int32_t norm, bool b_left_black)
{
    bool const b_both_black = b_left_black && (norm > NORM_BLACK);
    bool const b_both_white = (!b_left_black) && (norm < NORM_WHITE);

    // Vehicle Control has resumed following.  Start again from TRACKING with
    // fresh timers, so time spent off the line during a manoeuvre does not
    // count as line loss, and a real loss is always reported again.
    if (g_clear_seq != g_clear_done)
    {
        g_clear_done      = g_clear_seq;
        g_ctx.state       = LINE_STATE_TRACKING;
        g_ctx.junction_ms = 0u;
        g_ctx.lost_ms     = 0u;
    }

    // Vehicle Control is heading for a line: report it when it is seen.
    if (g_search_seq != g_search_done)
    {
        g_search_done     = g_search_seq;
        g_ctx.state       = LINE_STATE_SEARCHING;
        g_ctx.junction_ms = 0u;
        g_ctx.lost_ms     = 0u;
        g_ctx.found_ms    = 0u;
    }

    g_ctx.junction_ms = b_both_black ? (g_ctx.junction_ms + MS_PER_PERIOD) :
                                       0u;
    g_ctx.lost_ms     = b_both_white ? (g_ctx.lost_ms + MS_PER_PERIOD) : 0u;
    g_ctx.found_ms    = (norm > NORM_FOUND) ?
                        (g_ctx.found_ms + MS_PER_PERIOD) : 0u;

    switch (g_ctx.state)
    {
        case LINE_STATE_TRACKING:
            if (g_ctx.junction_ms >= LINE_JUNCTION_MS)
            {
                g_ctx.state = LINE_STATE_JUNCTION;
                robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE,
                                   ROBOT_EVT_JUNCTION);
            }
            else if (g_ctx.lost_ms > LINE_LOST_MS)
            {
                g_ctx.state = LINE_STATE_SEARCHING;
                robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE,
                                   ROBOT_EVT_LINE_LOST);
                telemetry_post_event(TELEMETRY_EVT_LINE, 1u);
            }
            else
            {
                // Normal tracking.
            }
        break;

        case LINE_STATE_JUNCTION:
            // Held until Vehicle Control has executed the junction action.
        break;

        case LINE_STATE_SEARCHING:
            if (g_ctx.found_ms >= LINE_FOUND_MS)
            {
                g_ctx.state = LINE_STATE_TRACKING;
                robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE,
                                   ROBOT_EVT_LINE_FOUND);
                telemetry_post_event(TELEMETRY_EVT_LINE, 0u);
            }
        break;

        default:
            g_ctx.state = LINE_STATE_TRACKING;
        break;
    }
}


/*!
 * @brief Build the line_state word and publish it with one store.
 *
 * @param[in] offset        Signed offset.
 * @param[in] raw           Raw centre reading.
 * @param[in] b_left_black  Left sensor sees black.
 */
static void
publish (int32_t offset, uint32_t raw, bool b_left_black)
{
    uint32_t const stamp = (board_hw_time_us() / US_PER_STAMP_TICK) &
                           STAMP_MASK;
    uint32_t       word  = 0u;

    // The state enumeration is below 8, so the cast is exact.
    word = bitpack_word_set(word, LINE_OFFSET_SHIFT, LINE_OFFSET_WIDTH,
                            bitpack_from_signed(offset, LINE_OFFSET_WIDTH));
    word = bitpack_word_set(word, LINE_CENTRE_SHIFT, LINE_CENTRE_WIDTH, raw);
    word = bitpack_word_set(word, LINE_LEFT_SHIFT, 1u,
                            b_left_black ? 1u : 0u);
    word = bitpack_word_set(word, LINE_STATE_SHIFT, LINE_STATE_WIDTH,
                            (uint32_t) g_ctx.state);  // Enum < 8.
    word = bitpack_word_set(word, LINE_STAMP_SHIFT, LINE_STAMP_WIDTH, stamp);
    sys_status_write(SYS_STATUS_LINE, word);
}

/*** end of file ***/

