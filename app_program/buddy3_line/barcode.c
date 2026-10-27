/** @file barcode.c
 *
 * @brief Barcode Task: interrupt edge capture and Code 39 decoding.
 *
 * @par
 * FIFO entries are 16 bits (Change Requirement 1): bit 15 is the sensor
 * level after the edge (1 = black) and bits 14..0 the duration of the
 * element that just ended, in 16 us ticks (saturating at 524 ms).  The ISR
 * writes the head index and the task writes the tail index; the two are
 * separate words, never packed together, so neither side performs a
 * read-modify-write on data the other side writes.
 *
 * @par
 * The quiet zone that ends a capture is adaptive: at least BARCODE_QUIET_MS
 * and at least four times the widest element seen so far.  A Code 39 quiet
 * zone is at least ten narrow elements wide while a wide element is at most
 * three, so this never splits a barcode, at any constant speed.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "gpio_irq.h"
#include "robot_ipc.h"
#include "sys_status.h"
#include "bitpack.h"
#include "telemetry.h"
#include "code39.h"
#include "barcode.h"

#define TICK_US                 16u
#define ENTRY_BLACK_BIT         0x8000u
#define ENTRY_WIDTH_MASK        0x7FFFu
#define MAX_ELEMENTS            32u
#define QUIET_FACTOR            4u
#define US_PER_MS               1000u
#define LETTER_COUNT            26u
#define FIRST_LETTER            0x41u     // ASCII code of the letter A.

/*!
 * @brief One finished capture.
 */
typedef struct
{
    uint32_t    widths[MAX_ELEMENTS];
    uint32_t    count;
    bool        b_started_on_bar;
    bool        b_overflow;
} capture_t;

// Interrupt-to-task FIFO.  Head and overflow count: written by the ISR only.
// Tail: written by the task only.
static uint16_t volatile g_fifo[BARCODE_FIFO_SLOTS];
static uint32_t volatile g_fifo_head      = 0u;
static uint32_t volatile g_fifo_tail      = 0u;
static uint32_t volatile g_overflow_count = 0u;
static uint32_t          g_last_edge_us   = 0u;     // ISR only.

static ID                gh_task          = 0;
static uint32_t          g_status         = 0u;     // Task only.
static uint32_t          g_sequence       = 0u;     // Task only.
static uint32_t          g_reads          = 0u;     // Task only.
static uint32_t          g_rejects        = 0u;     // Task only.
static uint64_t          g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void     barcode_task(INT start_code, void * p_ext_info);
static void     capture(capture_t * p_capture);
static bool     drain(capture_t * p_capture, bool b_first);
static void     report(uint8_t symbol, bool b_reversed);
static void     reject(barcode_reject_t reason, uint32_t count);
static void     set_state(barcode_state_t state);
static barcode_reject_t to_reject(code39_result_t result);
static void     barcode_edge_isr(uint32_t pin, uint32_t edges,
                                 uint32_t time_us, uint32_t levels);


/*!
 * @brief Configure the barcode input and start the Barcode Task.
 *
 * @return 0 on success, or a negative error code.
 *
 * @par
 * The IR pins themselves are configured by line_follow_start() (ir_hw).
 */
int32_t
barcode_start (void)
{
    int32_t result = 0;

    gh_task = robot_ipc_start_task(barcode_task, PRIO_BARCODE, g_stack,
                                   sizeof(g_stack));

    if ((gh_task <= 0) ||
        (!gpio_irq_attach(PIN_IR_BARCODE_DO, barcode_edge_isr, true, true)))
    {
        result = -1;
    }

    return (result);
}


/*!
 * @brief Extract the letter from a navigation message.
 *
 * @param[in] msg  Navigation message byte.
 *
 * @return Letter index, 0 = A .. 25 = Z.
 */
uint32_t
barcode_msg_letter (uint8_t msg)
{
    return (bitpack_word_get(msg, BARCODE_LETTER_SHIFT,
                             BARCODE_LETTER_WIDTH));
}


/*!
 * @brief Barcode Task: IDLE, CAPTURING, DECODING, REPORT or REJECT.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
barcode_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        capture_t cap;

        // IDLE: wait for the first edge.
        set_state(BARCODE_STATE_IDLE);
        (void) tk_slp_tsk(TMO_FEVR);  // No timeout: cannot fail.

        set_state(BARCODE_STATE_CAPTURING);
        capture(&cap);

        set_state(BARCODE_STATE_DECODING);

        if (cap.b_overflow)
        {
            reject(BARCODE_REJECT_OVERFLOW, cap.count);
        }
        else if (!cap.b_started_on_bar)
        {
            reject(BARCODE_REJECT_START, cap.count);
        }
        else
        {
            uint8_t               symbol     = 0u;
            bool                  b_reversed = false;
            code39_result_t const result     = code39_decode(cap.widths,
                                                             cap.count,
                                                             &symbol,
                                                             &b_reversed);

            if (CODE39_OK == result)
            {
                report(symbol, b_reversed);
            }
            else
            {
                reject(to_reject(result), cap.count);
            }
        }

        // Discard wake-ups queued by edges that belonged to this capture.
        (void) tk_can_wup(TSK_SELF);  // The count itself is not needed.
    }
}


/*!
 * @brief Collect elements until a quiet zone passes with no edge.
 *
 * @param[out] p_capture  The finished capture.
 */
static void
capture (capture_t * p_capture)
{
    uint32_t const overflow_before = g_overflow_count;
    bool           b_first         = true;
    bool           b_quiet         = false;

    p_capture->count            = 0u;
    p_capture->b_started_on_bar = false;
    p_capture->b_overflow       = false;

    while (!b_quiet)
    {
        uint32_t widest   = 0u;
        uint32_t quiet_ms = BARCODE_QUIET_MS;

        b_first = drain(p_capture, b_first);

        for (uint32_t idx = 0u; idx < p_capture->count; idx++)
        {
            widest = (p_capture->widths[idx] > widest) ?
                     p_capture->widths[idx] : widest;
        }

        if (((widest * TICK_US * QUIET_FACTOR) / US_PER_MS) > quiet_ms)
        {
            quiet_ms = (widest * TICK_US * QUIET_FACTOR) / US_PER_MS;
        }

        // quiet_ms is at most about 2.1 s, so the conversion is exact.
        b_quiet = (E_TMOUT == tk_slp_tsk((TMO) quiet_ms));
    }

    (void) drain(p_capture, b_first);  // Final drain; result not needed.
    p_capture->b_overflow = (overflow_before != g_overflow_count);
}


/*!
 * @brief Move FIFO entries into the capture.
 *
 * @param[in,out] p_capture  Capture being built.
 * @param[in]     b_first    true if no entry has been taken yet.
 *
 * @return The updated "no entry taken yet" flag.
 *
 * @par
 * The first entry of a capture is the quiet zone before the first bar: it
 * is not stored, but the level after it must be black.
 */
static bool
drain (capture_t * p_capture, bool b_first)
{
    bool     b_still_first = b_first;
    uint32_t tail          = g_fifo_tail;

    while (tail != g_fifo_head)
    {
        uint32_t const entry = g_fifo[tail];

        if (b_still_first)
        {
            p_capture->b_started_on_bar = (0u != (entry & ENTRY_BLACK_BIT));
            b_still_first               = false;
        }
        else if (p_capture->count < MAX_ELEMENTS)
        {
            p_capture->widths[p_capture->count] = entry & ENTRY_WIDTH_MASK;
            p_capture->count++;
        }
        else
        {
            // Too many elements for a single-letter frame.
            p_capture->b_overflow = true;
        }

        tail = (tail + 1u) % BARCODE_FIFO_SLOTS;
    }

    g_fifo_tail = tail;

    return (b_still_first);
}


/*!
 * @brief Send a decoded letter to Vehicle Control and Telemetry.
 *
 * @param[in] symbol      ASCII letter 'A'..'Z'.
 * @param[in] b_reversed  true if read backwards.
 */
static void
report (uint8_t symbol, bool b_reversed)
{
    uint32_t const letter = (uint32_t) symbol - FIRST_LETTER;  // Widening.
    uint32_t       msg    = 0u;
    uint8_t        byte   = 0u;

    set_state(BARCODE_STATE_REPORT);
    g_sequence = (g_sequence + 1u) % (1u << BARCODE_SEQ_WIDTH);
    g_reads++;

    msg  = bitpack_word_set(msg, BARCODE_LETTER_SHIFT, BARCODE_LETTER_WIDTH,
                            letter % LETTER_COUNT);
    msg  = bitpack_word_set(msg, BARCODE_REVERSED_SHIFT, 1u,
                            b_reversed ? 1u : 0u);
    msg  = bitpack_word_set(msg, BARCODE_SEQ_SHIFT, BARCODE_SEQ_WIDTH,
                            g_sequence);
    byte = (uint8_t) msg;  // msg uses only bits 7..0: exact.

    (void) robot_ipc_send(ROBOT_IPC_MBF_NAV_CMD, &byte, sizeof(byte),
                          ROBOT_IPC_POLL);  // Full buffer: newest dropped.
    robot_ipc_flag_set(ROBOT_IPC_FLAG_VEHICLE, ROBOT_EVT_NAV_CMD);
    telemetry_post_event(TELEMETRY_EVT_BARCODE, msg);

    // The low 8 bits of the status word repeat the message.
    g_status = bitpack_word_set(g_status, 0u, BARCODE_MSG_WIDTH, msg);
    g_status = bitpack_word_set(g_status, BARCODE_READS_SHIFT,
                                BARCODE_COUNT_WIDTH,
                                bitpack_saturate(g_reads,
                                                 BARCODE_COUNT_WIDTH));
    sys_status_write(SYS_STATUS_BARCODE, g_status);
}


/*!
 * @brief Record a rejected capture.  Nothing is sent to Vehicle Control.
 *
 * @param[in] reason  Why the capture was rejected.
 * @param[in] count   Number of elements captured.
 */
static void
reject (barcode_reject_t reason, uint32_t count)
{
    uint32_t data = 0u;

    set_state(BARCODE_STATE_REJECT);
    g_rejects++;

    // The enumeration value is below 8, so the cast is exact.
    data = bitpack_word_set(data, 0u, BARCODE_REJECT_REASON_WIDTH,
                            (uint32_t) reason);  // Enum < 8.
    data = bitpack_word_set(data, BARCODE_REJECT_COUNT_SHIFT,
                            BARCODE_REJECT_COUNT_WIDTH,
                            bitpack_saturate(count,
                                             BARCODE_REJECT_COUNT_WIDTH));
    telemetry_post_event(TELEMETRY_EVT_DECODE_FAIL, data);

    if (BARCODE_REJECT_OVERFLOW == reason)
    {
        robot_ipc_raise_fault(ROBOT_FAULT_BARCODE_OVF);
    }

    g_status = bitpack_word_set(g_status, BARCODE_REJECTS_SHIFT,
                                BARCODE_COUNT_WIDTH,
                                bitpack_saturate(g_rejects,
                                                 BARCODE_COUNT_WIDTH));
    sys_status_write(SYS_STATUS_BARCODE, g_status);
}


/*!
 * @brief Record the task state in the status word.
 *
 * @param[in] state  New state.
 */
static void
set_state (barcode_state_t state)
{
    // The enumeration value is below 8, so the cast is exact.
    g_status = bitpack_word_set(g_status, BARCODE_STATE_SHIFT,
                                BARCODE_STATE_WIDTH, (uint32_t) state);
    sys_status_write(SYS_STATUS_BARCODE, g_status);
}


/*!
 * @brief Map a decoder result to a reject reason.
 *
 * @param[in] result  Failed decoder result.
 *
 * @return Reject reason.
 */
static barcode_reject_t
to_reject (code39_result_t result)
{
    barcode_reject_t reason = BARCODE_REJECT_COUNT;

    switch (result)
    {
        case CODE39_BAD_RATIO:
            reason = BARCODE_REJECT_RATIO;
        break;

        case CODE39_NO_MATCH:
            reason = BARCODE_REJECT_NO_MATCH;
        break;

        case CODE39_BAD_FRAME:
            reason = BARCODE_REJECT_FRAME;
        break;

        default:
            reason = BARCODE_REJECT_COUNT;
        break;
    }

    return (reason);
}


/*!
 * @brief GPIO handler for the barcode sensor (interrupt context).
 *
 * @param[in] pin      PIN_IR_BARCODE_DO.
 * @param[in] edges    GPIO_IRQ_EDGE_* bits.
 * @param[in] time_us  Interrupt entry time.
 * @param[in] levels   Pin levels at interrupt entry.
 *
 * @par
 * Stores one FIFO entry and wakes the task.  Two coalesced edges mean an
 * element was lost, which is recorded as an overflow so the capture is
 * rejected rather than decoded wrongly.
 */
static void
barcode_edge_isr (uint32_t pin, uint32_t edges, uint32_t time_us,
                  uint32_t levels)
{
    uint32_t const ticks   = (time_us - g_last_edge_us) / TICK_US;
    uint32_t const level   = (levels >> pin) & 1u;
    bool const     b_black = (IR_DO_BLACK_IS_HIGH == level);
    uint32_t const next    = (g_fifo_head + 1u) % BARCODE_FIFO_SLOTS;
    uint32_t       entry   = (ticks > ENTRY_WIDTH_MASK) ? ENTRY_WIDTH_MASK :
                                                          ticks;

    g_last_edge_us = time_us;

    if (b_black)
    {
        entry |= ENTRY_BLACK_BIT;
    }

    if ((GPIO_IRQ_EDGE_RISING | GPIO_IRQ_EDGE_FALLING) == edges)
    {
        g_overflow_count++;
    }

    if (next == g_fifo_tail)
    {
        g_overflow_count++;
    }
    else
    {
        g_fifo[g_fifo_head] = (uint16_t) entry;  // entry fits in 16 bits.
        g_fifo_head         = next;
    }

    (void) tk_wup_tsk(gh_task);  // E_QOVR is benign: the task drains all.
}

/*** end of file ***/

