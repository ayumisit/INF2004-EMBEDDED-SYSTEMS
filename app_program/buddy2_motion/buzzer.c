/** @file buzzer.c
 *
 * @brief Buzzer Task: short sound cues for detected events.
 *
 * @par
 * Each cue has its own request flag, written only by buzzer_play() and
 * cleared only by this task, so no request word is shared between writers.
 * A request made while the same cue is already pending merges with it.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "robot_cfg.h"
#include "robot_ipc.h"
#include "buzzer_hw.h"
#include "buzzer.h"

#define POLL_MS                 20u
#define MAX_NOTES               2u
#define CUE_COUNT               ((uint32_t) BUZZER_CUE_COUNT)  // Enum: exact.

/*!
 * @brief One note of a cue: tone (0 = silence) and duration.
 */
typedef struct
{
    uint32_t freq_hz;
    uint32_t duration_ms;
} note_t;

/*!
 * @brief A cue: up to MAX_NOTES notes with a gap between them.
 */
typedef struct
{
    note_t   notes[MAX_NOTES];
    uint32_t count;
    uint32_t gap_ms;
} cue_t;

static cue_t const g_cues[BUZZER_CUE_COUNT] =
{
    { { { 2500u, 100u }, { 0u, 0u } }, 1u, 0u },        // BARCODE
    { { { 1000u, 300u }, { 0u, 0u } }, 1u, 0u }         // HUMP
};

static bool volatile g_requested[BUZZER_CUE_COUNT];   // Writer: callers.
static uint64_t      g_stack[STACK_SMALL / sizeof(uint64_t)];

static void buzzer_task(INT start_code, void * p_ext_info);
static void play_cue(cue_t const * p_cue);


/*!
 * @brief Set up the buzzer pin and start the Buzzer Task.
 *
 * @return The task handle (> 0), or a negative kernel error code.
 */
int32_t
buzzer_start (void)
{
    for (uint32_t idx = 0u; idx < CUE_COUNT; idx++)
    {
        g_requested[idx] = false;
    }

    buzzer_hw_init();

    return (robot_ipc_start_task(buzzer_task, PRIO_BUZZER, g_stack,
                                 sizeof(g_stack)));
}


/*!
 * @brief Ask for a cue to be played; returns at once.
 *
 * @param[in] cue  Which sound.
 */
void
buzzer_play (buzzer_cue_t cue)
{
    if (cue < BUZZER_CUE_COUNT)
    {
        g_requested[cue] = true;
    }
}


/*!
 * @brief Buzzer Task: play each requested cue in turn.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
buzzer_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    for (;;)
    {
        for (uint32_t idx = 0u; idx < CUE_COUNT; idx++)
        {
            if (g_requested[idx])
            {
                g_requested[idx] = false;
                play_cue(&g_cues[idx]);
            }
        }

        robot_ipc_sleep_ms(POLL_MS);
    }
}


/*!
 * @brief Play one cue, then leave the buzzer silent.
 *
 * @param[in] p_cue  Cue to play.
 */
static void
play_cue (cue_t const * p_cue)
{
    for (uint32_t idx = 0u; idx < p_cue->count; idx++)
    {
        if (idx > 0u)
        {
            buzzer_hw_tone(0u);
            robot_ipc_sleep_ms(p_cue->gap_ms);
        }

        buzzer_hw_tone(p_cue->notes[idx].freq_hz);
        robot_ipc_sleep_ms(p_cue->notes[idx].duration_ms);
    }

    buzzer_hw_tone(0u);
}

/*** end of file ***/

