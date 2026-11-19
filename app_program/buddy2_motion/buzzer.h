/** @file buzzer.h
 *
 * @brief Buzzer Task: short sound cues for detected events.
 *
 * @par
 * buzzer_play() only records a request, so any task may call it without
 * waiting; the low-priority Buzzer Task plays the sounds.  Each cue is
 * distinct so the team can tell events apart by ear:
 *   BARCODE  one short high beep  (2.5 kHz, 100 ms)
 *   HUMP     one longer low beep  (1 kHz, 300 ms), once per hump, when its
 *            peak height has been calculated
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/*!
 * @brief Sound cues.
 */
typedef enum
{
    BUZZER_CUE_BARCODE = 0,
    BUZZER_CUE_HUMP,
    BUZZER_CUE_COUNT
} buzzer_cue_t;

int32_t buzzer_start(void);
void    buzzer_play(buzzer_cue_t cue);

#endif /* BUZZER_H */

/*** end of file ***/

