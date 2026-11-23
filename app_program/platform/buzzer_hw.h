/** @file buzzer_hw.h
 *
 * @brief Robo Pico piezo buzzer (GP22) driven by PWM.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BUZZER_HW_H
#define BUZZER_HW_H

#include <stdint.h>

// Tones outside this range are clamped (16-bit wrap at a 1 MHz count).
#define BUZZER_HW_MIN_HZ            100u
#define BUZZER_HW_MAX_HZ            8000u

void buzzer_hw_init(void);
void buzzer_hw_tone(uint32_t freq_hz);

#endif /* BUZZER_HW_H */

/*** end of file ***/

