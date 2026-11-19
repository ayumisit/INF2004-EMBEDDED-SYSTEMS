/** @file buzzer_hw.c
 *
 * @brief Robo Pico piezo buzzer (GP22) driven by PWM.
 *
 * @par
 * The slice counts at 1 MHz (125 MHz / 125); a tone of f Hz wraps every
 * 1e6 / f counts with a 50% duty cycle.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "robot_cfg.h"
#include "buzzer_hw.h"

#define CLOCK_DIVIDER           125u
#define COUNT_HZ                1000000u


/*!
 * @brief Route GP22 to its PWM slice, silent.
 */
void
buzzer_hw_init (void)
{
    uint32_t const slice = pwm_gpio_to_slice_num(PIN_BUZZER);

    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    pwm_set_clkdiv_int_frac(slice, CLOCK_DIVIDER, 0u);
    pwm_set_gpio_level(PIN_BUZZER, 0u);
    pwm_set_enabled(slice, true);
}


/*!
 * @brief Play a tone, or stop.
 *
 * @param[in] freq_hz  Tone in Hz; 0 silences the buzzer.
 */
void
buzzer_hw_tone (uint32_t freq_hz)
{
    if (0u == freq_hz)
    {
        pwm_set_gpio_level(PIN_BUZZER, 0u);
    }
    else
    {
        uint32_t const freq  = (freq_hz < BUZZER_HW_MIN_HZ) ?
                               BUZZER_HW_MIN_HZ :
                               ((freq_hz > BUZZER_HW_MAX_HZ) ?
                                BUZZER_HW_MAX_HZ : freq_hz);
        uint32_t const wrap  = (COUNT_HZ / freq) - 1u;  // 124..9999.

        // Casts: wrap is below 10000, so both values fit in 16 bits.
        pwm_set_wrap(pwm_gpio_to_slice_num(PIN_BUZZER), (uint16_t) wrap);
        pwm_set_gpio_level(PIN_BUZZER, (uint16_t) (wrap / 2u));
    }
}

/*** end of file ***/

