/** @file ir_hw.c
 *
 * @brief TCRT5000 IR sensor inputs on the RP2040 ADC and GPIO.
 *
 * @par
 * Reference: RP2040 Datasheet section 4.9 (ADC); Pico SDK hardware_adc.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "robot_cfg.h"
#include "ir_hw.h"

static bool level_means_black(bool b_level);


/*!
 * @brief Configure the ADC input and the two digital IR inputs.
 */
void
ir_hw_init (void)
{
    adc_init();
    adc_gpio_init(PIN_IR_CENTRE_AO);
    adc_select_input(ADC_INPUT_IR_CENTRE);

    // The modules drive DO actively, so no pad pull is needed.
    gpio_init(PIN_IR_LEFT_DO);
    gpio_set_dir(PIN_IR_LEFT_DO, GPIO_IN);
    gpio_disable_pulls(PIN_IR_LEFT_DO);
    gpio_init(PIN_IR_BARCODE_DO);
    gpio_set_dir(PIN_IR_BARCODE_DO, GPIO_IN);
    gpio_disable_pulls(PIN_IR_BARCODE_DO);
}


/*!
 * @brief One conversion of the centre sensor.
 *
 * @return Raw 12-bit reading, 0 to 4095.
 */
uint32_t
ir_hw_read_centre (void)
{
    adc_select_input(ADC_INPUT_IR_CENTRE);

    return (adc_read());
}


/*!
 * @brief Report whether the left sensor is over black.
 *
 * @return true over the line.
 */
bool
ir_hw_is_left_black (void)
{
    return (level_means_black(gpio_get(PIN_IR_LEFT_DO)));
}


/*!
 * @brief Report whether the barcode sensor is over black.
 *
 * @return true over a bar.
 */
bool
ir_hw_is_barcode_black (void)
{
    return (level_means_black(gpio_get(PIN_IR_BARCODE_DO)));
}


/*!
 * @brief Translate a DO level into "black" using the configured polarity.
 *
 * @param[in] b_level  Pin level.
 *
 * @return true if the level means black.
 */
static bool
level_means_black (bool b_level)
{
    return (b_level == (0u != IR_DO_BLACK_IS_HIGH));
}

/*** end of file ***/

