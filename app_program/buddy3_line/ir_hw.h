/** @file ir_hw.h
 *
 * @brief TCRT5000 IR sensor inputs: centre analogue (GP27, ADC1), left
 *        digital (GP3) and barcode digital (GP28).
 *
 * @par
 * Owner: Buddy 3.  The ADC is used only by the Line Following task.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef IR_HW_H
#define IR_HW_H

#include <stdint.h>
#include <stdbool.h>

// Largest raw ADC reading (12-bit converter).
#define IR_HW_ADC_MAX               4095u

void     ir_hw_init(void);
uint32_t ir_hw_read_centre(void);
bool     ir_hw_is_left_black(void);
bool     ir_hw_is_barcode_black(void);

#endif /* IR_HW_H */

/*** end of file ***/

