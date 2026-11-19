/** @file imu_diag_hw.h
 *
 * @brief Raw I2C access for the Buddy 4 IMU bench diagnostic (BUDDY_TEST=4).
 *
 * @par
 * Bus scan and single-register reads on I2C0 (GP0 SDA, GP1 SCL), so the
 * diagnostic can show which devices answer and what their identity
 * registers hold, independently of the LSM303 driver.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef IMU_DIAG_HW_H
#define IMU_DIAG_HW_H

#include <stdint.h>
#include <stdbool.h>

void     imu_diag_hw_setup(uint32_t baud_hz);
uint32_t imu_diag_hw_line_levels(void);
bool     imu_diag_hw_probe(uint8_t address);
bool     imu_diag_hw_read_reg(uint8_t address, uint8_t reg, uint8_t * p_value);

// imu_diag_hw_line_levels() bits.
#define IMU_DIAG_HW_SDA_HIGH        0x01u
#define IMU_DIAG_HW_SCL_HIGH        0x02u

#endif /* IMU_DIAG_HW_H */

/*** end of file ***/

