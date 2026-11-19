/** @file imu_diag_hw.c
 *
 * @brief Raw I2C access for the Buddy 4 IMU bench diagnostic (BUDDY_TEST=4).
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "robot_cfg.h"
#include "imu_diag_hw.h"

#define TIMEOUT_US              2000u


/*!
 * @brief (Re)start I2C0 on the IMU pins at the given rate.
 *
 * @param[in] baud_hz  Bus clock in Hz.
 */
void
imu_diag_hw_setup (uint32_t baud_hz)
{
    (void) i2c_init(i2c0, baud_hz);  // The actual rate is not needed.
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);
}


/*!
 * @brief Read the idle levels of SDA and SCL.
 *
 * @return IMU_DIAG_HW_SDA_HIGH and IMU_DIAG_HW_SCL_HIGH bits.  An idle bus
 *         reads both high; a line held low is shorted or a device is stuck.
 */
uint32_t
imu_diag_hw_line_levels (void)
{
    uint32_t levels = 0u;

    if (gpio_get(PIN_IMU_SDA))
    {
        levels |= IMU_DIAG_HW_SDA_HIGH;
    }

    if (gpio_get(PIN_IMU_SCL))
    {
        levels |= IMU_DIAG_HW_SCL_HIGH;
    }

    return (levels);
}


/*!
 * @brief Check whether a device acknowledges an address.
 *
 * @param[in] address  7-bit I2C address.
 *
 * @return true if a one-byte read was acknowledged.
 */
bool
imu_diag_hw_probe (uint8_t address)
{
    uint8_t byte = 0u;

    return (1 == i2c_read_timeout_us(i2c0, address, &byte, 1u, false,
                                     TIMEOUT_US));
}


/*!
 * @brief Read one register.
 *
 * @param[in]  address  7-bit I2C address.
 * @param[in]  reg      Register number.
 * @param[out] p_value  Register contents.
 *
 * @return true if the read completed.
 */
bool
imu_diag_hw_read_reg (uint8_t address, uint8_t reg, uint8_t * p_value)
{
    bool b_ok = false;

    if (1 == i2c_write_timeout_us(i2c0, address, &reg, 1u, true, TIMEOUT_US))
    {
        b_ok = (1 == i2c_read_timeout_us(i2c0, address, p_value, 1u, false,
                                         TIMEOUT_US));
    }

    return (b_ok);
}

/*** end of file ***/

