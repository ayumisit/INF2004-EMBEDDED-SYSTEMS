/** @file lsm303_hw.c
 *
 * @brief LSM303DLHC / LSM303D accelerometer and magnetometer driver on Pico
 *        SDK I2C.
 *
 * @par
 * Reference: ST LSM303DLHC datasheet (DocID018771), register maps in
 * sections 7.1 (accelerometer, address 0x19) and 7.2 (magnetometer,
 * address 0x1E).  Accelerometer: 100 Hz, +/-2 g, high-resolution mode,
 * 1 mg per LSB after the 4-bit left justification is removed.
 * Magnetometer: 30 Hz, +/-1.3 gauss, continuous conversion.
 *
 * @par
 * Some GY-511 boards carry an LSM303D instead (ST DocID023312): one device
 * at 0x1D or 0x1E (SA0 low or high) for both sensors, WHO_AM_I 0x49,
 * little-endian outputs.  Accelerometer: 100 Hz, +/-2 g, 0.061 mg per LSB.
 * Magnetometer: 25 Hz, high resolution, +/-4 gauss, continuous.  The chip
 * is detected in lsm303_hw_init().  Both chips give acceleration in mg;
 * the magnetic field stays raw, which is enough for the atan2 heading.
 *
 * @par
 * Every transfer has a timeout so a disconnected or stuck sensor can never
 * hang the IMU Task; the task counts failures and re-initialises.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "robot_cfg.h"
#include "lsm303_hw.h"

#define ACCEL_ADDR              0x19u
#define MAG_ADDR                0x1Eu
#define REG_CTRL1_A             0x20u
#define REG_CTRL4_A             0x23u
#define REG_OUT_X_L_A           0x28u
#define REG_AUTO_INCREMENT      0x80u
#define REG_CRA_M               0x00u
#define REG_CRB_M               0x01u
#define REG_MR_M                0x02u
#define REG_OUT_X_H_M           0x03u
#define REG_IRA_M               0x0Au
#define CTRL1_A_100HZ_XYZ       0x57u
#define CTRL4_A_HR_2G           0x08u
#define CRA_M_30HZ              0x14u
#define CRB_M_1G3               0x20u
#define MR_M_CONTINUOUS         0x00u
#define IRA_M_IDENTITY          0x48u
#define TIMEOUT_US              2000u
#define DATA_BYTES              6u
#define REG_WRITE_BYTES         2u
#define RECOVERY_CLOCKS         9u
#define RECOVERY_HALF_PERIOD_US 5u
#define SIGN_BIT_16             0x8000u
#define RANGE_16                65536
#define JUSTIFY_DIVISOR         16

// LSM303D (DocID023312 section 8).  Its address depends on the SA0 pin:
// 0x1D when low (the GY-511 on this robot), 0x1E when high.
#define D_ADDR_SA0_LOW          0x1Du
#define D_ADDR_SA0_HIGH         0x1Eu
#define D_REG_WHO_AM_I          0x0Fu
#define D_REG_OUT_X_L_M         0x08u
#define D_REG_CTRL1             0x20u
#define D_REG_CTRL2             0x21u
#define D_REG_CTRL5             0x24u
#define D_REG_CTRL6             0x25u
#define D_REG_CTRL7             0x26u
#define D_REG_OUT_X_L_A         0x28u
#define D_IDENTITY              0x49u
// Reboot (reload the factory trimming) before every configuration.  After
// a battery-only power-on the accelerometer can answer and keep its
// settings while its readings never change, and re-writing the settings
// does not help.  BOOT reloads the chip's internal memory, the nearest
// thing to a power cycle that software can do (DocID023312 CTRL0; DLHC:
// DocID018771 CTRL_REG5_A).
#define D_REG_CTRL0             0x1Fu
#define D_CTRL0_BOOT            0x80u
#define REG_CTRL5_A             0x24u
#define CTRL5_A_BOOT            0x80u
#define BOOT_WAIT_US            20000u  // Boot takes about 5 ms.
#define D_CTRL1_100HZ_XYZ       0x67u   // AODR 100 Hz, X/Y/Z enabled.
#define D_CTRL2_2G              0x00u   // +/-2 g, 773 Hz anti-alias filter.
#define D_CTRL5_HR_25HZ         0x6Cu   // Magnetic high resolution, 25 Hz.
#define D_CTRL6_4GAUSS          0x20u   // +/-4 gauss.
#define D_CTRL7_CONTINUOUS      0x00u   // Magnetic continuous conversion.
#define D_UG_PER_LSB            61      // 0.061 mg per LSB at +/-2 g.
#define UG_PER_MG               1000

/*!
 * @brief Which chip lsm303_hw_init() found.
 */
typedef enum
{
    CHIP_NONE = 0,
    CHIP_DLHC,
    CHIP_D
} chip_t;

static chip_t  g_chip   = CHIP_NONE;
static uint8_t g_d_addr = D_ADDR_SA0_LOW;  // LSM303D address found.

static bool    init_dlhc(void);
static bool    init_d(uint8_t address);
static bool    write_register(uint8_t address, uint8_t reg, uint8_t value);
static bool    read_registers(uint8_t address, uint8_t first_reg,
                              uint8_t * p_data, uint32_t count);
static int32_t to_signed(uint32_t high, uint32_t low);
static void    recover_bus(void);
static void    reboot_chip(uint8_t address, uint8_t reg, uint8_t boot_bit);
static void    wait_us(uint32_t delay_us);


/*!
 * @brief Recover the bus, start I2C0, detect the chip and configure it.
 *
 * @return true if an LSM303DLHC or LSM303D identified itself and accepted
 *         its configuration.
 */
bool
lsm303_hw_init (void)
{
    recover_bus();
    (void) i2c_init(i2c0, IMU_I2C_HZ);  // The actual rate is not needed.
    gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_pull_up(PIN_IMU_SCL);

    if (init_dlhc())
    {
        g_chip = CHIP_DLHC;
    }
    else if ((init_d(D_ADDR_SA0_LOW)) || (init_d(D_ADDR_SA0_HIGH)))
    {
        g_chip = CHIP_D;
    }
    else
    {
        g_chip = CHIP_NONE;
    }

    return (CHIP_NONE != g_chip);
}


/*!
 * @brief Identify and configure an LSM303DLHC.
 *
 * @return true if the magnetometer identified itself and both sensors
 *         accepted their configuration.
 */
static bool
init_dlhc (void)
{
    uint8_t identity = 0u;
    bool    b_found  = false;

    b_found = (read_registers(MAG_ADDR, REG_IRA_M, &identity, 1u)) &&
              (IRA_M_IDENTITY == identity);

    if (b_found)
    {
        reboot_chip(ACCEL_ADDR, REG_CTRL5_A, CTRL5_A_BOOT);
    }

    return (b_found &&
            (write_register(ACCEL_ADDR, REG_CTRL1_A, CTRL1_A_100HZ_XYZ)) &&
            (write_register(ACCEL_ADDR, REG_CTRL4_A, CTRL4_A_HR_2G)) &&
            (write_register(MAG_ADDR, REG_CRA_M, CRA_M_30HZ)) &&
            (write_register(MAG_ADDR, REG_CRB_M, CRB_M_1G3)) &&
            (write_register(MAG_ADDR, REG_MR_M, MR_M_CONTINUOUS)));
}


/*!
 * @brief Identify and configure an LSM303D at one address.
 *
 * @param[in] address  0x1D or 0x1E (SA0 low or high).
 *
 * @return true if WHO_AM_I matched and every control register was written;
 *         the address is then kept for reading.
 */
static bool
init_d (uint8_t address)
{
    uint8_t identity = 0u;
    uint8_t ctrl1    = 0u;
    bool    b_found  = false;
    bool    b_ok     = false;
    bool    b_set    = false;

    b_found = (read_registers(address, D_REG_WHO_AM_I, &identity, 1u)) &&
              (D_IDENTITY == identity);

    if (b_found)
    {
        reboot_chip(address, D_REG_CTRL0, D_CTRL0_BOOT);
    }

    b_ok = (b_found &&
            (write_register(address, D_REG_CTRL1, D_CTRL1_100HZ_XYZ)) &&
            (write_register(address, D_REG_CTRL2, D_CTRL2_2G)) &&
            (write_register(address, D_REG_CTRL5, D_CTRL5_HR_25HZ)) &&
            (write_register(address, D_REG_CTRL6, D_CTRL6_4GAUSS)) &&
            (write_register(address, D_REG_CTRL7, D_CTRL7_CONTINUOUS)));

    // Read the accelerometer setting back.  A write made while the chip is
    // still starting up is acknowledged but lost, which leaves the
    // accelerometer switched off and its readings frozen.
    b_set = b_ok &&
            (read_registers(address, D_REG_CTRL1, &ctrl1, 1u)) &&
            (D_CTRL1_100HZ_XYZ == ctrl1);

    if (b_set)
    {
        g_d_addr = address;
    }

    return (b_set);
}


/*!
 * @brief Read the three acceleration axes.
 *
 * @param[out] p_accel_mg  Array of LSM303_HW_AXES values in milli-g.
 *
 * @return true on success; the outputs are unchanged on failure.
 */
bool
lsm303_hw_read_accel (int32_t * p_accel_mg)
{
    uint8_t    data[DATA_BYTES];
    bool const b_d  = (CHIP_D == g_chip);
    bool const b_ok = read_registers(b_d ? g_d_addr : ACCEL_ADDR,
                                     (b_d ? D_REG_OUT_X_L_A : REG_OUT_X_L_A) |
                                     REG_AUTO_INCREMENT, data, DATA_BYTES);

    if (b_ok)
    {
        // Little-endian X, Y, Z on both chips.  DLHC: 12-bit values
        // left-justified in 16 bits.  D: 16 bits at 0.061 mg per LSB, so
        // |raw| * 61 stays below 2^21.
        for (uint32_t axis = 0u; axis < LSM303_HW_AXES; axis++)
        {
            int32_t const raw = to_signed(data[(2u * axis) + 1u],
                                          data[2u * axis]);

            p_accel_mg[axis] = b_d ? ((raw * D_UG_PER_LSB) / UG_PER_MG) :
                                     (raw / JUSTIFY_DIVISOR);
        }
    }

    return (b_ok);
}


/*!
 * @brief Read the three magnetic-field axes.
 *
 * @param[out] p_field  Array of LSM303_HW_AXES raw values (X, Y, Z).
 *
 * @return true on success; the outputs are unchanged on failure.
 */
bool
lsm303_hw_read_mag (int32_t * p_field)
{
    uint8_t    data[DATA_BYTES];
    bool const b_d  = (CHIP_D == g_chip);
    bool const b_ok = b_d ?
                      read_registers(g_d_addr,
                                     D_REG_OUT_X_L_M | REG_AUTO_INCREMENT,
                                     data, DATA_BYTES) :
                      read_registers(MAG_ADDR, REG_OUT_X_H_M, data,
                                     DATA_BYTES);

    if (b_ok && b_d)
    {
        // LSM303D: little-endian X, Y, Z.
        p_field[LSM303_HW_AXIS_X] = to_signed(data[1], data[0]);
        p_field[LSM303_HW_AXIS_Y] = to_signed(data[3], data[2]);
        p_field[LSM303_HW_AXIS_Z] = to_signed(data[5], data[4]);
    }
    else if (b_ok)
    {
        // LSM303DLHC: big-endian, and the register order is X, Z, Y.
        p_field[LSM303_HW_AXIS_X] = to_signed(data[0], data[1]);
        p_field[LSM303_HW_AXIS_Z] = to_signed(data[2], data[3]);
        p_field[LSM303_HW_AXIS_Y] = to_signed(data[4], data[5]);
    }
    else
    {
        // Read failed: outputs unchanged.
    }

    return (b_ok);
}


/*!
 * @brief Write one register.
 *
 * @param[in] address  7-bit device address.
 * @param[in] reg      Register number.
 * @param[in] value    Value to write.
 *
 * @return true if every byte was acknowledged.
 */
static bool
write_register (uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t const frame[REG_WRITE_BYTES] = { reg, value };
    int32_t const written = i2c_write_timeout_us(i2c0, address, frame,
                                                 REG_WRITE_BYTES, false,
                                                 TIMEOUT_US);

    return (REG_WRITE_BYTES == written);
}


/*!
 * @brief Read consecutive registers.
 *
 * @param[in]  address    7-bit device address.
 * @param[in]  first_reg  First register (with the auto-increment bit if the
 *                        device needs it).
 * @param[out] p_data     Destination.
 * @param[in]  count      Number of bytes.
 *
 * @return true if the whole transfer completed.
 */
static bool
read_registers (uint8_t address, uint8_t first_reg, uint8_t * p_data,
                uint32_t count)
{
    bool b_ok = false;

    if (1 == i2c_write_timeout_us(i2c0, address, &first_reg, 1u, true,
                                  TIMEOUT_US))
    {
        // count is at most DATA_BYTES, so the conversion is exact.
        b_ok = ((int32_t) count == i2c_read_timeout_us(i2c0, address, p_data,
                                                       count, false,
                                                       TIMEOUT_US));
    }

    return (b_ok);
}


/*!
 * @brief Combine two bytes into a signed 16-bit value.
 *
 * @param[in] high  Most significant byte.
 * @param[in] low   Least significant byte.
 *
 * @return The two's complement value, -32768 to 32767.
 */
static int32_t
to_signed (uint32_t high, uint32_t low)
{
    uint32_t const raw   = (high << 8) | low;

    // raw is below 2^16, so the conversion to int32_t is exact.
    int32_t        value = (int32_t) raw;

    if (0u != (raw & SIGN_BIT_16))
    {
        value -= RANGE_16;
    }

    return (value);
}


/*!
 * @brief Clock out a stuck slave and leave the bus idle.
 *
 * @par
 * At boot the bus pins may have carried other signals, which can leave the
 * sensor part-way through a byte holding SDA low.  Nine SCL pulses let it
 * finish (NXP UM10204 I2C specification, section 3.1.16).
 */
static void
recover_bus (void)
{
    gpio_init(PIN_IMU_SDA);
    gpio_set_dir(PIN_IMU_SDA, GPIO_IN);
    gpio_pull_up(PIN_IMU_SDA);
    gpio_init(PIN_IMU_SCL);
    gpio_put(PIN_IMU_SCL, true);
    gpio_set_dir(PIN_IMU_SCL, GPIO_OUT);

    for (uint32_t pulse = 0u; pulse < RECOVERY_CLOCKS; pulse++)
    {
        gpio_put(PIN_IMU_SCL, false);
        wait_us(RECOVERY_HALF_PERIOD_US);
        gpio_put(PIN_IMU_SCL, true);
        wait_us(RECOVERY_HALF_PERIOD_US);
    }
}


/*!
 * @brief Reboot a sensor (reload its factory trimming) and wait for it.
 *
 * @param[in] address   7-bit device address.
 * @param[in] reg       Register holding the BOOT bit.
 * @param[in] boot_bit  Value that sets the BOOT bit.
 *
 * @par
 * See BOOT_WAIT_US.  A failed write is not an error here: the
 * configuration that follows fails too if the bus is down.  The wait
 * runs in the IMU Task, below the motion and line tasks in priority.
 */
static void
reboot_chip (uint8_t address, uint8_t reg, uint8_t boot_bit)
{
    // The (void) discards the result: a failed reboot is not retried.  A
    // dead bus also fails the configuration writes that follow.
    (void) write_register(address, reg, boot_bit);
    wait_us(BOOT_WAIT_US);
}


/*!
 * @brief Busy-wait for a short delay.
 *
 * @param[in] delay_us  Microseconds, at most BOOT_WAIT_US.
 */
static void
wait_us (uint32_t delay_us)
{
    uint32_t const start_us = time_us_32();

    while ((time_us_32() - start_us) < delay_us)
    {
        // Spin: at most BOOT_WAIT_US (20 ms) in the IMU Task.
    }
}

/*** end of file ***/

