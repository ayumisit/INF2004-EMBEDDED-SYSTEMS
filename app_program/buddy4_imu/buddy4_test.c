/** @file buddy4_test.c
 *
 * @brief Buddy 4 stand-alone bench test: IMU (GY-511 / LSM303) diagnostic.
 *
 * @par
 * Built only with BUDDY_TEST=4 (see app_main.c).  Each round, at 100 kHz
 * and then 400 kHz: print the idle SDA/SCL levels, scan the bus, read the
 * identity registers that tell the LSM303 variants apart, then run the
 * real driver's lsm303_hw_init() and READ_COUNT accelerometer reads.
 *
 * @par
 * Expected for a working LSM303DLHC: lines high, devices at 0x19 and 0x1E,
 * magnetometer identity "H43", init OK, 0 failed reads and about 1000 mg.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "robot_cfg.h"
#include "robot_ipc.h"
#include "fix_math.h"
#include "lsm303_hw.h"
#include "imu_diag_hw.h"
#include "buddy4_test.h"

// Time to open the serial monitor after the USB port appears.
#define BOOT_DELAY_MS           4000u
#define ROUND_PAUSE_MS          3000u

#define SLOW_BAUD_HZ            100000u
#define FAST_BAUD_HZ            400000u

// 7-bit addresses outside the reserved ranges.
#define FIRST_ADDRESS           0x08u
#define LAST_ADDRESS            0x77u

#define ACCEL_ADDR              0x19u
#define MAG_ADDR                0x1Eu
#define REG_IRA_M               0x0Au   // DLHC magnetometer: 'H' '4' '3'.
#define IDENTITY_BYTES          3u
#define REG_WHO_AM_I_A          0x0Fu   // AGR accelerometer: 0x33.
#define REG_WHO_AM_I_M          0x4Fu   // AGR magnetometer: 0x40.
#define REG_WHO_AM_I            0x0Fu   // At 0x1E: LSM303D 0x49, LIS3MDL 0x3D.

// Driver read test: READ_COUNT reads READ_STEP_MS apart.
#define READ_COUNT              100u
#define READ_STEP_MS            10u

static uint64_t g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static void buddy4_test_task(INT start_code, void * p_ext_info);
static void run_round(uint32_t baud_hz);
static void print_scan(void);
static void print_identity(void);
static void print_driver_test(void);
static void print_reg(char const * p_name, uint8_t address, uint8_t reg);


/*!
 * @brief Create the bench-test task.
 *
 * @return The task handle (> 0), or a negative kernel error code.
 */
int32_t
buddy4_test_start (void)
{
    return (robot_ipc_start_task(buddy4_test_task, PRIO_TELEMETRY, g_stack,
                                 sizeof(g_stack)));
}


/*!
 * @brief Bench-test task: one diagnostic round at each bus rate, forever.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
buddy4_test_task (INT start_code, void * p_ext_info)
{
    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    robot_ipc_sleep_ms(BOOT_DELAY_MS);

    // Every (UB *) cast below only changes the signedness of a string
    // literal's characters, as the monitor API expects; (void) discards the
    // printed character count.
    (void) tm_printf((UB *) "\n=== AAI03 Buddy 4 IMU diagnostic ===\n");
    (void) tm_printf((UB *) "Motors are OFF.  SDA GP%lu, SCL GP%lu.\n",
                     PIN_IMU_SDA, PIN_IMU_SCL);
    (void) tm_printf((UB *) "Good LSM303DLHC: lines high, 0x19 + 0x1E, "
                     "ID H43, init OK, 0 failed reads, ~1000 mg.\n");

    for (;;)
    {
        run_round(SLOW_BAUD_HZ);
        run_round(FAST_BAUD_HZ);
        (void) tm_printf((UB *) "\n");  // (UB *): monitor API type.
        robot_ipc_sleep_ms(ROUND_PAUSE_MS);
    }
}


/*!
 * @brief One diagnostic round at one bus rate.
 *
 * @param[in] baud_hz  Bus clock in Hz.
 */
static void
run_round (uint32_t baud_hz)
{
    uint32_t const levels = imu_diag_hw_line_levels();

    imu_diag_hw_setup(baud_hz);

    // (UB *): string literals to the monitor API's character type.
    (void) tm_printf((UB *) "--- %lu kHz  SDA %s  SCL %s\n",
                     baud_hz / 1000u,
                     (0u != (levels & IMU_DIAG_HW_SDA_HIGH)) ?
                     "high" : "LOW (stuck?)",
                     (0u != (levels & IMU_DIAG_HW_SCL_HIGH)) ?
                     "high" : "LOW (stuck?)");
    print_scan();
    print_identity();
    print_driver_test();
}


/*!
 * @brief Print every address that acknowledges.
 */
static void
print_scan (void)
{
    uint32_t found = 0u;

    (void) tm_printf((UB *) "scan:");  // (UB *): monitor API type.

    for (uint32_t address = FIRST_ADDRESS; address <= LAST_ADDRESS;
         address++)
    {
        // Cast: the address is at most 0x77, so it fits in a byte.
        if (imu_diag_hw_probe((uint8_t) address))
        {
            (void) tm_printf((UB *) " 0x%02lx", address);  // (UB *): API.
            found++;
        }
    }

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "%s\n", (0u == found) ?
                     " NOTHING answered (power, GND or SDA/SCL wiring)" : "");
}


/*!
 * @brief Print the identity registers that tell the LSM303 variants apart.
 */
static void
print_identity (void)
{
    uint8_t ident[IDENTITY_BYTES] = { 0u, 0u, 0u };
    bool    b_ok               = true;

    for (uint32_t idx = 0u; b_ok && (idx < IDENTITY_BYTES); idx++)
    {
        // Cast: the register number is at most 0x0C, so it fits in a byte.
        b_ok = imu_diag_hw_read_reg(MAG_ADDR, (uint8_t) (REG_IRA_M + idx),
                                    &ident[idx]);
    }

    if (b_ok)
    {
        uint32_t const first  = ident[0];  // Widened for the conversions.
        uint32_t const second = ident[1];
        uint32_t const third  = ident[2];

        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "mag 0x1E ID regs: 0x%02lx 0x%02lx 0x%02lx "
                         "('%c%c%c')  %s\n", first, second, third,
                         first, second, third,
                         ((0x48u == first) && (0x34u == second) &&
                          (0x33u == third)) ?
                         "= LSM303DLHC, supported" : "NOT an LSM303DLHC");
    }
    else
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "mag 0x1E ID regs: read FAILED\n");
    }

    print_reg("accel 0x19 WHO_AM_I_A (AGR = 0x33)", ACCEL_ADDR,
              REG_WHO_AM_I_A);
    print_reg("mag 0x1E WHO_AM_I_M (AGR = 0x40)", MAG_ADDR, REG_WHO_AM_I_M);
    print_reg("dev 0x1E WHO_AM_I 0x0F (LSM303D = 0x49, LIS3MDL = 0x3D, "
              "LSM9DS1 mag = 0x3D)", MAG_ADDR, REG_WHO_AM_I);
}


/*!
 * @brief Run the real driver: initialise, then count failed reads.
 */
static void
print_driver_test (void)
{
    bool const b_init   = lsm303_hw_init();  // Uses IMU_I2C_HZ.
    uint32_t   failures = 0u;
    int32_t    accel[LSM303_HW_AXES] = { 0, 0, 0 };

    for (uint32_t count = 0u; b_init && (count < READ_COUNT); count++)
    {
        if (!lsm303_hw_read_accel(accel))
        {
            failures++;
        }

        robot_ipc_sleep_ms(READ_STEP_MS);
    }

    if (b_init)
    {
        // Each axis is within +/-2048 mg, so the squares sum below 2^24
        // and the conversion is exact.
        uint32_t const sum_sq = (uint32_t) ((accel[0] * accel[0]) +
                                            (accel[1] * accel[1]) +
                                            (accel[2] * accel[2]));

        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "driver: init OK, %lu/%lu reads failed, "
                         "last x %ld y %ld z %ld mg, |a| %lu mg\n",
                         failures, READ_COUNT, accel[0], accel[1], accel[2],
                         fix_math_isqrt(sum_sq));
    }
    else
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "driver: lsm303_hw_init FAILED (this is "
                         "what raises the IMU fault)\n");
    }
}


/*!
 * @brief Print one register, or that the read failed.
 *
 * @param[in] p_name   Label.
 * @param[in] address  7-bit I2C address.
 * @param[in] reg      Register number.
 */
static void
print_reg (char const * p_name, uint8_t address, uint8_t reg)
{
    uint8_t value = 0u;

    if (imu_diag_hw_read_reg(address, reg, &value))
    {
        // (UB *): string literal to the monitor API's character type.
        // Cast: widen the byte for the conversion.
        (void) tm_printf((UB *) "%s: 0x%02x\n", p_name, (uint32_t) value);
    }
    else
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "%s: no answer\n", p_name);
    }
}

/*** end of file ***/

