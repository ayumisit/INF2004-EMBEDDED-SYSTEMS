/** @file buddy4_test.h
 *
 * @brief Buddy 4 stand-alone bench test: IMU (GY-511 / LSM303) diagnostic.
 *
 * @par
 * Started only with BUDDY_TEST=4 (see app_main.c).  Only this task and the
 * Buzzer Task run (the IMU Task does not, so nothing else uses the bus)
 * and the motors stay off.  Every few seconds it runs one round at
 * 100 kHz and one at 400 kHz.  Each round scans I2C0, reads the identity
 * registers, then counts failed accelerometer reads through the driver,
 * which always runs the bus at IMU_I2C_HZ.  Results are printed on the
 * USB console.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BUDDY4_TEST_H
#define BUDDY4_TEST_H

#include <stdint.h>

int32_t buddy4_test_start(void);

#endif /* BUDDY4_TEST_H */

/*** end of file ***/

