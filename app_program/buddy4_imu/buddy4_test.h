/** @file buddy4_test.h
 *
 * @brief Buddy 4 stand-alone bench test: IMU (GY-511 / LSM303) diagnostic.
 *
 * @par
 * Built only with BUDDY_TEST=4.  Only this task runs (the IMU Task does
 * not, so nothing else uses the bus) and the motors stay off.  Every few
 * seconds it scans I2C0, reads the identity registers and counts failed
 * accelerometer reads, at 100 kHz and at 400 kHz, on the USB console.
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

