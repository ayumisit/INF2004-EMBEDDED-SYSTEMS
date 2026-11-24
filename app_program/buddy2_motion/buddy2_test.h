/** @file buddy2_test.h
 *
 * @brief Buddy 2 stand-alone bench test: drive straight, report the wheels.
 *
 * @par
 * Built only with BUDDY_TEST=2.  Only Motion Control and the encoders run,
 * with the buzzer and timing services every build starts.  A press of the
 * START button (GP20) drives 50 cm straight ahead (a press during the drive
 * stops it) at a slow speed without the line sensor, printing both wheel
 * speeds and encoder counts, so motor and encoder faults can be told apart
 * from line-sensor faults.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BUDDY2_TEST_H
#define BUDDY2_TEST_H

#include <stdint.h>

int32_t buddy2_test_start(void);

#endif /* BUDDY2_TEST_H */

/*** end of file ***/

