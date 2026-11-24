/** @file buddy3_test.h
 *
 * @brief Buddy 3 stand-alone bench test: IR sensors, line state, barcodes.
 *
 * @par
 * Started only with BUDDY_TEST=3 (see app_main.c); the BUDDY_TEST=23 test
 * also calls buddy3_test_calibrate() and buddy3_test_report_events().  The
 * robot then starts the Buddy 3 tasks, this test task and the Buzzer Task
 * that every build starts: the motors stay off, and the IR readings, line
 * states and decoded barcodes are printed on the USB console.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef BUDDY3_TEST_H
#define BUDDY3_TEST_H

#include <stdint.h>
#include <stdbool.h>

int32_t buddy3_test_start(void);
bool    buddy3_test_calibrate(void);
void    buddy3_test_report_events(void);

#endif /* BUDDY3_TEST_H */

/*** end of file ***/

