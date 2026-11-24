/** @file robot_cfg.h
 *
 * @brief Board wiring and tuning constants for the AAI03 robot car.
 *
 * @par
 * This header is the single place where pins, mechanical dimensions, task
 * periods and tuning values are defined.  It is shared by every subsystem and
 * deliberately contains constants only, so it can be included from both the
 * kernel-side modules and the Pico SDK hardware modules.
 *
 * @par
 * Wiring source: "current wiring record 2026-10-20" plus the GP26 conflict
 * fix agreed after Tribunal 1 (centre IR AO moved to GP27, ultrasonic moved to
 * the GP4/GP5 Grove port).  Values marked "TBC" must be measured on the
 * finished chassis before they are trusted.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2026 AAI03 team, INF2004.  All rights reserved.
 */

#ifndef ROBOT_CFG_H
#define ROBOT_CFG_H

/* ------------------------------------------------------------------------ */
/* Pin map (Raspberry Pi Pico W on Cytron Robo Pico).                      */
/* ------------------------------------------------------------------------ */

// Buddy 4: LSM303DLHC accelerometer/magnetometer on I2C0 (Grove port 1).
#define PIN_IMU_SDA                 0u
#define PIN_IMU_SCL                 1u

// Buddy 3: IR1 digital output (left line sensor).  IR1 AO on GP2 is not an
// ADC pin and is left unused with its input buffer disabled.
#define PIN_IR_LEFT_DO              3u
#define PIN_IR1_AO_UNUSED           2u

// Buddy 5: HC-SR04 on the GP4/GP5 Grove port.  NOTE: confirm which wire is
// TRIG and which is ECHO; swap these two values if the trace says otherwise.
#define PIN_SONAR_ECHO              4u
#define PIN_SONAR_TRIG              5u

// Buddy 2: right wheel encoder on GP16 (channel B, quadrature only, GP17).
// Hand-tested 2026-11-01: turning the robot's right wheel moves GP16 only.
#define PIN_ENC_RIGHT_A             16u
#define PIN_ENC_RIGHT_B             17u

// Buddy 3: IR3 digital output (dedicated barcode sensor).  IR3 AO on GP7 is
// not an ADC pin and is left unused with its input buffer disabled.
#define PIN_IR_BARCODE_DO           28u
#define PIN_IR3_AO_UNUSED           7u

// Buddy 2: Robo Pico motor driver.  M1 = GP8/GP9, M2 = GP10/GP11.  TBC which
// motor is on the left; swap the pairs below if the robot spins on FORWARD.
#define PIN_MOTOR_LEFT_A            8u
#define PIN_MOTOR_LEFT_B            9u
#define PIN_MOTOR_RIGHT_A           10u
#define PIN_MOTOR_RIGHT_B           11u

// Buddy 5: scanning servo (Robo Pico servo header GP15).
#define PIN_SERVO                   15u

// Buddy 2: left wheel encoder on GP6, its second channel on GP26.  GP6 and
// GP26 are the same wheel: 2026-11-04 telemetry showed the old 26/6 pair
// counting in lockstep through steering.
#define PIN_ENC_LEFT_A              6u
#define PIN_ENC_LEFT_B              26u

// Integration: Robo Pico on-board push button GP20 (active low) = START.
#define PIN_START_BUTTON            20u

// Integration: Robo Pico on-board piezo buzzer (GP22, PWM slice 3, which
// nothing else uses).  The board's mute switch silences it.
#define PIN_BUZZER                  22u

// Buddy 3: centre IR2 analogue output on GP27 (ADC input 1).
#define PIN_IR_CENTRE_AO            27u
#define ADC_INPUT_IR_CENTRE         1u

// Buddy 3: barcode IR module.  Its digital output cannot see a bar narrower
// than the sensor's spot on the paper, so its analogue output is wired to
// GP28 (ADC input 2) instead and the firmware finds the edges itself.  Set
// BARCODE_SENSOR_ANALOGUE to 0u to go back to the digital output on GP28.
//
// 2026-11-11: back to the digital output (DO on GP28, AO parked on GP7).
// The new sensor must sit 1-2 cm up to clear the hump; there the analogue
// signal blurred a barcode into 1-5 elements, so the module's comparator is
// used instead, with its potentiometer tuned on the robot so the LED
// flickers once per bar.  Set back to 1u (and AO to GP28) to undo.
#ifndef BARCODE_SENSOR_ANALOGUE
#define BARCODE_SENSOR_ANALOGUE     0u
#endif
#define PIN_IR_BARCODE_AO           28u
#define ADC_INPUT_IR_BARCODE        2u

/* ------------------------------------------------------------------------ */
/* Sensor polarity.                                                        */
/* ------------------------------------------------------------------------ */

// TCRT5000 modules: black tape reflects little IR, so the comparator output
// (DO) is high and the analogue output (AO) is high over black.  Set to 0u if
// a module on this robot behaves the other way round.
#define IR_DO_BLACK_IS_HIGH         1u
#define IR_AO_BLACK_IS_HIGH         1u

// Encoders: 0u = one channel counted, direction taken from the motor command
// (slotted-disc LM393 module); 1u = quadrature A/B with direction from B.
#define ENC_QUADRATURE              0u

/* ------------------------------------------------------------------------ */
/* Mechanical dimensions (TBC: measure on the finished chassis).           */
/* ------------------------------------------------------------------------ */

// Encoder resolution.  Measured on the robot on 2026-11-02 by pushing it
// 300 mm: 1693 edges on each wheel, so 177 um per edge.  The encoders are
// on the motor shaft behind the gearbox, about 1150 edges per wheel
// revolution, not the 40 of a 20-slot wheel disc.  Re-measure with the
// live tune (enc_um_per_edge) if wheels or motors change.
#define ENC_EDGES_PER_REV           1150u
#define ENC_UM_PER_EDGE             177u

// Distance between the two wheel contact points.
#define TRACK_WIDTH_MM              120u

// How far the robot backs up before retrying (REVERSE_RETRY).
#define REVERSE_DISTANCE_MM         100u

/* ------------------------------------------------------------------------ */
/* Task periods.  The kernel tick is CNF_TIMER_PERIOD = 10 ms.             */
/* ------------------------------------------------------------------------ */

#define PERIOD_BASE_MS              10u
#define PERIOD_ENCODER_TICKS        1u
#define PERIOD_LINE_TICKS           1u
#define PERIOD_MOTION_TICKS         1u
#define PERIOD_IMU_TICKS            2u
#define PERIOD_TELEMETRY_TICKS      20u
#define HEARTBEAT_EVERY_FRAMES      5u

/* ------------------------------------------------------------------------ */
/* Task priorities (1 is the highest).  The port's USB-CDC service runs at */
/* 3 and the CYW43 radio-owner task (Buddy 1 network poll) runs at 4, so   */
/* every control and sensing task sits above them (Week 6, NFR17).         */
/* ------------------------------------------------------------------------ */

#define PRIO_VEHICLE                1
#define PRIO_BARCODE                1
#define PRIO_ENCODER                2
#define PRIO_LINE                   2
#define PRIO_MOTION                 2
#define PRIO_OBSTACLE               2
#define PRIO_IMU                    5
#define PRIO_TELEMETRY              8
#define PRIO_BUZZER                 9

// Stack sizes in bytes.  Single-core build; trimmed after measurement.
#define STACK_SMALL                 1024u
#define STACK_MEDIUM                2048u

/* ------------------------------------------------------------------------ */
/* Buddy 2: motion control tuning (fixed point, TBC on the chassis).       */
/* ------------------------------------------------------------------------ */

#define MOTOR_PWM_HZ                10000u
#define MOTOR_LEFT_REVERSED         1u
#define MOTOR_RIGHT_REVERSED        0u
#define MOTOR_MIN_PERMILLE          250u
// Speed limit: the most PWM either motor is ever given, in permille of
// full power (1000).  Lower it to make the robot slower; keep it above
// MOTOR_MIN_PERMILLE or the wheels may not turn at all.
// 2026-11-17: 450 to 600 with the follow speed raised to 75, so the outer
// wheel in a turn (up to 127 mm/s, about 415 permille before correction)
// is not held back by the limit.
#define MOTOR_MAX_PERMILLE          600u
#define MOTOR_FF_PERMILLE_PER_M_S   1300u
// Slowed 2026-11-04 for smooth tracking (was 130 / 100 / 100), and again
// 2026-11-11 to 45 after live tests (was 70 / 70 / 70): 30 and 40 surged
// near the motor dead band, 45 is the lowest barcode-reading speed.
// 2026-11-17: follow speed 45 to 75.  The steering gains and the off-edge
// ramps below were set for 70, so they suit 75 unchanged; the turn and
// bypass speeds stay at 45.  Not yet checked at 75: barcode reading.
// 2026-11-17 (build 8): 75 to 55.  Every logged run at 75 swung across the
// line edge in curves (line offset SD 350-540, wheel difference SD 43-65
// mm/s); the shake began when the speed was raised from 45 today.  55 is
// between the smooth 45 and the shaky 75.
#define SPEED_FOLLOW_MM_S           55u
// 2026-11-17: slow speed 45 to 75.  It sets the straight legs of a bypass,
// which took 27 s at 45; turns on the spot stay at 45 for accuracy.
#define SPEED_SLOW_MM_S             75u
#define SPEED_TURN_MM_S             45u
// Slow start of line following (2026-11-24).  It used SPEED_SLOW_MM_S
// (75), faster than following itself (55), so the robot rejoined the line
// at an angle and faster than usual and the steering swung hard (wheels
// 28 to 166 mm/s).  40 is the slowest the left wheel runs reliably
// (FOLLOW_LEFT_MIN_MM_S).  The steering is limited to
// FOLLOW_SLOW_STEER_PERCENT of the speed meanwhile (normally
// STEER_LIMIT_PERCENT), for _LONG_MS after a bypass or line search and
// _SHORT_MS after a junction turn, calibration or a clear scan.
#define FOLLOW_SLOW_SPEED_MM_S      40u
#define FOLLOW_SLOW_STEER_PERCENT   35u
#define FOLLOW_SLOW_LONG_MS         1500u
#define FOLLOW_SLOW_SHORT_MS        600u
// 2026-11-17 (build 8): size of each on-the-spot turn, in percent of the
// angle asked for.  The robot coasts on after the wheels reach their
// distance, and on the track right turns came out too big, so the bypass
// came back to the line too shallow and missed it.  90 for right turns was
// tried and put back to 100 (bypass worse); change only after measuring.
// Live-tunable over MQTT (tlm_decode.py tune turn_right_pct ...).
#define TURN_RIGHT_PERCENT          100u
#define TURN_LEFT_PERCENT           100u
#define SPEED_MAX_MM_S              600u
#define PID_KP_MILLI                1200
// 2026-11-17 (build 8): Ki was tried at 250 and put back to 400; the logs
// showed no less shake.
#define PID_KI_MILLI                400
#define PID_KD_MILLI                50
// Steering gains scaled with the slower follow speed (were 100 / 60 at
// 130 mm/s): the same offset now gives the same path curvature.
#define STEER_KP_MILLI              55
#define STEER_KD_MILLI              25
// Line following: the steering correction is limited to this percentage of
// the base speed, so the inner wheel never stops.  A stopped wheel drops
// from the MOTOR_MIN_PERMILLE dead-band drive to zero and back, which is
// what made the robot jerk at low speed.
#define STEER_LIMIT_PERCENT         70u
// 2026-11-24: the limit for left curves only.  The left wheel cannot go
// below FOLLOW_LEFT_MIN_MM_S, so at 70 the tightest left curve was about
// 12 cm radius and the robot ran off a sharp left bend.  At 100 the right
// wheel may reach about 150 mm/s, about 10 cm radius.  The left wheel never
// stops (its floor), so the reason for the 70 limit does not apply.
#define STEER_LEFT_LIMIT_PERCENT    100u
// Line following: the lowest speed each wheel is asked for.  Measured
// 2026-11-17 at 75 mm/s: in left curves the left (inner) wheel was asked
// for 22 mm/s, stuck near 0 to 30 and then surged past 100, which shook
// the robot; the right wheel ran steadily at 21 to 50 in right curves.
// What is added to a wheel is added to the other one too, which keeps the
// turn rate but widens the turn (86 to 106 mm at full steering).  A floor
// on both wheels made right curves shake, so only the left has one.
#define FOLLOW_LEFT_MIN_MM_S        40u
#define FOLLOW_RIGHT_MIN_MM_S       0u
// Line following: each wheel's target may change by at most this much per
// 10 ms motion period (8 mm/s per period = 0.8 m/s^2), smoothing steering
// steps and the start from rest.
#define FOLLOW_SLEW_MM_S            8
// Line following: while the barcode sensor is crossing bars, the steering
// can be held straight for at most this long, so the sensor crosses the
// barcode square.  2026-11-11: tried at 2000 ms, but dropping and then
// restoring the steering made the robot drift and snap back (a wobble at
// every barcode), so it is off (0).
#define BARCODE_STRAIGHT_MS         0u
// Steering sign: 1 or -1.  Flip it if the robot steers away from the line
// (which edge it follows depends on where the centre sensor is mounted).
#define STEER_DIRECTION             1
// 2026-11-17 (build 8): 300 to 500 ms.  The inner wheel sticking for a
// moment in a tight curve could count as a stall and stop the robot (a
// likely cause of the one unexplained stop).  A real stall still stops it
// within half a second.
#define STALL_TIMEOUT_MS            500u
#define ENC_EDGE_TIMEOUT_MS         150u
// Encoder noise filter: edges closer than this are ignored.  At the top
// speed (600 mm/s) real edges are 8.5 ms apart, so 5 ms only drops noise,
// such as bursts picked up from the motor wires.
// Edges closer than this are noise.  At 0.18 mm per edge the wheel gives an
// edge every 1.2 ms at line-following speed, so this must stay well below.
#define ENC_MIN_EDGE_US             200u
#define STOPPED_SPEED_MM_S          20u
#define LINE_STALE_MS               50u

/* ------------------------------------------------------------------------ */
/* Buddy 3: line and barcode tuning.                                       */
/* ------------------------------------------------------------------------ */

#define LINE_MIN_CONTRAST           400u
#define LINE_JUNCTION_MS            20u
// Was 150: 500 lets the ramped left turn find the line before a sweep.
#define LINE_LOST_MS                500u
// 2026-11-24: a line lost again within LINE_RELOST_MS of following resuming
// is not searched for at once: the robot keeps following (steering hard to
// the side the line went) for up to LINE_RELOST_WAIT_MS more.  At a sharp
// left bend it lost and found the line 6 times in 2 s, lurching each time.
#define LINE_RELOST_MS              1000u
#define LINE_RELOST_WAIT_MS         700u
#define LINE_FOUND_MS               30u
// Longest gap allowed between edges before a capture ends.  It must exceed
// the widest element at the slowest reading speed: 400 ms lets a 17 mm bar
// pass at 45 mm/s (about a third of line-following speed), and once some
// elements are in the gap grows with them (see barcode.c).
#define BARCODE_QUIET_MS            400u
// Elements shorter than this are comparator flicker at a blurred bar edge
// and are merged away (2026-11-11, digital mode).  A 1.5 mm narrow bar
// lasts 33 ms at 45 mm/s, so real elements are far longer.
#define BARCODE_GLITCH_MS           3u
#define BARCODE_FIFO_SLOTS          64u

// Analogue barcode sensing.  Both IR analogue inputs are converted in turn,
// IR_SAMPLE_HZ times a second each.  A bar needs the black-white contrast
// seen so far to be at least BARCODE_MIN_SPAN ADC counts.  It is noticed when
// the reading rises above the white level by that contrast divided by
// 2^BARCODE_ENTER_SHIFT and ends when the reading falls below
// BARCODE_SPLIT_PERCENT of the bar's own height.  Both of its edges are then
// placed at that level (the start is found in the last BARCODE_HISTORY
// samples), so a bar narrower than the sensor's spot, which never reaches
// full black, still measures at its true width.  The contrast envelope
// decays by 1/2^BARCODE_DECAY_SHIFT per sample (about 0.5 s).
//
// These BARCODE_* settings are used only by the barcode sensor (GP28,
// barcode.c); the line-following sensors use the LINE_* settings.
//
// 2026-11-11: tuned for the new barcode sensor, which cannot be mounted
// lower than 1-2 cm (it would hit the hump).  At that height solid black
// read 3965 but barcode bars only ~1160 against ~160 white, and the thin
// gaps between bars never fell halfway back to white, so bars merged and
// a swipe gave 1-3 elements instead of 29.  Was: span 300, enter 3,
// split 50, average 16, min bar 8 ms, accept 2.
#define IR_SAMPLE_HZ                2000u
#ifndef BARCODE_MIN_SPAN
#define BARCODE_MIN_SPAN            200u
#endif
#ifndef BARCODE_ENTER_SHIFT
#define BARCODE_ENTER_SHIFT         4u
#endif
// A bar ends once the reading has dropped from its peak to this percentage
// of the bar's height above white: 65 splits two bars whose gap only dips
// 35% of the way back to white.
#ifndef BARCODE_SPLIT_PERCENT
#define BARCODE_SPLIT_PERCENT       65u
#endif
#define BARCODE_DECAY_SHIFT         10u
#define BARCODE_HISTORY             1024u
// Readings pass through a 3-sample median, which removes single-sample
// spikes, then a BARCODE_AVERAGE-sample running average (8 samples = 4 ms,
// 2 ms of delay; the blur of the high sensor already smooths the signal, so
// less averaging keeps the shallow gaps between bars).  A bar is only
// accepted if it lasted at least BARCODE_MIN_BAR_MS and reached
// 1/2^BARCODE_ACCEPT_SHIFT of the contrast above white (an eighth).
// Overridable for host tests.
#ifndef BARCODE_AVERAGE
#define BARCODE_AVERAGE             8u
#endif
#ifndef BARCODE_MIN_BAR_MS
#define BARCODE_MIN_BAR_MS          4u
#endif
#ifndef BARCODE_ACCEPT_SHIFT
#define BARCODE_ACCEPT_SHIFT        3u
#endif

/* ------------------------------------------------------------------------ */
/* Buddy 4: IMU tuning.                                                    */
/* ------------------------------------------------------------------------ */

#define IMU_I2C_HZ                  400000u
#define IMU_FORWARD_AXIS            0u
#define IMU_FORWARD_SIGN            1
// The horizontal axis across the robot (Z points up on the GY-511).  Its
// sign does not matter: the hump detector uses the combined tilt.
#define IMU_SIDE_AXIS               1u
#define IMU_CALIB_SAMPLES           50u
// 2026-11-24: level is re-learnt from every still block of IMU_CALIB_SAMPLES
// in READY (imu.c relevel()); still means the raw pitch spans at most this.
#define IMU_RELEVEL_SPAN_MDEG       2000
// Identical readings in a row (20 ms apart) that mean the accelerometer
// has stopped updating and must be started again.
#define IMU_FROZEN_SAMPLES          50u
// Time after start-up before the IMU chip is first addressed.
#define IMU_START_DELAY_MS          300u
#define IMU_FAULT_ERRORS            3u
#define TURNING_MDEG_S              30000
#define ACCEL_EVENT_MM_S2           300
// Hump detection on the combined tilt (any direction).  2026-11-04: was
// pitch only at 6 / 3 degrees, which missed the course hump because only
// one wheel crosses it and the robot leans sideways.  5 degrees for 100 ms
// catches a 1.3 cm hump (peak lean asin(13 / 97) = 7.7 degrees).
// 2026-11-11: briefly raised to 7 degrees for 150 ms against false humps
// from the wobble, but that missed the 1.3 cm hump; the wobble is fixed
// and on-the-spot turns and braking are filtered out in imu.c instead.
#define HUMP_ENTER_MDEG             3000
#define HUMP_EXIT_MDEG              2000
#define HUMP_HOLD_MS                100u
// Line following must have lasted this long before hump samples count, so
// the lean from pulling away and speeding up is not measured as a hump.
#define HUMP_SETTLE_MS              1200u
// A shaky turn let the turn rate
// dip below the limit for a moment, which let a turn's lean read as a hump.
#define HUMP_AFTER_TURN_MS          500u
// Lifted drive wheel to the pivot line through the other drive wheel and
// the caster, for height = lever x sin(peak tilt).  Measured 2026-11-04:
// wheel spacing T = 115 mm, axle line to caster d = 90 mm, so
// lever = T x d / sqrt((T/2)^2 + d^2) = 10350 / 106.8 = 97 mm.
#define HUMP_LEVER_MM               97u
// Raised 2026-11-04 from 1500: driving onto the course hump gave a
// 1.63 g jolt, which counted as a collision and stopped the run.
#define IMPACT_MG                   2000u
#define IMPACT_COOLDOWN_MS          500u
// Lift detection (Week 12 requirement): the motors stop when the robot is
// picked up and tipped.  A lift is a lean of LIFT_TILT_MDEG or more lasting
// LIFT_HOLD_MS.  Turning and braking read up to 17 degrees and the 1.3 cm
// course hump 7.7, so 25 is beyond anything the floor does (a hump would
// have to be 41 mm high to reach it).  2026-11-17: an upward-push rule was
// tried and dropped; it cannot tell a gentle lift from a wheel on the hump.
#define LIFT_TILT_MDEG              25000
// for hump detection, the floor tilts
// the robot at most 7.7 degrees, so a lean of 15 (a 25 mm "hump") is
// taken as a lift.  The 25-degree limit above applies everywhere else.
#define LIFT_TILT_STEADY_MDEG       15000
// 2026-11-17: hold 80 to 300 ms.  Pulling away in reverse read 19.5
// degrees for a moment; a hand holds the lean for much longer.
#define LIFT_HOLD_MS                300u

/* ------------------------------------------------------------------------ */
/* Buddy 5: obstacle tuning.                                               */
/* ------------------------------------------------------------------------ */

// 2026-11-24: 250 to 190, to stop about 15 cm from the obstacle.  The robot
// stops about 4 cm after detecting (profiles read 17-23 cm at 250).  At
// 140 (about 12 cm) the first scan failed every time: the flat face
// reflected the sound away at the steep edge angles, the obstacle read
// 2-3 cm wide, and the robot reversed and scanned again.
#define OBSTACLE_DETECT_MM          190u
#define OBSTACLE_RELEASE_MM         350u
#define MONITOR_PERIOD_MS           60u
#define SONAR_TIMEOUT_MS            40u
#define SERVO_MIN_US                500u
#define SERVO_MAX_US                2500u
#define SERVO_LEFT_IS_HIGH          1u
#define SCAN_FINE_STEP_DEG          5u
// 2026-11-17 (build 8): 60 to 80 ms, and each angle is pinged SCAN_PINGS
// times and the middle reading kept.  One stray or missing echo used to end
// the obstacle early or extend it (widths of 3 to 43 cm for one obstacle).
#define SCAN_FINE_SETTLE_MS         80u
#define SCAN_PINGS                  3u
#define SCAN_ECHOES_NEEDED          2u
#define SCAN_PING_GAP_MS            15u
#define CLEARANCE_MAX_MM            1000u
#define OBSTACLE_PROFILE_MM         600u
#define SONAR_MAX_MM                2000u
#define SERVO_SETTLE_MS             150u
#define SERVO_CENTRE_DEG            90u
#define VEHICLE_WIDTH_MM            150u
#define CLEARANCE_MARGIN_MM         60u
// 2026-11-17 (build 8): sonar beam correction for the obstacle width.  The
// beam is a cone (about 15 degrees each side), so it still hears an edge
// after the servo has turned past it, and each edge was placed too far
// out.  Each scan point on the obstacle is moved this many degrees towards
// the obstacle's closest point (never past it).  Live-tunable over MQTT
// (dashboard keys 5/6): put a box of known width in front, scan, adjust.
#define OBS_BEAM_HALF_DEG           10u
// 2026-11-17 (build 8): the course obstacle is at most 20 cm wide (given
// by the module lecturer).  A wider scan reading is capped to this, so the
// bypass does not swing out onto a neighbouring line.
#define OBSTACLE_MAX_WIDTH_MM       200u

/* ------------------------------------------------------------------------ */
/* Integration: Vehicle Control.                                           */
/* ------------------------------------------------------------------------ */

#define REACQUIRE_TIMEOUT_MS        5000u
// Was 15000: a bypass is about 1 m of driving, which takes over 30 s at
// the slowest tuned speed (30 mm/s) and timed out into SAFE STOP.
// 2026-11-16: 90 s, because the bypass approach leg now runs until the
// line is found (up to BYPASS_APPROACH_MAX_MM).
#define SEQUENCE_TIMEOUT_MS         90000u
#define PROFILE_TIMEOUT_MS          10000u
#define INIT_SETTLE_MS              1500u
// The course obstacle is at most 200 mm long along the line (given
// 2026-11-16; was 150).  Its width is not known and comes from the scan.
#define OBSTACLE_DEPTH_EST_MM       200u
// 2026-11-24: 180 to 140; only used for the bypass pass leg (vehicle.c),
// which is now 4 cm shorter.
#define VEHICLE_LENGTH_MM           140u
#define BYPASS_RETRIES_MAX          2u
#define JUNCTION_ADVANCE_MM         60u
// Bypass (2026-11-11): back away this far before turning out, so the turn
// does not swing the robot into the obstacle, and step out at least this
// far sideways from the line (more for a wide obstacle).
// 2026-11-24: 50 to 100; the robot now stops about 15 cm away.
#define BYPASS_BACKUP_MM            100u
#define BYPASS_LATERAL_MM           150u
// 2026-11-17: the sideways distance worked out from the scanned width is
// cut to this percentage (was 100).  The robot swung out too wide and met
// a neighbouring line.  BYPASS_LATERAL_MM stays the least it will go.
#define BYPASS_LATERAL_PERCENT      80u
// 2026-11-24: added to the sideways distance after the percentage and the
// minimum, so every bypass steps out 2 cm further.
#define BYPASS_LATERAL_EXTRA_MM     20u
// On the way back, a line only counts once this percentage of the expected
// distance back has been covered.  Seen 2026-11-17: a neighbouring line
// 19 cm into a 51 cm return leg was taken for the one that was left.
// 2026-11-24: 50 to 15.  On the small course platform the line was met
// 5-6 cm into a 27 cm return leg (about 20 %) in every failed rejoin; it
// was ignored, the robot went on to other black further out, and the
// realign turn then missed the line.
#define BYPASS_REJOIN_GUARD_PERCENT 15u
// Bypass return (2026-11-16): the approach leg drives until the centre
// sensor finds the line, giving up after BYPASS_APPROACH_MAX_MM.  The robot
// then drives BYPASS_CENTRE_MM further, which is the distance from the
// centre sensor back to the axle, so its turning centre sits on the line,
// and turns back towards its old heading until the sensor meets the line
// again, giving up after BYPASS_REALIGN_MAX_DEG.
// 2026-11-24: 700 to 400.  The way back is about 210-270 mm, and on the
// small course platform a 70 cm miss drove far past the line before the
// sweep started; giving up sooner sweeps for the line close to it.
#define BYPASS_APPROACH_MAX_MM      400u
#define BYPASS_CENTRE_MM            JUNCTION_ADVANCE_MM
#define BYPASS_REALIGN_MAX_DEG      100

/* ------------------------------------------------------------------------ */
/* Buddy 1: MQTT.  MQTT_BROKER_IP is in config/wifi_credentials.h.         */
/* ------------------------------------------------------------------------ */

#define MQTT_BROKER_PORT            1883u
#define MQTT_CLIENT_ID              "aai03-robot"
#define MQTT_TOPIC_ROOT             "inf2004/aai03/"
#define MQTT_KEEPALIVE_S            10u

#endif /* ROBOT_CFG_H */

/*** end of file ***/

