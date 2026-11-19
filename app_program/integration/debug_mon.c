/** @file debug_mon.c
 *
 * @brief USB console monitor for the full mission (no MQTT needed).
 *
 * @par
 * Reads only the sys_status words and the fault flag, so it cannot change
 * what the robot does.  Runs at the telemetry priority (the lowest), so it
 * never delays a control task.
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
#include "sys_status.h"
#include "bitpack.h"
#include "line_follow.h"
#include "barcode.h"
#include "ir_hw.h"
#include "app_main.h"
#include "motion.h"
#include "imu.h"
#include "obstacle.h"
#include "vehicle.h"
#include "debug_mon.h"
#include "comms.h"
#include "tlm_codec.h"

// Time to open the serial monitor after the USB port appears.
#define BOOT_DELAY_MS           4000u

// Poll step, and how many steps between full status lines (1 s).
#define LOOP_STEP_MS            50u
#define REPORT_STEPS            20u
#define DUMP_STEPS              300u    // DIAGNOSTIC: every 15 s when idle.
#define SEND_EVERY_STEPS        2u      // DIAGNOSTIC: one chunk per 100 ms.
#define SEND_PASSES             3u      // DIAGNOSTIC: times it is repeated.
#define SEND_HEADER_BYTES       4u
#define SEND_SAMPLES            2u
#define SEND_BYTES              (SEND_HEADER_BYTES +                        \
                                 (SEND_SAMPLES * sizeof(motion_rec_t)))
#define BYTE_MASK               0xFFu
#define BYTE_BITS               8u

#define LETTER_A                65u     // ASCII 'A'.
#define FAULT_COUNT             8u
#define NO_OBSTACLE_CM          255u

static uint64_t g_stack[STACK_MEDIUM / sizeof(uint64_t)];

static char const * const gp_vehicle_names[] =
{
    "INITIALISING", "READY", "CALIBRATING", "LINE_FOLLOWING",
    "JUNCTION_ACTION", "OBSTACLE_SCAN", "BYPASS", "LINE_REACQUIRE",
    "REVERSE_RETRY", "SAFE_STOP", "MISSION_COMPLETE"
};

static char const * const gp_line_names[] =
{
    "CALIBRATING", "TRACKING", "JUNCTION", "LOST", "SEARCHING"
};

static char const * const gp_mode_names[] =
{
    "IDLE", "FOLLOWING", "MOVING", "TURNING", "BRAKING", "STALLED"
};

static char const * const gp_obstacle_names[] =
{
    "MONITOR", "DETECTED", "COARSE_SCAN", "FINE_SCAN", "PROFILE",
    "ECHO_FAULT"
};

static char const * const gp_fault_names[] =
{
    "motor stall", "encoder", "IMU", "ultrasonic no echo", "WiFi lost",
    "IR calibration", "barcode overflow", "MQTT lost"
};

static char const * const gp_action_names[] =
{
    "LEFT", "RIGHT", "STRAIGHT", "U-TURN"
};

/*!
 * @brief Values last printed, to report only changes.
 */
typedef struct
{
    uint32_t vehicle_state;
    uint32_t faults;
    uint32_t barcode_reads;
    uint32_t barcode_rejects;
    uint32_t hump_count;
    uint32_t lift_count;
    uint32_t obstacle_state;
} debug_mon_seen_t;

static void        debug_mon_task(INT start_code, void * p_ext_info);
static void        report_vehicle(debug_mon_seen_t * p_seen);
static void        report_faults(debug_mon_seen_t * p_seen);
static void        report_barcode(debug_mon_seen_t * p_seen);
static void        report_hump_obstacle(debug_mon_seen_t * p_seen);
static void        print_status(void);
static void        dump_record(void);
static void        send_record(void);
static char const * name_of(char const * const * pp_names, uint32_t count,
                            uint32_t index);


/*!
 * @brief Create the monitor task.
 *
 * @return The task handle (> 0), or a negative kernel error code.
 */
int32_t
debug_mon_start (void)
{
    return (robot_ipc_start_task(debug_mon_task, PRIO_TELEMETRY, g_stack,
                                 sizeof(g_stack)));
}


/*!
 * @brief Monitor task: report changes at once, the full status each second.
 *
 * @param[in] start_code  Unused start code.
 * @param[in] p_ext_info  Unused extended information.
 */
static void
debug_mon_task (INT start_code, void * p_ext_info)
{
    // Out-of-range values make the first pass report everything.
    debug_mon_seen_t seen = { 99u, 0u, 0u, 0u, 0u, 0u, 99u };
    uint32_t         step = 0u;
    uint32_t         dump_step = 0u;

    (void) start_code;  // Unused: started with code 0.
    (void) p_ext_info;  // Unused: no extended information.

    robot_ipc_sleep_ms(BOOT_DELAY_MS);

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "\n=== AAI03 full robot, USB monitor ===\n"
                     "GP20: start / end run / reset.\n\n");

    for (;;)
    {
        report_vehicle(&seen);
        report_faults(&seen);
        report_barcode(&seen);
        report_hump_obstacle(&seen);

        send_record();
        dump_step++;

        if ((dump_step >= DUMP_STEPS) && (motion_rec_is_idle()) &&
            (motion_rec_count() > 0u))
        {
            dump_step = 0u;
            dump_record();
        }

        step++;

        if (step >= REPORT_STEPS)
        {
            step = 0u;
            print_status();
        }

        robot_ipc_sleep_ms(LOOP_STEP_MS);
    }
}


/*!
 * @brief DIAGNOSTIC BUILD ONLY: send the line-following record over MQTT.
 *
 * @par
 * Called every monitor step.  While the robot is not line following, the
 * record goes out two samples at a time on the debug topic, SEND_PASSES
 * times over so that a lost message is filled in by a later pass.  Each
 * chunk carries a run number and the index of its first sample.
 */
static void
send_record (void)
{
    static uint32_t s_serial = 0u;
    static uint32_t s_run    = 0u;
    static uint32_t s_index  = 0u;
    static uint32_t s_pass   = SEND_PASSES;
    static uint32_t s_step   = 0u;
    uint32_t const  count    = motion_rec_count();

    if (!motion_rec_is_idle())
    {
        s_pass = SEND_PASSES;  // Recording: nothing to send yet.
    }
    else if (motion_rec_serial() != s_serial)
    {
        // A new record: start sending it from the beginning.
        s_serial = motion_rec_serial();
        s_run++;
        s_index  = 0u;
        s_pass   = 0u;
    }
    else
    {
        // Same record as before: carry on.
    }

    s_step++;

    if ((motion_rec_is_idle()) && (s_pass < SEND_PASSES) &&
        (s_step >= SEND_EVERY_STEPS) && (count >= SEND_SAMPLES))
    {
        uint8_t      chunk[SEND_BYTES];
        motion_rec_t rec[SEND_SAMPLES];

        s_step = 0u;
        motion_rec_get(s_index, &rec[0]);
        motion_rec_get(s_index + 1u, &rec[1]);

        chunk[0] = (uint8_t) (s_run & BYTE_MASK);  // One byte.
        chunk[1] = (uint8_t) ((count / 4u) & BYTE_MASK);  // One byte.
        chunk[2] = (uint8_t) (s_index & BYTE_MASK);  // One byte.
        chunk[3] = (uint8_t) ((s_index >> BYTE_BITS) & BYTE_MASK);  // Byte.

        {
            uint8_t const * const p_bytes = (uint8_t const *) rec;  // Raw.

            for (uint32_t idx = 0u; idx < sizeof(rec); idx++)
            {
                chunk[SEND_HEADER_BYTES + idx] = p_bytes[idx];
            }
        }

        // A dropped chunk is sent again on a later pass.
        (void) comms_queue(TLM_TOPIC_DEBUG, chunk, sizeof(chunk));

        s_index += SEND_SAMPLES;

        if ((s_index + SEND_SAMPLES) > count)
        {
            s_index = 0u;
            s_pass++;
        }
    }
}


/*!
 * @brief DIAGNOSTIC BUILD ONLY: print the line-following record.
 */
static void
dump_record (void)
{
    uint32_t const count = motion_rec_count();

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "T begin %lu\n", count);

    for (uint32_t index = 0u; index < count; index++)
    {
        motion_rec_t rec;

        motion_rec_get(index, &rec);

        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "T %lu %lx %d %d %d %d %d %d\n", index,
                         rec.line_word, rec.target[0], rec.target[1],
                         rec.speed[0], rec.speed[1], rec.output[0],
                         rec.output[1]);
    }

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) "T end\n");
}


/*!
 * @brief Print the vehicle state when it changes.
 *
 * @param[in,out] p_seen  Values last printed.
 */
static void
report_vehicle (debug_mon_seen_t * p_seen)
{
    uint32_t const word  = sys_status_read(SYS_STATUS_VEHICLE);
    uint32_t const state = bitpack_word_get(word, VEHICLE_STATE_SHIFT,
                                            VEHICLE_STATE_WIDTH);

    if (state != p_seen->vehicle_state)
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "### STATE -> %s\n",
                         name_of(gp_vehicle_names,
                                 (uint32_t) VEHICLE_MISSION_COMPLETE + 1u,
                                 state));  // Cast: enum to a count.
        p_seen->vehicle_state = state;
    }
}


/*!
 * @brief Print each fault the moment it is raised or cleared.
 *
 * @param[in,out] p_seen  Values last printed.
 */
static void
report_faults (debug_mon_seen_t * p_seen)
{
    uint32_t const faults  = robot_ipc_faults();
    uint32_t const changed = faults ^ p_seen->faults;

    for (uint32_t bit = 0u; bit < FAULT_COUNT; bit++)
    {
        uint32_t const mask = 1u << bit;

        if (0u != (changed & mask))
        {
            // (UB *): string literal to the monitor API's character type.
            (void) tm_printf((UB *) "!!! FAULT %s: %s\n",
                             (0u != (faults & mask)) ? "RAISED" : "cleared",
                             gp_fault_names[bit]);
        }
    }

    p_seen->faults = faults;
}


/*!
 * @brief Print each barcode read or reject as its counter moves.
 *
 * @param[in,out] p_seen  Values last printed.
 */
static void
report_barcode (debug_mon_seen_t * p_seen)
{
    uint32_t const word    = sys_status_read(SYS_STATUS_BARCODE);
    uint32_t const reads   = bitpack_word_get(word, BARCODE_READS_SHIFT,
                                              BARCODE_COUNT_WIDTH);
    uint32_t const rejects = bitpack_word_get(word, BARCODE_REJECTS_SHIFT,
                                              BARCODE_COUNT_WIDTH);
    uint32_t const vehicle = sys_status_read(SYS_STATUS_VEHICLE);

    if (reads != p_seen->barcode_reads)
    {
        uint32_t const letter = bitpack_word_get(word, BARCODE_LETTER_SHIFT,
                                                 BARCODE_LETTER_WIDTH);
        uint32_t const action = bitpack_word_get(vehicle,
                                                 VEHICLE_ACTION_SHIFT,
                                                 VEHICLE_ACTION_WIDTH);

        // (UB *): string literal to the monitor API's character type.  The
        // letter field is below 26, so LETTER_A + letter is printable.
        (void) tm_printf((UB *) ">>> BARCODE '%c' -> next junction %s\n",
                         LETTER_A + letter, gp_action_names[action]);
        p_seen->barcode_reads = reads;
    }

    if (rejects != p_seen->barcode_rejects)
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "xxx barcode rejected (%lu so far)\n",
                         rejects);
        p_seen->barcode_rejects = rejects;
    }
}


/*!
 * @brief Print each new hump and each obstacle state change.
 *
 * @param[in,out] p_seen  Values last printed.
 */
static void
report_hump_obstacle (debug_mon_seen_t * p_seen)
{
    uint32_t const hump  = sys_status_read(SYS_STATUS_HUMP);
    uint32_t const count = bitpack_word_get(hump, IMU_HUMP_COUNT_SHIFT,
                                            IMU_HUMP_COUNT_WIDTH);
    uint32_t const obs   = sys_status_read(SYS_STATUS_OBSTACLE);
    uint32_t const state = bitpack_word_get(obs, OBSTACLE_STATE_SHIFT,
                                            OBSTACLE_STATE_WIDTH);

    if (count != p_seen->hump_count)
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "^^^ HUMP %lu mm (highest %lu mm)\n",
                         bitpack_word_get(hump, IMU_HUMP_LAST_SHIFT,
                                          IMU_HUMP_MM_WIDTH),
                         bitpack_word_get(hump, IMU_HUMP_MAX_SHIFT,
                                          IMU_HUMP_MM_WIDTH));
        p_seen->hump_count = count;
    }

    if (imu_lift_count() != p_seen->lift_count)
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "/// LIFTED: motors stopped\n");
        p_seen->lift_count = imu_lift_count();
    }

    if (state != p_seen->obstacle_state)
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "[ ] OBSTACLE %s at %lu cm\n",
                         name_of(gp_obstacle_names,
                                 (uint32_t) OBSTACLE_STATE_ECHO_FAULT + 1u,
                                 state),  // Cast: enum to a count.
                         bitpack_word_get(obs, OBSTACLE_DIST_SHIFT,
                                          OBSTACLE_DIST_WIDTH));
        p_seen->obstacle_state = state;
    }
}


/*!
 * @brief Print one line with every subsystem's current values.
 */
static void
print_status (void)
{
    uint32_t const line   = sys_status_read(SYS_STATUS_LINE);
    uint32_t const motion = sys_status_read(SYS_STATUS_MOTION);
    uint32_t const imu    = sys_status_read(SYS_STATUS_IMU);
    uint32_t const obs    = sys_status_read(SYS_STATUS_OBSTACLE);
    uint32_t const dist   = bitpack_word_get(obs, OBSTACLE_DIST_SHIFT,
                                             OBSTACLE_DIST_WIDTH);
    int32_t const  pitch  = bitpack_to_signed(
                                bitpack_word_get(imu, IMU_PITCH_SHIFT,
                                                 IMU_PITCH_WIDTH),
                                IMU_PITCH_WIDTH);

    // (UB *): string literal to the monitor API's character type.  Pitch
    // is in half degrees; the speeds are excess-1024 fields.
    (void) tm_printf((UB *) "line %-9s c%4lu off%5ld | motor %-9s "
                     "L%4ld R%4ld | tilt %3ld hdg %3lu | front ",
                     name_of(gp_line_names, 5u,
                             bitpack_word_get(line, LINE_STATE_SHIFT,
                                              LINE_STATE_WIDTH)),
                     bitpack_word_get(line, LINE_CENTRE_SHIFT,
                                      LINE_CENTRE_WIDTH),
                     line_follow_offset(line),
                     name_of(gp_mode_names, 6u,
                             bitpack_word_get(motion, MOTION_MODE_SHIFT,
                                              MOTION_MODE_WIDTH)),
                     bitpack_to_signed(bitpack_word_get(motion,
                                           MOTION_LEFT_SHIFT,
                                           MOTION_SPEED_WIDTH),
                                       MOTION_SPEED_WIDTH),
                     bitpack_to_signed(bitpack_word_get(motion,
                                           MOTION_RIGHT_SHIFT,
                                           MOTION_SPEED_WIDTH),
                                       MOTION_SPEED_WIDTH),
                     pitch / 2,
                     bitpack_word_get(imu, IMU_HEADING_SHIFT,
                                      IMU_HEADING_WIDTH));

    if (NO_OBSTACLE_CM == dist)
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "clear");
    }
    else
    {
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) "%lu cm", dist);
    }

    {
        uint32_t dark  = 0u;
        uint32_t white = 0u;
        uint32_t black = 0u;

        uint32_t calls    = 0u;
        uint32_t overruns = 0u;

        barcode_debug_levels(&dark, &white, &black);
        ir_hw_sampler_stats(&calls, &overruns);
        // (UB *): string literal to the monitor API's character type.
        (void) tm_printf((UB *) " | bc d%4lu w%4lu b%4lu adc n%lu ov%lu "
                         "boot%02lx", dark, white, black, calls, overruns,
                         app_main_boot_failures());
    }

    // (UB *): string literal to the monitor API's character type.
    (void) tm_printf((UB *) " | faults 0x%02lx\n", robot_ipc_faults());
}


/*!
 * @brief Look up a name safely.
 *
 * @param[in] pp_names  Name table.
 * @param[in] count     Entries in the table.
 * @param[in] index     Value to name.
 *
 * @return The name, or "?" if the value is out of range.
 */
static char const *
name_of (char const * const * pp_names, uint32_t count, uint32_t index)
{
    return ((index < count) ? pp_names[index] : "?");
}

/*** end of file ***/

