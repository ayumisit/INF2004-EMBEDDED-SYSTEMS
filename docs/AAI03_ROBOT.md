# AAI03 Autonomous Robot Car: firmware guide

micro T-Kernel 3.0 on the Raspberry Pi Pico W + Cytron Robo Pico, written in C
with the Pico C SDK and checked against the Barr-C:2018 coding standard.

The firmware is built on the RP2040 port of micro T-Kernel 3.0
(`sirfonzie/mtk3smp-rp2040`, linked from `sirfonzie/mtk3-hub`).  The kernel,
the port and the build system are unchanged apart from the small board changes
listed in [PORT_CHANGES.md](PORT_CHANGES.md).  The robot application lives in
`app_program/`.

| Document | Contents |
|---|---|
| this file | Build, flash, wiring, task design, bring-up checklist |
| [BARR_C_COMPLIANCE.md](BARR_C_COMPLIANCE.md) | How every Barr-C rule is enforced, and the deviations |
| [MQTT_PROTOCOL.md](MQTT_PROTOCOL.md) | Topics, binary frames, commands, the PC tool (Buddy 1) |
| [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md) | Change Requirement 1: every field, its range and its bits |
| [PORT_CHANGES.md](PORT_CHANGES.md) | Every change made to the micro T-Kernel port, and why |
| [ABBREVIATIONS.md](ABBREVIATIONS.md) | Project abbreviation table (Barr-C rule 1.5.b) |

---

## 1. Who owns what

Every buddy has their own folder.  Files ending in `_hw.c` or `_net.c` are the
only ones that touch the Pico SDK, lwIP or the radio; everything else is
hardware-independent C that talks to the kernel.

| Folder | Owner | Modules |
|---|---|---|
| `app_program/` | Integration | `app_main` (start-up order, `usermain()`) |
| `platform/` | Integration / platform | `robot_cfg.h` (pins and tuning), `board_hw` (resets, timer, GPIO), `gpio_irq` (shared GPIO interrupt), `robot_ipc` (all shared kernel objects), `period_tick` (10 ms release of periodic tasks), `sys_status` (packed status board), `bitpack` (minimum-bit packing), `fix_math` (integer trig), `sdk_stub_hw` |
| `buddy1_comms/` | Buddy 1 | `comms` (queue, command routing, link state), `comms_net` (WiFi recovery + MQTT client in the radio task), `telemetry` (Telemetry Task), `tlm_codec` (binary frames) |
| `buddy2_motion/` | Buddy 2 | `motor_hw` (PWM H-bridge), `encoder` (edge ISR + Encoder Task), `pid_ctrl`, `motion` (Motion Control Task and the motion API) |
| `buddy3_line/` | Buddy 3 | `ir_hw` (ADC + digital IR), `line_follow` (Line Following Task), `barcode` (Barcode Task + edge FIFO), `code39` (decoder) |
| `buddy4_imu/` | Buddy 4 | `lsm303_hw` (I2C driver), `imu` (IMU Task), `hump` (hump state machine) |
| `buddy5_obstacle/` | Buddy 5 | `servo_hw`, `ultrasonic` (HC-SR04 + sem_echo), `obstacle` (Obstacle Scanning Task), `obs_profile` (profile and plan) |
| `integration/` | Shared (owner TBC) | `vehicle` (Vehicle Control Task, system statechart), `nav_map` (barcode letter to action map) |

The motion API the brief asks for is in `motion.h`:
`motion_move_forward(mm, speed)`, `motion_move_backward(mm, speed)`,
`motion_turn_left(deg)`, `motion_turn_right(deg)`, `motion_stop()` and
`motion_follow_line(speed)`.

---

## 2. Wiring used by this firmware

From the wiring record of 2026-10-20, after the GP26 conflict fix.  All pins
are defined once, in `app_program/platform/robot_cfg.h`.

| Device | Signal | GPIO | Notes |
|---|---|---|---|
| LSM303DLHC (IMU) | SDA / SCL | GP0 / GP1 | I2C0, 400 kHz.  The kernel's UART pins are disabled so I2C can use them |
| IR1 (left line sensor) | DO | GP3 | AO on GP2 is not an ADC pin; its input is disabled |
| HC-SR04 | ECHO / TRIG | GP4 / GP5 | **Confirm which wire is which**; swap in `robot_cfg.h` if needed.  ECHO needs a 5 V to 3.3 V divider |
| Right encoder | A / B | GP6 / GP26 | One channel counted (`ENC_QUADRATURE 0u`) |
| IR3 (barcode sensor) | DO | GP28 | AO on GP7 is not an ADC pin; its input is disabled |
| Motors (Robo Pico) | M1A M1B M2A M2B | GP8 GP9 GP10 GP11 | Which motor is left is TBC |
| Servo | PWM | GP15 | 50 Hz, own PWM slice |
| Left encoder | A / B | GP16 / GP17 | |
| START button | | GP20 | Robo Pico on-board button, active low |
| IR2 (centre line sensor) | AO | GP27 (ADC1) | DO not connected |
| Kernel liveness LED | | GP14 | Moved from GP16 (left encoder) |

---

## 3. Building

### What you need

* `arm-none-eabi-gcc` (the port is qualified with 13.2.1), GNU make, a host
  `g++` (the build compiles its own ELF-to-UF2 tool) and Python 3 (for the
  Barr-C scan).
* The Raspberry Pi Pico SDK **2.2.0** with the `lwip`, `cyw43-driver` and
  `tinyusb` submodules:
  ```sh
  git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git
  cd pico-sdk
  git submodule update --init lib/lwip lib/cyw43-driver lib/tinyusb
  ```
  By default the build looks for it at `../sdk/pico-sdk` next to this
  repository; otherwise pass `PICO_SDK_PATH=...`.

### Local settings (never commit these)

1. WiFi: `cp config/wifi_credentials.example.h config/wifi_credentials.h`,
   fill in the SSID and password, and set `UTK_WIFI_CREDENTIALS_SET` to 1.
2. Broker: set `MQTT_BROKER_IP` (and the group in `MQTT_TOPIC_ROOT`) in
   `app_program/platform/robot_cfg.h`.

### Build commands

```sh
cd build_make

# Full robot: USB console, WiFi, DHCP, MQTT (the competition build)
make SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 MQTT=1 -j8

# Bench build without the radio (everything except Buddy 1's network)
make SMP=0 CONSOLE=usb_cdc -j8
```

The image is named after the profile, for example
`mtk3pico_smp0_usb_cdc_wifi_dhcp_mqtt.uf2`.  Hold BOOTSEL, plug the Pico in,
and copy the `.uf2` onto the `RPI-RP2` drive.

Every build first runs `tools/barr_check.py` over `app_program/` and stops on
any violation, then compiles the kernel-side modules as strict ISO C99 and the
hardware modules as gnu11, both with `-Werror` (details in
BARR_C_COMPLIANCE.md).  `BARR_CHECK=0` skips the scan on a machine without
Python.

The build refuses `SMP=1` and `CONSOLE=uart` on purpose: GP0/GP1 are the IMU
bus, and the shared-data rules in this application are proven for one core.

### Host unit tests

```sh
make -C tests
```

Builds the hardware-independent modules (Code 39 decoder, hump estimator,
obstacle profiler, bit packing, fixed-point maths, MQTT codec, barcode map,
PID) with the PC compiler and runs 84 checks, then cross-checks the PC
decoder `tools/tlm_decode.py` against frames built by the firmware codec.

---

## 4. RTOS design

### Tasks

Priority 1 is the highest.  The port's own USB-CDC service runs at 3 and the
CYW43 radio-owner task at 4; every control and sensing task runs above them so
the network can never delay control (NFR17).

| Task | Owner | Priority | Release | Stack |
|---|---|---|---|---|
| Vehicle Control | Integration | 1 | Event flag, 50 ms timeout | 2 KiB |
| Barcode | Buddy 3 | 1 | Woken by the GPIO ISR (`tk_wup_tsk`) | 2 KiB |
| Encoder | Buddy 2 | 2 | Every 10 ms | 1 KiB |
| Line Following | Buddy 3 | 2 | Every 10 ms | 1 KiB |
| Motion Control | Buddy 2 | 2 | Every 10 ms | 2 KiB |
| Obstacle Scanning | Buddy 5 | 2 | 60 ms monitor, or SCAN_REQ | 2 KiB |
| (USB-CDC console) | port | 3 | | |
| Communication = CYW43 radio task + `comms_net` poll | port + Buddy 1 | 4 | Radio poll | 6 KiB |
| IMU | Buddy 4 | 5 | Every 20 ms | 2 KiB |
| Telemetry | Buddy 1 | 8 | Every 200 ms | 2 KiB |

Periodic tasks are released by one 10 ms cyclic handler (`period_tick`),
so their periods do not drift.  If a task is still running when its next
release arrives, the kernel queues the wake-up and `period_tick_wait()` counts
it as a missed period.  The total is published in telemetry (HEALTH word),
which is the evidence for NFR17 ("miss no period").

### Shared objects (`robot_ipc`)

| Object | Kind | Writers | Reader |
|---|---|---|---|
| `flg_vehicle_evt` | Event flag | Line, Barcode, Motion, Obstacle, IMU, Comms | Vehicle Control |
| `flg_fault` | Event flag (fault bitmask) | Any subsystem | Vehicle Control, Telemetry |
| `flg_comm` | Event flag | Comms, Vehicle Control | Telemetry |
| `flg_obstacle` | Event flag (SCAN_REQ) | Vehicle Control | Obstacle |
| `mbf_nav_cmd` | Message buffer, 1 B | Barcode | Vehicle Control |
| `mbf_obs_profile` | Message buffer, 6 B | Obstacle | Vehicle Control |
| `mbf_ctrl_cmd` | Message buffer, 4 B | Comms | Vehicle Control |
| `mbf_motion_cmd` | Message buffer, 4 B | Vehicle Control | Motion |
| `mbf_tlm_event` | Message buffer, 4 B | Every producer | Telemetry |
| `mbf_mqtt_tx` | Message buffer, up to 40 B | Telemetry, Comms | Radio task |
| `enc_state` | Mutex (priority inheritance) | Encoder | Motion, IMU |
| `line_state` | One packed 32-bit word | Line | Motion |
| `sys_status` | Eleven packed 32-bit words, one writer each | Owners | Telemetry, Vehicle |
| `sem_echo` | Semaphore (Buddy 5's module) | ECHO ISR | Obstacle |

Every buffer and stack is statically allocated (`TA_USERBUF`), so the whole
RAM budget is visible in the linker map.  The application uses about 20 KB of
flash and 16.7 KB of RAM, 14 KB of which is task stacks.

### One GPIO interrupt

The RP2040 has a single interrupt for all bank-0 pins.  `gpio_irq` owns it,
takes one timestamp, acknowledges every pending edge and calls the handler
attached to each pin: encoder counting (Buddy 2), barcode edge FIFO (Buddy 3)
and ECHO timing (Buddy 5).  Handlers only record data and signal a task.

---

## 5. System behaviour

States (`vehicle.h`): INITIALISING, READY, CALIBRATING, LINE_FOLLOWING,
JUNCTION_ACTION, OBSTACLE_SCAN, BYPASS, LINE_REACQUIRE, REVERSE_RETRY,
SAFE_STOP, MISSION_COMPLETE.

1. **INITIALISING** (1.5 s): the robot must be still while the IMU removes its
   mounting bias.  If the IMU does not answer, the robot goes to SAFE_STOP;
   pressing START acknowledges this and allows a run without hump data.
2. **READY**: motors off.  START (GP20 button or MQTT `start`) begins a run.
3. **CALIBRATING**: the robot turns 25 degrees left, 50 right and 25 left
   across the line while the centre IR records its darkest and brightest
   readings.  Too little contrast is a fault (SAFE_STOP).  **Place the robot
   on the line before pressing START.**
4. **LINE_FOLLOWING**: speed PID on both wheels plus PD steering from the
   line offset.  Decoded barcodes are converted to an action immediately and
   latched (a newer barcode replaces an older one).
5. **JUNCTION_ACTION**: drive 60 mm to put the axle on the junction, then turn
   90 degrees left or right, or turn 180 degrees, or drive 100 mm straight
   across.  No barcode latched means straight on.
6. **OBSTACLE_SCAN, BYPASS**: stop, scan, then leave the line at 45 degrees on
   the side that fits the vehicle and deviates least, pass the obstacle, and
   come back at 45 degrees until the line is found.  If neither side fits:
   **REVERSE_RETRY** (back 100 mm, scan again, at most twice).
7. **LINE_REACQUIRE**: a line lost for no known reason starts a left-right
   sweep; nothing within 5 s means SAFE_STOP.
8. **SAFE_STOP**: on impact, stall, ultrasonic fault, calibration failure or a
   timeout.  Press START or send `reset` to return to READY.
9. **MISSION_COMPLETE**: press START during a run or send `stop`.  The run
   summary (highest hump, barcodes, obstacles, distance) is published.

WiFi or broker loss never changes the robot's state.

---

## 6. Bring-up checklist (do these in order on the real robot)

Every value marked TBC in `robot_cfg.h` is an assumption until measured.

1. **ECHO voltage**: if the HC-SR04 is powered at 5 V, fit the divider on GP4
   before flashing.
2. **GP14**: make sure nothing is plugged into the servo header GP14 (the
   kernel drives it as its liveness LED).
3. Flash the bench build, open the USB serial port and check the line
   `AAI03 robot: boot failures 0x0000`.  Any set bit names the subsystem that
   failed to start (`app_main.h`).
4. **TRIG/ECHO**: wave a hand in front of the sensor and watch the obstacle
   distance in telemetry; if it never changes, swap `PIN_SONAR_ECHO` and
   `PIN_SONAR_TRIG`.
5. **Motor direction**: lift the wheels and press START.  The calibration
   sweep turns left first, so the left wheel must run backwards and the right
   wheel forwards.  If a wheel turns the wrong way, set `MOTOR_LEFT_REVERSED`
   or `MOTOR_RIGHT_REVERSED`; if the two wheels are swapped, swap the left and
   right motor pin pairs.
6. **Encoders**: push the robot 1 m by hand and compare the odometry; adjust
   `ENC_UM_PER_EDGE`.  Turn it 360 degrees and adjust `TRACK_WIDTH_MM`.
7. **IR polarity**: hold each sensor over black and white and watch the line
   word; change `IR_DO_BLACK_IS_HIGH` / `IR_AO_BLACK_IS_HIGH` if inverted.
8. **IMU axis**: tilt the nose up; pitch must go positive.  Otherwise change
   `IMU_FORWARD_AXIS` / `IMU_FORWARD_SIGN`.
9. **Servo direction**: during a scan, 150 degrees must point left; otherwise
   set `SERVO_LEFT_IS_HIGH` to `0u`.
10. Tune the PID (`PID_*`), steering (`STEER_*`) and speeds on the track, then
    the thresholds (junction, hump, impact, obstacle).

---

## 7. Known limits and open points

* The end-of-course trigger is not defined by the brief; this firmware ends a
  run with the START button or the MQTT `stop` command.
* Right-only junctions (assumption A8) are not detected: the left and centre
  sensors cannot see a branch to the right.
* Hump height assumes both wheels climb together (assumption A4).
* Heading is not tilt-compensated and the motors disturb it, so it is reported
  but not used for control (assumption A6).
* Single-core only (`SMP=0`).

