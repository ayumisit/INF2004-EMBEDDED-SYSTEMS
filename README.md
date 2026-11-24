# Autonomous Robot Car (INF2004 Embedded Systems, team AAI03)

A line-following robot car built on a Raspberry Pi Pico W and a Cytron Robo
Pico board. The firmware runs on **micro T-Kernel 3.0** (RP2040 port by
sirfonzie), is written in C, and follows the **Barr-C:2018** coding standard.

The robot:

* follows a black line using PID speed control on each wheel and PD steering;
* reads **Code 39** barcodes (A to Z, either direction) and uses them to pick
  its turn at the next junction. The letter-to-turn map is sent from the PC
  over MQTT and is never hard-coded;
* measures **hump** height with the IMU and beeps when it crosses one;
* **detects obstacles** with an ultrasonic sensor, scans them with a servo to
  measure their width, drives around them and rejoins the line;
* **stops if it is lifted** off the ground;
* streams **telemetry** to a PC over Wi-Fi/MQTT and accepts commands and live
  tuning from it.

---

## Contents

1. [Team](#1-team)
2. [Hardware and wiring](#2-hardware-and-wiring)
3. [Repository layout](#3-repository-layout)
4. [Software design](#4-software-design)
5. [Building](#5-building)
6. [Flashing and running](#6-flashing-and-running)
7. [PC tools and MQTT](#7-pc-tools-and-mqtt)
8. [Testing](#8-testing)
9. [Tuning](#9-tuning)
10. [Further documentation](#10-further-documentation)
11. [Credits and licences](#11-credits-and-licences)

---

## 1. Team

| Buddy | Student ID | Responsibility | Code |
|---|---|---|---|
| 1 | 2503687 | Communication: Wi-Fi, MQTT telemetry and commands | `app_program/buddy1_comms/` |
| 2 | 2502437 | Motion: motors, wheel encoders, PID speed control | `app_program/buddy2_motion/` |
| 3 | 2502515 | Line following and barcode decoding (IR sensors) | `app_program/buddy3_line/` |
| 4 | 2501723 | IMU (LSM303): tilt, heading, hump height, lift detection | `app_program/buddy4_imu/` |
| 5 | 2502046 | Obstacles: ultrasonic sensor, servo scan, bypass | `app_program/buddy5_obstacle/` |

Shared code lives in `app_program/platform/` (kernel objects, timing, bit
packing, fixed-point maths, live settings, buzzer) and
`app_program/integration/` (vehicle state machine, barcode map, debug monitor).

---

## 2. Hardware and wiring

| Part | Model |
|---|---|
| Microcontroller | Raspberry Pi Pico W (RP2040 + CYW43439 Wi-Fi) |
| Carrier board | Cytron Robo Pico (motor driver, servo headers, Grove ports, button, buzzer) |
| Line / barcode sensors | 3 × TCRT5000 IR modules |
| IMU | GY-511 (LSM303DLHC accelerometer + magnetometer) |
| Distance sensor | HC-SR04 ultrasonic, on a scanning servo |
| Drive | 2 × DC gear motors with wheel encoders |

All pins are defined once, in `app_program/platform/robot_cfg.h`. This table
matches that file.

| Device | Signal | GPIO | Notes |
|---|---|---|---|
| LSM303DLHC (IMU) | SDA / SCL | GP0 / GP1 | I2C0, 400 kHz |
| Left IR sensor | DO | GP3 | Digital. Its AO pin (GP2) is not ADC-capable and is left unused |
| Centre IR sensor | AO | GP27 (ADC1) | Analogue, used for line offset |
| Barcode IR sensor | DO | GP28 | Digital (`BARCODE_SENSOR_ANALOGUE 0u`); its AO is parked on GP7 |
| HC-SR04 | ECHO / TRIG | GP4 / GP5 | ECHO through a 5 V → 3.3 V divider |
| Left wheel encoder | A / (B) | GP6 / (GP26) | One channel counted (`ENC_QUADRATURE 0u`) |
| Right wheel encoder | A / (B) | GP16 / (GP17) | One channel counted |
| Left motor | M1A / M1B | GP8 / GP9 | Robo Pico motor driver |
| Right motor | M2A / M2B | GP10 / GP11 | Robo Pico motor driver |
| Scan servo | PWM | GP15 | 50 Hz, on its own PWM slice |
| START button | | GP20 | On-board, active low |
| Buzzer | PWM | GP22 | On-board piezo; the board's mute switch silences it |

---

## 3. Repository layout

```
app_program/              Robot application (all team code is here)
  app_main.c              Start-up: creates the kernel objects and tasks
  platform/               Shared: robot_cfg.h (pins, tuning), robot_ipc,
                          period_tick, gpio_irq, tune, buzzer, fix_math
  buddy1_comms/           Wi-Fi/MQTT link, telemetry, binary frame codec
  buddy2_motion/          Motor driver, encoders, PID, motion commands
  buddy3_line/            IR sampling, line following, barcode + Code 39
  buddy4_imu/             LSM303 driver, tilt/heading, hump and lift detection
  buddy5_obstacle/        Ultrasonic, servo, obstacle scan and width profile
  integration/            Vehicle Control state machine, barcode map,
                          USB debug monitor, combined bench test
build_make/               Makefiles; run make here
config/                   Kernel config and *.example.h templates for local
                          settings (Wi-Fi credentials, network)
docs/                     Protocol, setup, coding-standard and port docs
tests/                    Host unit tests (run on a PC, no hardware needed)
tools/                    PC tools: dashboard, MQTT decoder, Barr-C checker,
                          Mosquitto config, elf2uf2 converter
kernel/ lib/ include/
device/ etc/ templates/   micro T-Kernel 3.0 and the RP2040 port (upstream)
OPEN_DASHBOARD.bat        Windows: start the broker and open the dashboard
```

---

## 4. Software design

### 4.1 Tasks

Priority 1 is the highest. The port's USB-CDC service runs at 3 and the CYW43
radio task at 4, so every control and sensing task runs above the network and
Wi-Fi traffic can never delay control.

| Task | Owner | Priority | Released by | Stack |
|---|---|---|---|---|
| Vehicle Control | Integration | 1 | Event flag (with timeout) | 2 KiB |
| Barcode | Buddy 3 | 1 | Woken by the GPIO interrupt | 2 KiB |
| Encoder | Buddy 2 | 2 | Every 10 ms | 1 KiB |
| Line Following | Buddy 3 | 2 | Every 10 ms | 1 KiB |
| Motion Control | Buddy 2 | 2 | Every 10 ms | 2 KiB |
| Obstacle | Buddy 5 | 2 | 60 ms monitor, or a scan request | 2 KiB |
| (USB-CDC console) | port | 3 | | |
| Communication (CYW43 radio + MQTT poll) | port + Buddy 1 | 4 | Radio poll | |
| IMU | Buddy 4 | 5 | Every 20 ms | 2 KiB |
| Telemetry | Buddy 1 | 8 | Every 200 ms (5 Hz) | 2 KiB |
| Buzzer | Platform | 9 | Polls cue requests | 1 KiB |

Periodic tasks are released by a single 10 ms cyclic handler
(`platform/period_tick.c`), so their periods do not drift. If a task overruns
its period, the missed release is counted and reported in telemetry.

All stacks and buffers are statically allocated; nothing uses the heap.

### 4.2 Inter-task communication

Tasks never share variables directly. They use kernel objects created in
`platform/robot_ipc.c`:

* **Event flags** carry events to Vehicle Control (line lost, junction,
  obstacle, hump, lift, command received), faults, and scan requests.
* **Message buffers** carry data: barcode actions, obstacle profiles, PC
  commands, motion commands, telemetry events and outgoing MQTT frames.
* **A mutex** with priority inheritance protects the shared encoder state.
* **Packed 32-bit status words** (`sys_status`), each with exactly one writer,
  let Telemetry and Vehicle Control read every subsystem's latest state
  without locking.

The RP2040 has one interrupt for all GPIO pins. `platform/gpio_irq.c` owns it,
timestamps each edge and calls the handler for that pin (encoders, barcode
edges, ultrasonic echo). Handlers only record data and wake a task.

### 4.3 Vehicle states

Defined in `integration/vehicle.h` and run by `integration/vehicle.c`.

| State | What happens |
|---|---|
| INITIALISING | 1.5 s settle while the IMU learns its level position. Keep the robot still |
| READY | Motors off. START (button or MQTT `start`) begins a run |
| CALIBRATING | Sweeps 25° each side across the line to learn the black and white IR levels |
| LINE_FOLLOWING | PID + steering on the line. A decoded barcode is turned into a junction action and latched |
| JUNCTION_ACTION | Drives onto the junction, then turns left, right, U-turns or goes straight (no barcode = straight) |
| OBSTACLE_SCAN | Stops about 15 cm from the obstacle and scans it with the servo to measure its width |
| BYPASS | Backs up, steps out to the clearer side, passes the obstacle and turns back to find the line |
| REVERSE_RETRY | Neither side was clear: reverse and scan again (at most twice) |
| LINE_REACQUIRE | Line lost for no known reason: left-right sweep; nothing within 5 s means SAFE_STOP |
| SAFE_STOP | Motors off after a fault (impact, stall, sensor fault, lift, calibration failure, timeout). START or `reset` returns to READY |
| MISSION_COMPLETE | Run ended by START or `stop`. A summary (highest hump, barcodes, obstacles, distance) is published |

Losing Wi-Fi or the broker never changes the robot's state: it carries on and
reconnects in the background.

---

## 5. Building

A full step-by-step setup, including installing the tools and checking new
hardware, is in [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md).

### 5.1 What you need

* Arm GNU toolchain (`arm-none-eabi-gcc`)
* GNU make: `mingw32-make` from MSYS2 on Windows, or `make` on Linux/WSL
* A host C++ compiler (`g++`) to build the `elf2uf2` converter once
* Pico SDK 2.2.0 (for USB console, Wi-Fi and lwIP)
* Python 3 (the Barr-C check runs before every build)

### 5.2 Local settings (never commit these)

```
cp config/wifi_credentials.example.h config/wifi_credentials.h
```

Edit `config/wifi_credentials.h`: set the SSID, password and the PC's IP
address (`MQTT_BROKER_IP`), then set `UTK_WIFI_CREDENTIALS_SET` to `1`. The
file is in `.gitignore`. The Pico W supports 2.4 GHz Wi-Fi only.

### 5.3 Build commands

From `build_make/` (use `mingw32-make` on Windows):

```
# Full robot: USB console, Wi-Fi, DHCP, MQTT, debug monitor
make SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
     MQTT=1 DEBUG_MON=1 PICO_SDK_PATH=<path to pico-sdk>

# Without the radio (everything except Buddy 1's network)
make SMP=0 CONSOLE=usb_cdc DEBUG_MON=1 PICO_SDK_PATH=<path to pico-sdk>
```

The output is a `.uf2` file in `build_make/`, named after the build profile.

### 5.4 Bench builds (one subsystem at a time)

Add `BUDDY_TEST=<n>` to a build command to replace the full mission with a
stand-alone test:

| `BUDDY_TEST` | Runs |
|---|---|
| `0` (default) | Full mission |
| `2` | Buddy 2: motors and encoders only |
| `3` | Buddy 3: IR sensors, line and barcode reading only |
| `4` | Buddy 4: IMU diagnostic (motors held off) |
| `23` | Buddy 2 + Buddy 3: line following without the rest |

---

## 6. Flashing and running

1. Hold **BOOTSEL** on the Pico and plug in USB. A drive called `RPI-RP2`
   appears.
2. Copy the `.uf2` onto it. The Pico restarts by itself.
3. Start the broker and dashboard on the PC (section 7). The PC must be on the
   same network as the robot.
4. Wait for **ROBOT ONLINE** on the dashboard. Check that SAFETY shows
   **LIFT STOP ARMED**.
5. Press **M** and enter the barcode letters for FORWARD, U-TURN, LEFT and
   RIGHT. The map is lost when the robot is switched off, so send it again
   after every power-up.
6. Place the robot on the line and press **START** (button or **S**).

Over USB, any serial terminal shows the start-up result
(`boot failures 0x0000` means every subsystem started) and, with
`DEBUG_MON=1`, a live status line.

---

## 7. PC tools and MQTT

Install once: Mosquitto and `pip install paho-mqtt`.

**Windows:** double-click `OPEN_DASHBOARD.bat`. It starts Mosquitto and opens
the dashboard.

**Any OS:**

```
mosquitto -c tools/mosquitto_robot.conf -v
python3 tools/robot_dashboard.py --broker <PC IP>
```

Dashboard keys:

| Key | Action |
|---|---|
| S / X / R | Start / stop / reset |
| U | Request a run summary |
| W | Scan the obstacle in front and show its width (robot stopped) |
| M | Send the barcode map |
| Q | Quit |

Every decoded frame is saved to `tools/logs/`.

`tools/tlm_decode.py` is the command-line version:

```
python3 tools/tlm_decode.py monitor --broker <PC IP>
python3 tools/tlm_decode.py send start --broker <PC IP>
python3 tools/tlm_decode.py send map C right --version 3 --broker <PC IP>
```

Topics are under `inf2004/aai03/`: `tlm` (telemetry, 5 Hz), `hb`
(heartbeat), `evt` (events), `sum` (summary), `ack` (command
acknowledgements) and `cmd` (commands to the robot). Frames are compact
binary, packed bit by bit. The full format is in
[docs/MQTT_PROTOCOL.md](docs/MQTT_PROTOCOL.md).

---

## 8. Testing

### Host unit tests

The hardware-independent modules (bit packing, fixed-point maths, telemetry
codec, PID, Code 39 decoding, hump and lift detection, obstacle profiling,
barcode map) are tested on a PC with no hardware:

```
make -C tests
```

This builds and runs `robot_tests` (154 checks) and checks that the Python
decoder agrees with the C encoder.

### Barr-C check

`tools/barr_check.py` runs automatically before every firmware build and
stops the build on a violation. To run it by hand:

```
python3 tools/barr_check.py app_program
```

The rules it enforces and the deliberate deviations are listed in
[docs/BARR_C_COMPLIANCE.md](docs/BARR_C_COMPLIANCE.md).

---

## 9. Tuning

Every tuning value (speeds, PID gains, steering limits, thresholds, bypass
distances) is in `app_program/platform/robot_cfg.h`, grouped by buddy.

Some values can be changed live over MQTT without reflashing, for example
encoder scale, turn sizes, sonar beam correction and obstacle detection
distance. Live changes are lost on reset; copy good values back into
`robot_cfg.h`. The list of live settings is in
[docs/MQTT_PROTOCOL.md](docs/MQTT_PROTOCOL.md#live-tuning).

---

## 10. Further documentation

| Document | Contents |
|---|---|
| [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) | Step by step: install tools, build, flash, hardware bring-up, first run, troubleshooting |
| [docs/MQTT_PROTOCOL.md](docs/MQTT_PROTOCOL.md) | Topics, frame layouts, commands, live tuning |
| [docs/MQTT_SETUP.md](docs/MQTT_SETUP.md) | Installing and running the broker; connection troubleshooting |
| [docs/BARR_C_COMPLIANCE.md](docs/BARR_C_COMPLIANCE.md) | How Barr-C is enforced, and deviations |
| [docs/ABBREVIATIONS.md](docs/ABBREVIATIONS.md) | Project abbreviation table (Barr-C rule 1.5.b) |
| [README_MTKERNEL.md](README_MTKERNEL.md) | The upstream micro T-Kernel RP2040 port |
| [docs/PORT_RP2040.md](docs/PORT_RP2040.md) | What the port supports and has validated |

---

## 11. Credits and licences

* micro T-Kernel 3.0 by TRON Forum, RP2040 port by
  [sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040).
  The kernel and port are used with only small board-specific changes; all
  robot code is in `app_program/`.
* Raspberry Pi Pico SDK (BSD-3-Clause).
* Licence texts are in `LICENSES/`. micro T-Kernel is distributed under
  T-License 2.x; `ucode.png` is its required traceability registration mark.