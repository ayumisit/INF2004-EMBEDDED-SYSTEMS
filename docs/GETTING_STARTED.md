# Getting Started: AAI03 robot car

From a clean PC to the robot running a course: install the tools, build the
firmware, flash it, check the hardware and start a run.

For what the robot does and how the code is organised, see the
[README](../README.md). The wiring is in
[README section 2](../README.md#2-hardware-and-wiring).

---

## 1. What you need

### Hardware

| Item | Notes |
|---|---|
| Assembled robot | Pico W on a Cytron Robo Pico, wired as in [README section 2](../README.md#2-hardware-and-wiring) |
| USB cable with data lines | Charge-only cables will not show the `RPI-RP2` drive |
| PC on the same 2.4 GHz network as the robot | A phone hotspot works. The Pico W cannot use 5 GHz |

### Software

| Tool | Why |
|---|---|
| Arm GNU toolchain (`arm-none-eabi-gcc`) | Compiles the firmware |
| GNU make | The build system (`mingw32-make` on Windows) |
| A host C++ compiler (`g++`) | Builds the `elf2uf2` converter once, on the first build |
| Pico SDK **2.2.0** | Hardware drivers, USB console, Wi-Fi driver and lwIP |
| Python 3 | Runs the Barr-C check before every build, and the PC tools |
| Mosquitto + `paho-mqtt` | MQTT broker and Python client for the dashboard |

---

## 2. Install the tools

### Windows

1. Install the **Arm GNU Toolchain** (arm-none-eabi) from Arm's website and
   tick "Add path to environment variable".
2. Install **MSYS2**, then in the MSYS2 MinGW shell:

   ```sh
   pacman -S mingw-w64-x86_64-make mingw-w64-x86_64-gcc git
   ```

   Add `C:\msys64\mingw64\bin` to your `PATH` so `mingw32-make` and `g++` work
   from any terminal.
3. Install **Python 3** (tick "Add to PATH") and **Mosquitto** (default
   folder `C:\Program Files\mosquitto`).
4. `python -m pip install paho-mqtt`

### Ubuntu / WSL

```sh
sudo apt install gcc-arm-none-eabi build-essential git python3 python3-pip mosquitto
python3 -m pip install --user paho-mqtt
```

Under WSL the robot cannot reach a broker running inside WSL by default. Run
Mosquitto on Windows instead, or forward port 1883.

### Check

```sh
arm-none-eabi-gcc --version
g++ --version
python3 --version
```

---

## 3. Install the Pico SDK

The build compiles a few SDK sources and headers directly; it does not use the
SDK's CMake build. Only three submodules are needed.

```sh
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
cd ~/pico/pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

Always pass `PICO_SDK_PATH` to make (or export it). The makefile's fallback
default is a relative path that will not exist on your machine.

---

## 4. Get the code and set up local settings

```sh
git clone https://github.com/ayumisit/INF2004-EMBEDDED-SYSTEMS.git
cd INF2004-EMBEDDED-SYSTEMS
cp config/wifi_credentials.example.h config/wifi_credentials.h
```

Edit `config/wifi_credentials.h`:

| Setting | Set it to |
|---|---|
| `UTK_WIFI_SSID` | Your network or hotspot name |
| `UTK_WIFI_PASSWORD` | Its password |
| `MQTT_BROKER_IP` | The IP address of the PC that will run Mosquitto |
| `UTK_WIFI_CREDENTIALS_SET` | `1` (the build refuses to run while it is `0`) |

Find the PC's address with `ipconfig` (Windows), `hostname -I` (Linux) or
`ipconfig getifaddr en0` (macOS).

> [!WARNING]
> `config/wifi_credentials.h` is in `.gitignore`. Never commit it, and never
> put real values in `wifi_credentials.example.h`.

---

## 5. Build

From `build_make/` (use `mingw32-make` instead of `make` on Windows):

```sh
cd build_make
make SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 \
     MQTT=1 DEBUG_MON=1 PICO_SDK_PATH=~/pico/pico-sdk -j8
```

This is the full robot: USB console, Wi-Fi, MQTT and the debug monitor. It
produces `mtk3pico_smp0_usb_cdc_wifi_dhcp_mqtt.uf2` in `build_make/`.

What happens during a build:

1. **Barr-C check.** `tools/barr_check.py` scans `app_program/` first. Any
   violation stops the build with the file and line.
2. **Compile.** All robot code is built with `-Werror`, so a warning stops the
   build too.
3. **UF2 packaging.** On the first build, `tools/elf2uf2` is compiled with your
   host `g++` (you will see `Host tool: tools/elf2uf2`). After that, every
   link ends by converting the `.elf` to a `.uf2`.

The robot only builds as **single core with the USB console**. Other settings
stop with an error:

| Error | Cause |
|---|---|
| `The AAI03 robot uses GP0/GP1 for I2C: build with CONSOLE=usb_cdc` | The UART console would take the IMU's pins |
| `The AAI03 robot application is single-core: build with SMP=0` | The shared-data design assumes one core |
| `The robot application needs the Pico SDK. Set PICO_SDK_PATH` | SDK not found |

Switching between build settings needs no `make clean`: the build notices and
discards objects from the old settings. The exception is `DEBUG_MON`: run
`make clean` after changing it, because `barcode.c` reads it but is not
rebuilt.

### Smaller builds

| Build | Add or change |
|---|---|
| Without Wi-Fi/MQTT | Drop `WIFI=...`, `WIFI_*` and `MQTT=1` |
| One subsystem only | Add `BUDDY_TEST=2` (motors/encoders), `3` (IR, line, barcode), `4` (IMU diagnostic) or `23` (line following with motors) |

Bench builds print their results on the USB console and do not need the PC
tools.

---

## 6. Flash

1. Hold **BOOTSEL** on the Pico W and plug in the USB cable.
2. Release it. A drive called `RPI-RP2` appears.
3. Copy the `.uf2` from `build_make/` onto the drive.

The Pico restarts into the new firmware as soon as the copy finishes.

---

## 7. First power-on checks

### USB console

Open the Pico's USB serial port with any terminal (PuTTY or the Arduino
Serial Monitor on Windows, `screen /dev/ttyACM0` on Linux). Output appears a
second or two after the port opens, not at reset.

The first line to look for is:

```
AAI03 robot: boot failures 0x0000
```

`0x0000` means every subsystem started. Any set bit names the one that failed:

| Bit | Subsystem | Bit | Subsystem |
|---|---|---|---|
| `0x0001` | Kernel objects (IPC) | `0x0040` | IMU |
| `0x0002` | Communication | `0x0080` | Obstacle |
| `0x0004` | Telemetry | `0x0100` | Vehicle Control |
| `0x0008` | Motion | `0x0200` | GPIO interrupt |
| `0x0010` | Line following | `0x0400` | Period tick |
| `0x0020` | Barcode | `0x0800` | Bench test task or debug monitor |
| | | `0x1000` | Buzzer |

With `DEBUG_MON=1`, a status line follows with the vehicle state, wheel
speeds, line, IMU, obstacle and faults.

### GP14

The kernel's start-up code sets **GP14** as an output for the port's
liveness LED (`BOARD_LED_PIN` in
`include/sys/sysdepend/pico_rp2040/sysdef.h`). The robot firmware does not
blink it, but keep that servo header empty so nothing else is driven by it.

---

## 8. Hardware bring-up checklist

Do this once on a newly assembled robot, or after re-wiring. Every setting
named here is in `app_program/platform/robot_cfg.h`.

1. **Ultrasonic ECHO voltage.** If the HC-SR04 runs at 5 V, fit the
   5 V → 3.3 V divider on ECHO (GP4) before powering up.
2. **Ultrasonic TRIG/ECHO.** Wave a hand in front of the sensor and watch the
   obstacle distance. If it never changes, swap `PIN_SONAR_ECHO` and
   `PIN_SONAR_TRIG`.
3. **Motor direction.** Lift the wheels and press START. Calibration turns
   left first, so the left wheel should run backwards and the right wheel
   forwards. If one wheel is wrong, set `MOTOR_LEFT_REVERSED` or
   `MOTOR_RIGHT_REVERSED`. If the wheels are swapped, swap the left and right
   motor pin pairs.
4. **Encoders.** Push the robot 1 m by hand and compare the distance shown;
   adjust `ENC_UM_PER_EDGE`. Turn it 360° on the spot and adjust
   `TRACK_WIDTH_MM`.
5. **IR polarity.** Hold each IR sensor over black, then white, and watch the
   line reading. If it is inverted, change `IR_DO_BLACK_IS_HIGH` (left and
   barcode sensors) or `IR_AO_BLACK_IS_HIGH` (centre sensor).
6. **Barcode sensor.** Adjust the module's potentiometer so its LED flickers
   once per bar as a barcode passes under it.
7. **IMU axis.** With the robot flat, the monitor's `tilt` must read about 0;
   tilt the nose up about 30 degrees and it must read about 30. `tilt` is
   pitch and roll combined and is never negative. If it does not follow,
   check `IMU_FORWARD_AXIS` / `IMU_FORWARD_SIGN`.
8. **Servo direction.** During a scan, 150° must point left. Otherwise set
   `SERVO_LEFT_IS_HIGH` to `0u`.
9. **Tune on the track.** Speeds (`SPEED_*`), PID (`PID_*`) and steering
   (`STEER_*`) first, then the hump, obstacle and bypass thresholds. Several
   of these can be changed live over MQTT (section 9).

---

## 9. Run the robot

### Start the broker and dashboard

**Windows:** double-click `OPEN_DASHBOARD.bat` in the repository root. It
starts Mosquitto in its own window (leave it open) and opens the dashboard.
The broker address is written in the `.bat` file; change it if your PC's IP
address is different.

**Any OS:**

```sh
mosquitto -c tools/mosquitto_robot.conf -v
python3 tools/robot_dashboard.py --broker <PC IP>
```

Allow incoming connections on port 1883 if the firewall asks.

### Before each run

1. Wait for **ROBOT ONLINE** at the top of the dashboard.
2. Check SAFETY shows **LIFT STOP ARMED**. If it is red, switch the robot off,
   plug in USB, switch it on, then unplug.
3. Press **M** and type the barcode letters for FORWARD, U-TURN, LEFT and
   RIGHT. Each letter should get a "Barcode Mapping Changed ... Applied: yes"
   event. The map is lost when the robot is switched off, so send it again
   after every power-up.
4. Put the robot on the line, keep it still, and press **S** (or the START
   button).

### Dashboard keys

Click the dashboard window first. The keys work on Windows only; on other
systems send commands with `tools/tlm_decode.py send ...`.

| Key | Action |
|---|---|
| S | Start a run |
| X | Stop the run and publish a summary |
| R | Reset any stopped state (also INITIALISING, ending its settle early) to READY |
| U | Request a summary |
| W | Scan the obstacle in front and show its width (robot stopped) |
| M | Send the barcode map |
| Q | Quit |

Every decoded frame is saved to `tools/logs/`. The frame formats and the full
command list are in [MQTT_PROTOCOL.md](MQTT_PROTOCOL.md); broker setup details
are in [MQTT_SETUP.md](MQTT_SETUP.md).

---

## 10. Tests

Host unit tests need only `gcc` and Python, no robot:

```sh
make -C tests
```

Expected result: `154 checks, 0 failures`, followed by the Python decoder
cross-check.

To run the Barr-C check without building:

```sh
python3 tools/barr_check.py app_program
```

Expected result: `Barr-C scan: clean`.

---

## 11. Troubleshooting

### `RPI-RP2` drive does not appear

Hold BOOTSEL **before** plugging in, and keep holding until the drive mounts.
Try another cable; charge-only cables have no data lines.

### `make` succeeded but there is no `.uf2`

The host tool `build_make/tools/elf2uf2` failed to build. Look further up for
a `g++` error and check `g++ --version`. Set `HOSTCXX` if your host compiler
has a different name. The `.elf` is fine; only the packaging was skipped.

### `TinyUSB not found` / `CYW43 driver not found` / `lwIP not found`

`PICO_SDK_PATH` is wrong, or a submodule was not fetched:

```sh
cd <pico-sdk> && git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

### `config/wifi_credentials.h: placeholder values`

Fill in the file (section 4) and set `UTK_WIFI_CREDENTIALS_SET` to `1`.

### Nothing on the USB console

Output starts only after the PC opens the port. Open it, then wait a second
or two. If no serial port appears at all, try another cable or USB port, and
check that the build used `CONSOLE=usb_cdc`.

### Dashboard never shows ROBOT ONLINE

* The robot and PC must be on the same network, and it must be 2.4 GHz.
* `MQTT_BROKER_IP` in `wifi_credentials.h` must be the PC's current address.
  Hotspot addresses can change, so check again and rebuild if needed.
* Mosquitto must be running with `tools/mosquitto_robot.conf` (it listens on
  all interfaces), and the firewall must allow port 1883.

More connection checks are in [MQTT_SETUP.md](MQTT_SETUP.md).

---

## 12. The bare kernel port (reference only)

The robot runs on the micro T-Kernel 3.0 RP2040 port by
[sirfonzie](https://github.com/sirfonzie/mtk3smp-rp2040). The port's own
material is kept for reference and is **not** part of the robot build:

* [README_MTKERNEL.md](../README_MTKERNEL.md): the port, its dual-core (SMP)
  support and its demo.
* [PORT_RP2040.md](PORT_RP2040.md): what the port has validated, and the
  RP2040 safety rules.
* `app_program/demo_tasks.c/.h`: the port's demo liveness-LED task. The
  robot makefile leaves it out of the build.
* `tools/linux_echo.py`, `tools/windows_udp_echo.ps1`,
  `tools/windows_tcp_echo.ps1`, `tools/verify_phase11_usb.py` and
  `config/udp_test_config.example.h` / `tcp_test_config.example.h`: the
  port's UDP/TCP echo tests and console check, used when bringing up the
  network stack before MQTT was added.
* The micro T-Kernel 3.0 specification: https://tron-forum.github.io/mtkernel_3/