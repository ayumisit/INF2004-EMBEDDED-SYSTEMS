# Project abbreviations (Barr-C rule 1.5.b)

Abbreviations from Barr-C Appendix A (adc, avg, b_, buf, cfg, curr, err, g_,
gpio, h_, init, io, isr, led, max, min, msec, msg, num, p_, pp_, prev, prio,
pwm, q, reg, rx, sem, str, sync, tmp, tx, usec) are used with their standard
meaning.  The project adds:

| Abbreviation | Meaning |
|---|---|
| ack | acknowledgement |
| addr | address (I2C device address) |
| ao / do | analogue output / digital output of an IR module |
| arg | argument |
| calib | calibration |
| cm, mm, um | centimetre, millimetre, micrometre (units in names) |
| cmd | command |
| ctrl | control |
| ctx | context (a module's working state) |
| dbg | debug |
| deg, mdeg | degree, millidegree |
| diag | diagnostic |
| dist | distance |
| enc | wheel encoder |
| env | envelope |
| evt | event |
| ff | feed-forward |
| flg | micro T-Kernel event flag |
| freq | frequency |
| hb | heartbeat |
| hw | hardware (module suffix `_hw`: uses the Pico SDK) |
| hz | hertz (units in names) |
| idx | index |
| imu | inertial measurement unit (LSM303D or LSM303DLHC) |
| ipc | inter-task communication |
| ir | infrared |
| irq | interrupt request |
| lwt | MQTT last will and testament |
| mag | magnitude; also magnetometer (lsm303_hw_read_mag) |
| mbf | micro T-Kernel message buffer |
| med | median |
| mg | milli-g (acceleration) |
| mon | monitor |
| ms, us | millisecond, microsecond (units in names) |
| mtx | micro T-Kernel mutex |
| nav | navigation |
| net | network (module suffix `_net`: uses lwIP and the radio) |
| norm | normalised (a reading scaled to a fixed range) |
| obs | obstacle |
| ovf | overflow |
| pid | proportional-integral-derivative controller |
| pos | position |
| q15 | fixed-point number with 15 fractional bits (1.0 = 32768) |
| sdk | Raspberry Pi Pico SDK |
| seq | sequence number |
| sq | square |
| tlm | telemetry |
