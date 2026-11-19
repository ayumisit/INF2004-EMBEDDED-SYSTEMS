@echo off
rem AAI03 robot: start the MQTT broker (if needed) and open the live dashboard.
rem Double-click this file.  Your PC must be on the same hotspot as the robot
rem and have the address in config/wifi_credentials.h (MQTT_BROKER_IP).
cd /d "%~dp0"
title AAI03 Robot Dashboard

netstat -an | findstr /r /c:":1883 .*LISTENING" >nul
if errorlevel 1 (
    echo Starting the Mosquitto broker in its own window...
    start "Mosquitto broker - leave open" /min "C:\Program Files\mosquitto\mosquitto.exe" -c "%~dp0tools\mosquitto_robot.conf" -v
    timeout /t 2 /nobreak >nul
) else (
    echo Mosquitto broker already running.
)

mode con: cols=100 lines=46 >nul 2>&1
python "%~dp0tools\robot_dashboard.py" --broker 172.20.10.3
if errorlevel 1 pause
