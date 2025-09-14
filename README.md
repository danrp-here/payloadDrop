# Payload Dropping using ESP32

This repo includes:
- **ESP32 firmware** (Arduino) — IMU (MPU6050) + GPS + 4× servos + MAVLink on UART2 (to SiK 433 MHz air radio).
- **ROS 2 ground package** — bridge + guidance + PID + *both* 2‑servo and 4‑servo mixers.
- Default **launch** uses the **4‑servo** mixer. A 2‑servo launch is also provided.

## Build steps (ROS 2)
```bash
# From your ROS 2 workspace root 
# Put this folder under src/:  ~/payloadDrop/src/payload_ground_stack
pip install pymavlink
colcon build --packages-select payload_ground_stack
. install/setup.bash

# 4‑servo layout (default):
ros2 launch payload_ground_stack stack.launch.py   conn_url:=/dev/ttyUSB1:57600 payload_sysid:=72   servo1:=1 servo2:=2 servo3:=3 servo4:=4

# 2‑servo elevon layout:
ros2 launch payload_ground_stack stack_2servo.launch.py   conn_url:=/dev/ttyUSB1:57600 payload_sysid:=72   servo_left:=1 servo_right:=2
```


## Wiring recap (payload side)
- ESP32 UART2: GPIO27 (TX2) → SiK RX, GPIO26 (RX2) ← SiK TX, GND↔GND, +5 V (BEC) → SiK VCC.
- GPS on UART1 (GPIO16 RX1 ← GPS TX, GPIO17 TX1 → GPS RX).
- IMU on I2C (GPIO21 SDA, GPIO22 SCL).
- Servos on GPIO 14,15,4,5 (LEDC @ 50 Hz), powered by separate BEC; **common ground**.

## Radios & Antennas (433 MHz)
- Pair of **SiK 433 MHz** radios (air + ground) with **433 MHz whip/dipole** antennas (λ/4 ≈ 17.3 cm).
- Start: **AirSpeed 32 kbps**, **Serial 57600**, **ECC ON**, **NETID unique**, **Tx power per local rules**.

# Quick Commands: Navigate There

After launching the stack, you can command waypoints directly:

## By Latitude/Longitude/Altitude
```bash
ros2 run payload_ground_stack nav_to -- <lat_deg> <lon_deg> <alt_m>
# example:
ros2 run payload_ground_stack nav_to -- -6.8930 107.6100 750
```

## By Local NED (meters, origin = first GPS fix)
```bash
ros2 run payload_ground_stack nav_to_ned -- <north_m> <east_m> <down_m>
# example: 300 m east of origin
ros2 run payload_ground_stack nav_to_ned -- 0 300 0
```

## Hold Here (sets target to current pose)
```bash
ros2 run payload_ground_stack nav_here
```

You can watch the active target on:
- `/state/target_ned` (published by guidance_node)
- `/state/pose_ned` (current NED position)

Tip: Wait until the bridge logs **"NED origin set"** before using `nav_to` (LLA).

## Wiring for ESP 32

```bash
ESP32 DevKitC ←→ SiK Air Radio (433 MHz)
-------------------------------------------
GPIO27 (TX2) → RADIO RX (3.3 V TTL)
GPIO26 (RX2) ← RADIO TX (3.3 V TTL)
GND ↔ GND (common)
+5 V (from BEC) → RADIO VCC (check radio spec; many accept 5–6 V)

GPS module ↔ ESP32 (UART1 remapped)
---------------------------------------------
GPS TX → GPIO16 (ESP32 RX1)
GPS RX (opt) ← GPIO17 (ESP32 TX1)
VCC (3.3/5V) & GND as per module (common GND)


IMU (MPU6050) ↔ ESP32 (I2C)
---------------------------------------------
SDA ↔ GPIO21
SCL ↔ GPIO22
3.3 V & GND (common GND)


Servos (x4) — LEDC PWM @ 50 Hz (separate BEC)
---------------------------------------------
Servo1 signal ← GPIO14
Servo2 signal ← GPIO15
Servo3 signal ← GPIO4
Servo4 signal ← GPIO5
Servos power from BEC (5–6 V), GND tied to ESP32 GND
```