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
