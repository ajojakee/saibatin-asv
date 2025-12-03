#!/usr/bin/env python3
"""
Debug: Check if Pixhawk actually outputs PWM
Monitor SERVO_OUTPUT_RAW messages
"""
import time
from pymavlink import mavutil

print("Connecting...")
conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
conn.wait_heartbeat()
print("✅ Connected\n")

print("Sending PWM 1700 to Channel 4...")
conn.mav.rc_channels_override_send(
    conn.target_system,
    conn.target_component,
    0, 0, 0, 1700, 0, 0, 0, 0
)

print("\nMonitoring SERVO_OUTPUT_RAW for 10 seconds...")
print("(Check if servo4_raw changes to 1700)\n")

start = time.time()
while time.time() - start < 10:
    msg = conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"Servo1: {msg.servo1_raw} | Servo2: {msg.servo2_raw} | "
              f"Servo3: {msg.servo3_raw} | Servo4: {msg.servo4_raw} | "
              f"Servo5: {msg.servo5_raw}")
        time.sleep(0.5)

print("\n✅ Done")
