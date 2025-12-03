#!/usr/bin/env python3
"""
Force Pixhawk to GUIDED mode + ARM + Test Motor
"""
from pymavlink import mavutil
import time

print("=" * 60)
print("   FORCE GUIDED MODE + ARM + MOTOR TEST")
print("=" * 60)

# Connect
print("\n1. Connecting to Pixhawk...")
conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
conn.wait_heartbeat()
print(f"✅ Connected (System {conn.target_system})")

# Set GUIDED mode (mode 15 for Rover)
print("\n2. Setting GUIDED mode...")
for i in range(3):  # Retry 3x
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        15,  # GUIDED mode
        0, 0, 0, 0, 0
    )
    time.sleep(1)
    
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if msg and msg.custom_mode == 15:
        print("✅ GUIDED mode activated")
        break
else:
    print("⚠️ Mode change uncertain, continuing...")

# ARM
print("\n3. Arming Pixhawk...")
conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
time.sleep(2)
print("✅ ARM command sent")

# Send PWM command to motor (Channel 4 = 1700)
print("\n4. Sending motor command (PWM 1700 to Channel 4)...")
for i in range(10):  # Send 10x untuk ensure
    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
        0, 0, 0, 1700, 0, 0, 0, 0
    )
    time.sleep(0.1)

print("✅ Motor command sent (10x)")

# Monitor servo output
print("\n5. Monitoring SERVO_OUTPUT_RAW (5 seconds)...")
print("   Check if servo4_raw = 1700\n")

start = time.time()
while time.time() - start < 5:
    msg = conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)
    if msg:
        print(f"Servo1: {msg.servo1_raw} | Servo2: {msg.servo2_raw} | "
              f"Servo3: {msg.servo3_raw} | Servo4: {msg.servo4_raw} | "
              f"Servo5: {msg.servo5_raw}")
        
        if msg.servo4_raw > 1600:
            print("\n🎉 SUCCESS! Motor responding!")
            break
        time.sleep(0.5)

# Stop motor
print("\n6. Stopping motor (PWM 1500)...")
conn.mav.rc_channels_override_send(
    conn.target_system,
    conn.target_component,
    0, 0, 0, 1500, 0, 0, 0, 0
)

print("\n" + "=" * 60)
print("If servo4_raw changed to 1700: ✅ MOTOR WORKING!")
print("If servo4_raw still 800: ❌ Check ESC/wiring")
print("=" * 60)
