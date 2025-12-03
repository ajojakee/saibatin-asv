#!/usr/bin/env python3
"""
Set Pixhawk to GUIDED mode for MAVLink control
"""
from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
conn.wait_heartbeat()
print(f"✅ Connected (System {conn.target_system})")

# Set mode to GUIDED (mode 15 untuk ArduRover/ArduBoat)
print("\nSetting flight mode to GUIDED...")
conn.mav.set_mode_send(
    conn.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    15  # GUIDED mode
)

# Wait for mode change
time.sleep(1)

# Verify mode
msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if msg:
    mode = msg.custom_mode
    mode_names = {15: 'GUIDED', 0: 'MANUAL', 10: 'AUTO', 4: 'HOLD'}
    print(f"✅ Current mode: {mode_names.get(mode, f'UNKNOWN({mode})')}")

# ARM Pixhawk
print("\n🔓 Arming Pixhawk...")
conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if msg and msg.result == 0:
    print("✅ ARMED successfully")
else:
    print("⚠️ ARM failed (may already be armed)")

print("\n✅ Ready for motor control!")
print("Run: python3 test_motor.py")
