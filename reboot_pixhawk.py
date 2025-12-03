#!/usr/bin/env python3
"""
Reboot Pixhawk via MAVLink command
"""
from pymavlink import mavutil
import time

print("Connecting to Pixhawk...")
conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
conn.wait_heartbeat()
print(f"✅ Connected (System {conn.target_system})")

print("\n⚠️ Rebooting Pixhawk in 3 seconds...")
time.sleep(3)

# Send reboot command
conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    0,  # confirmation
    1,  # param1: 1=reboot autopilot
    0, 0, 0, 0, 0, 0
)

print("✅ Reboot command sent!")
print("\nWaiting 30 seconds for Pixhawk to reboot...")
time.sleep(30)

print("\nReconnecting to Pixhawk...")
try:
    conn2 = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    conn2.wait_heartbeat(timeout=10)
    print("✅ Pixhawk rebooted successfully!")
except Exception as e:
    print(f"⚠️ Reconnect failed: {e}")
    print("   Try manual power cycle if needed")
