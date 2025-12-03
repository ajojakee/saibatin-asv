#!/usr/bin/env python3
"""
Enable RC Override on Pixhawk
"""
from pymavlink import mavutil

print("Connecting to Pixhawk...")
connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
connection.wait_heartbeat()
print(f"✅ Connected")

# Set SYSID_MYGCS = 255 (allow MAVLink control)
print("\nSetting SYSID_MYGCS = 255...")
connection.mav.param_set_send(
    connection.target_system,
    connection.target_component,
    b'SYSID_MYGCS',
    255,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)
print("✅ RC Override enabled")

# Disable safety switch
print("\nDisabling safety switch...")
connection.mav.param_set_send(
    connection.target_system,
    connection.target_component,
    b'BRD_SAFETY_DEFLT',
    0,
    mavutil.mavlink.MAV_PARAM_TYPE_INT8
)
print("✅ Safety disabled")

print("\n⚠️ Reboot Pixhawk untuk apply perubahan!")
