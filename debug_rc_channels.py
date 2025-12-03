#!/usr/bin/env python3
"""
Debug: Monitor RC Channels real-time
See if PWM commands actually reach Pixhawk
"""

import time
from pymavlink import mavutil

print("Connecting...")
connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
connection.wait_heartbeat()
print("✅ Connected\n")

print("Sending test PWM to Channel 4 (1700)...")
connection.mav.rc_channels_override_send(
    connection.target_system,
    connection.target_component,
    0, 0, 0, 1700, 0, 0, 0, 0
)

print("\nMonitoring RC_CHANNELS for 10 seconds...\n")
start = time.time()

while time.time() - start < 10:
    msg = connection.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
    if msg:
        print(f"Chan1: {msg.chan1_raw} | Chan2: {msg.chan2_raw} | Chan3: {msg.chan3_raw} | Chan4: {msg.chan4_raw} | Chan5: {msg.chan5_raw}")
        time.sleep(0.5)

print("\nStopping (PWM 1500)...")
connection.mav.rc_channels_override_send(
    connection.target_system,
    connection.target_component,
    0, 0, 0, 1500, 0, 0, 0, 0
)
print("✅ Done")
