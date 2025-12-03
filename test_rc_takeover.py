#!/usr/bin/env python3
"""
Test RC Takeover - Verify RC dapat override MAVLink
"""
from pymavlink import mavutil
import time

print("=" * 60)
print("   TEST RC TAKEOVER")
print("=" * 60)

# Connect
conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
conn.wait_heartbeat()
print("✅ Connected\n")

print("Test Procedure:")
print("1. Script akan kirim MAVLink command (PWM 1700)")
print("2. Gerakkan RC stick untuk takeover")
print("3. Lepas RC stick, MAVLink resume setelah 3 detik\n")

input("Press ENTER to start test...")

# Send MAVLink override
print("\n1. Sending MAVLink command (PWM 1700 to Channel 4)...")
for i in range(20):  # Send 20x (2 seconds)
    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
        0, 0, 0, 1700, 0, 0, 0, 0
    )
    time.sleep(0.1)

print("✅ MAVLink sending...")
print("\n2. Now move RC throttle stick (emergency takeover)...")
print("   Motor should respond to RC, not MAVLink")
print("   Monitoring for 10 seconds...\n")

# Monitor RC channels
start = time.time()
while time.time() - start < 10:
    msg = conn.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
    if msg:
        print(f"RC Ch3 (Throttle): {msg.chan3_raw} | "
              f"Servo4 (Motor): ", end="")
        
        # Check servo output
        servo_msg = conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
        if servo_msg:
            print(f"{servo_msg.servo4_raw}")
        else:
            print("N/A")
        
        time.sleep(0.5)

# Stop override
print("\n3. Stopping MAVLink override (neutral PWM)...")
conn.mav.rc_channels_override_send(
    conn.target_system,
    conn.target_component,
    0, 0, 0, 1500, 0, 0, 0, 0
)

print("\n" + "=" * 60)
print("✅ Test complete!")
print("=" * 60)
print("\nResults:")
print("  • If motor followed RC → ✅ Takeover works")
print("  • If motor stayed 1700 → ❌ RC not configured")
print("=" * 60)
