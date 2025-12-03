#!/usr/bin/env python3
"""
Setup Pixhawk untuk RC + MAVLink Hybrid Control
Allows: Autonomous MAVLink + Manual RC Emergency Takeover
"""
from pymavlink import mavutil
import time

print("=" * 60)
print("   SETUP RC + MAVLink HYBRID CONTROL")
print("=" * 60)

# Connect
print("\nConnecting to Pixhawk...")
conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
conn.wait_heartbeat()
print(f"✅ Connected (System {conn.target_system})\n")

# Parameters for hybrid control
params = [
    # MAVLink control
    ('SYSID_MYGCS', 255),          # Allow MAVLink
    ('FS_GCS_ENABLE', 0),          # Disable GCS failsafe
    ('RC_OVERRIDE_TIME', 3000),    # 3 sec timeout (RC priority)
    
    # Motor outputs
    ('SERVO4_FUNCTION', 73),       # ThrottleLeft
    ('SERVO5_FUNCTION', 74),       # ThrottleRight
    
    # Safety (keep enabled!)
    ('ARMING_CHECK', -9),          # Enable most checks (safe)
    ('BRD_SAFETY_DEFLT', 0),       # Disable safety button
    
    # RC failsafe (keep for safety!)
    ('FS_ACTION', 1),              # Hold position on RC loss
    ('FS_TIMEOUT', 5),             # 5 sec RC timeout
]

print("Setting hybrid control parameters...")
for param_name, param_value in params:
    print(f"  Setting {param_name} = {param_value}...")
    
    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        param_name.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.5)
    
    # Verify
    msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        param_id = msg.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode('utf-8').strip('\x00')
        elif isinstance(param_id, str):
            param_id = param_id.strip('\x00')
        
        if param_id == param_name:
            print(f"    ✅ {param_name} = {msg.param_value}")
        else:
            print(f"    ⚠️ {param_name} - verification uncertain")
    else:
        print(f"    ⚠️ {param_name} - no confirmation")

print("\n" + "=" * 60)
print("✅ Hybrid control configured!")
print("=" * 60)
print("\n📋 Usage:")
print("  • Autonomous: RC sticks neutral → Python controls motors")
print("  • Manual: Move RC sticks → RC takes over (emergency)")
print("  • Release RC sticks → Returns to autonomous (3 sec)")
print("\n⚠️ REBOOT PIXHAWK to apply changes!")
print("   Run: python3 reboot_pixhawk.py")
print("=" * 60)
