#!/usr/bin/env python3
"""
Disable Pixhawk Failsafe untuk enable MAVLink control tanpa RC
"""
from pymavlink import mavutil
import time

print("=" * 60)
print("   DISABLE PIXHAWK FAILSAFE")
print("=" * 60)

# Connect
print("\nConnecting to Pixhawk...")
conn = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
conn.wait_heartbeat()
print(f"✅ Connected (System {conn.target_system})\n")

# Critical parameters untuk disable failsafe
params = [
    ('ARMING_CHECK', 0),      # Disable pre-arm checks
    ('FS_GCS_ENABLE', 0),     # Disable GCS failsafe (CRITICAL!)
    ('FS_ACTION', 0),         # No failsafe action
    ('FS_TIMEOUT', 0),        # No failsafe timeout
    ('BRD_SAFETY_DEFLT', 0),  # Disable safety switch
    ('SYSID_MYGCS', 255),     # Allow all GCS
]

print("Setting critical parameters...")
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
        # FIX: param_id bisa string atau bytes, handle keduanya
        param_id = msg.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode('utf-8').strip('\x00')
        elif isinstance(param_id, str):
            param_id = param_id.strip('\x00')
        
        if param_id == param_name:
            print(f"    ✅ {param_name} = {msg.param_value}")
        else:
            print(f"    ⚠️ {param_name} - got {param_id} instead")
    else:
        print(f"    ⚠️ {param_name} - no confirmation")

print("\n" + "=" * 60)
print("✅ Parameters set!")
print("=" * 60)
print("\n⚠️ IMPORTANT:")
print("1. Reboot Pixhawk (power cycle)")
print("2. Run: python3 force_guided_mode.py")
print("3. Motor should work now!")
print("=" * 60)
