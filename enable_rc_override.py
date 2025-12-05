#!/usr/bin/env python3
"""
Enable RC Override on Pixhawk
"""
from pymavlink import mavutil

print("Connecting to Pixhawk...")
connection = mavutil.mavlink_connection('/devWindows IP Configuration


Wireless LAN adapter Local Area Connection* 1:

   Media State . . . . . . . . . . . : Media disconnected
   Connection-specific DNS Suffix  . :

Wireless LAN adapter Local Area Connection* 10:

   Media State . . . . . . . . . . . : Media disconnected
   Connection-specific DNS Suffix  . :

Wireless LAN adapter Wi-Fi:

   Connection-specific DNS Suffix  . :
   Link-local IPv6 Address . . . . . : fe80::f876:f90e:806e:9bbf%8
   IPv4 Address. . . . . . . . . . . : 10.115.96.157
   Subnet Mask . . . . . . . . . . . : 255.255.255.0
   Default Gateway . . . . . . . . . : 10.115.96.160
PS C:\Users\Zaky>/ttyAMA0', baud=57600)
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
