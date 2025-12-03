#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - Check Pixhawk Status
Check if Pixhawk is armed, GPS fix, and flight mode
"""

import time
import sys

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed!")
    print("Run: pip3 install pymavlink")
    sys.exit(1)

# ========== CONFIG ==========
PIXHAWK_PORTS = ["/dev/ttyAMA0", "/dev/ttyACM0", "/dev/ttyUSB0"]
PIXHAWK_BAUD = 57600

# ========== CONNECT ==========
def connect_pixhawk():
    print("Connecting to Pixhawk...")
    for port in PIXHAWK_PORTS:
        try:
            print(f"  Trying {port}...")
            connection = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD)
            connection.wait_heartbeat(timeout=5)
            print(f"✅ Connected on {port}")
            return connection
        except Exception as e:
            print(f"  ❌ {e}")
    print("\n❌ Cannot connect to Pixhawk!")
    sys.exit(1)

# ========== CHECK STATUS ==========
def check_pixhawk_status(connection):
    print("\n" + "=" * 60)
    print("   PIXHAWK STATUS CHECK")
    print("=" * 60)
    
    # Wait for HEARTBEAT
    print("\n1. Waiting for HEARTBEAT...")
    msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        print(f"   System ID: {connection.target_system}")
        print(f"   Component ID: {connection.target_component}")
        print(f"   Autopilot: {msg.autopilot}")
        print(f"   Type: {msg.type}")
        
        # Check if ARMED
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed:
            print("   Status: 🟢 ARMED (motors can run)")
        else:
            print("   Status: 🔴 DISARMED (motors disabled)")
        
        # Check flight mode
        custom_mode = msg.custom_mode
        mode_mapping = {
            0: 'MANUAL',
            1: 'ACRO',
            2: 'STEERING',
            3: 'HOLD',
            4: 'LOITER',
            5: 'FOLLOW',
            6: 'SIMPLE',
            10: 'AUTO',
            11: 'RTL',
            12: 'SMART_RTL',
            15: 'GUIDED'
        }
        mode_name = mode_mapping.get(custom_mode, f'UNKNOWN({custom_mode})')
        print(f"   Flight Mode: {mode_name}")
    else:
        print("   ❌ No HEARTBEAT received")
        return False
    
    # Check GPS
    print("\n2. Checking GPS...")
    msg = connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg:
        fix_type = msg.fix_type
        fix_names = {
            0: 'NO_GPS',
            1: 'NO_FIX',
            2: '2D_FIX',
            3: '3D_FIX',
            4: 'DGPS',
            5: 'RTK_FLOAT',
            6: 'RTK_FIXED'
        }
        print(f"   GPS Fix: {fix_names.get(fix_type, 'UNKNOWN')}")
        print(f"   Satellites: {msg.satellites_visible}")
        
        if fix_type >= 3:
            print("   GPS Status: 🟢 OK (3D Fix or better)")
        else:
            print("   GPS Status: 🔴 NOT READY (No 3D Fix)")
    else:
        print("   ❌ No GPS data")
    
    # Check Battery
    print("\n3. Checking Battery...")
    msg = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if msg:
        voltage = msg.voltage_battery / 1000.0  # mV to V
        current = msg.current_battery / 100.0   # cA to A
        remaining = msg.battery_remaining       # %
        print(f"   Voltage: {voltage:.2f}V")
        print(f"   Current: {current:.2f}A")
        print(f"   Remaining: {remaining}%")
        
        if remaining > 50:
            print("   Battery Status: 🟢 OK")
        elif remaining > 20:
            print("   Battery Status: 🟡 Low")
        else:
            print("   Battery Status: 🔴 Critical")
    else:
        print("   ❌ No battery data")
    
    # Check RC Channels
    print("\n4. Checking RC Channels (PWM)...")
    msg = connection.recv_match(type='RC_CHANNELS', blocking=True, timeout=5)
    if msg:
        print(f"   Chan1 (Roll):     {msg.chan1_raw}")
        print(f"   Chan2 (Pitch):    {msg.chan2_raw}")
        print(f"   Chan3 (Throttle): {msg.chan3_raw}")
        print(f"   Chan4 (Yaw):      {msg.chan4_raw}")
        print(f"   Chan5 (Motor L):  {msg.chan5_raw if hasattr(msg, 'chan5_raw') else 'N/A'}")
        print(f"   Chan6 (Motor R):  {msg.chan6_raw if hasattr(msg, 'chan6_raw') else 'N/A'}")
    else:
        print("   ❌ No RC channel data")
    
    print("\n" + "=" * 60)
    return True

# ========== ARM/DISARM FUNCTIONS ==========
def arm_pixhawk(connection):
    """ARM Pixhawk (enable motors)"""
    print("\n🔓 Arming Pixhawk...")
    
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # param1: 1=arm, 0=disarm
        0, 0, 0, 0, 0, 0
    )
    
    # Wait for ACK
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ Pixhawk ARMED successfully")
        print("⚠️ MOTORS CAN NOW RUN!")
        return True
    else:
        print("❌ ARM failed")
        if msg:
            print(f"   Result: {msg.result}")
        return False

def disarm_pixhawk(connection):
    """DISARM Pixhawk (disable motors)"""
    print("\n🔒 Disarming Pixhawk...")
    
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # param1: 0=disarm
        0, 0, 0, 0, 0, 0
    )
    
    # Wait for ACK
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ Pixhawk DISARMED successfully")
        print("✅ Motors disabled (safe)")
        return True
    else:
        print("❌ DISARM failed")
        return False

# ========== INTERACTIVE MENU ==========
def interactive_menu(connection):
    while True:
        print("\n" + "=" * 60)
        print("PIXHAWK CONTROL MENU")
        print("=" * 60)
        print("1. Check Status")
        print("2. ARM Pixhawk (Enable Motors)")
        print("3. DISARM Pixhawk (Disable Motors)")
        print("4. Monitor Real-time (5 seconds)")
        print("5. Exit")
        print("=" * 60)
        
        try:
            choice = input("\nSelect option (1-5): ").strip()
            
            if choice == '1':
                check_pixhawk_status(connection)
            
            elif choice == '2':
                print("\n⚠️ WARNING: This will ENABLE motors!")
                confirm = input("Type 'ARM' to confirm: ").strip().upper()
                if confirm == 'ARM':
                    arm_pixhawk(connection)
                else:
                    print("❌ Cancelled")
            
            elif choice == '3':
                disarm_pixhawk(connection)
            
            elif choice == '4':
                print("\nMonitoring for 5 seconds...")
                start_time = time.time()
                while time.time() - start_time < 5:
                    msg = connection.recv_match(blocking=False)
                    if msg:
                        print(f"  {msg.get_type()}: {msg}")
                    time.sleep(0.1)
                print("✅ Monitoring complete")
            
            elif choice == '5':
                print("\n👋 Exiting...")
                break
            
            else:
                print("❌ Invalid choice")
        
        except KeyboardInterrupt:
            print("\n\n🛑 Interrupted!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")

# ========== MAIN ==========
def main():
    try:
        connection = connect_pixhawk()
        check_pixhawk_status(connection)
        interactive_menu(connection)
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted!")
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
