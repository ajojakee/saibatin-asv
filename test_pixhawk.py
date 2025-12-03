#!/usr/bin/env python3
"""
Pixhawk Connection Test Script
Test koneksi dan baca data dari Pixhawk via pymavlink
"""

import time
import sys

try:
    from pymavlink import mavutil
    print("✅ pymavlink imported successfully")
except ImportError:
    print("❌ pymavlink not installed!")
    print("   Install: pip3 install pymavlink")
    sys.exit(1)

PIXHAWK_PORTS = ["/dev/ttyAMA0", "/dev/ttyACM0", "/dev/ttyUSB0"]
PIXHAWK_BAUD = 57600

def test_pixhawk_connection():
    """Test koneksi ke Pixhawk"""
    print("\n🔌 Testing Pixhawk Connection...")
    print("=" * 50)
    
    connection = None
    
    for port in PIXHAWK_PORTS:
        try:
            print(f"\n   Trying port: {port} @ {PIXHAWK_BAUD} baud")
            connection = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD)
            
            print("   Waiting for heartbeat...")
            connection.wait_heartbeat(timeout=10)
            
            print(f"✅ Heartbeat received from system {connection.target_system}")
            print(f"   Component: {connection.target_component}")
            break
            
        except Exception as e:
            print(f"❌ Failed on {port}: {e}")
            continue
    
    if not connection:
        print("\n❌ No Pixhawk found on any port!")
        print("\nTroubleshooting:")
        print("1. Check physical connection (UART/Telemetry)")
        print("2. Check Pixhawk power (LED blinking?)")
        print("3. Check port permissions: ls -l /dev/tty*")
        print("4. Try: sudo usermod -a -G dialout $USER")
        return None
    
    return connection

def test_gps_data(connection, duration=10):
    """Test baca GPS data selama N detik"""
    print(f"\n📡 Reading GPS data for {duration} seconds...")
    print("=" * 50)
    
    gps_count = 0
    vfr_count = 0
    attitude_count = 0
    battery_count = 0
    
    start_time = time.time()
    
    while time.time() - start_time < duration:
        try:
            msg = connection.recv_match(blocking=False)
            
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == "GLOBAL_POSITION_INT":
                    gps_count += 1
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0  # mm to meters
                    
                    if gps_count % 5 == 0:  # Print every 5 messages
                        print(f"📍 GPS: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m")
                
                elif msg_type == "VFR_HUD":
                    vfr_count += 1
                    speed = msg.groundspeed  # m/s
                    heading = msg.heading
                    
                    if vfr_count % 5 == 0:
                        print(f"🧭 VFR: Speed={speed:.2f}m/s, Heading={heading}°")
                
                elif msg_type == "ATTITUDE":
                    attitude_count += 1
                    import math
                    pitch = math.degrees(msg.pitch)
                    roll = math.degrees(msg.roll)
                    
                    if attitude_count % 10 == 0:
                        print(f"📐 ATT: Pitch={pitch:.1f}°, Roll={roll:.1f}°")
                
                elif msg_type == "SYS_STATUS":
                    battery_count += 1
                    voltage = msg.voltage_battery / 1000.0  # mV to V
                    battery_pct = msg.battery_remaining
                    
                    if battery_count % 5 == 0:
                        print(f"🔋 BAT: {voltage:.2f}V ({battery_pct}%)")
            
            time.sleep(0.1)
            
        except KeyboardInterrupt:
            print("\n⚠️ Test interrupted by user")
            break
        except Exception as e:
            print(f"⚠️ Error reading message: {e}")
    
    print("\n" + "=" * 50)
    print("📊 Summary:")
    print(f"   GPS messages:      {gps_count}")
    print(f"   VFR_HUD messages:  {vfr_count}")
    print(f"   ATTITUDE messages: {attitude_count}")
    print(f"   BATTERY messages:  {battery_count}")
    
    if gps_count == 0:
        print("\n⚠️ WARNING: No GPS data received!")
        print("   - GPS may not have lock (need clear sky)")
        print("   - Wait 3-5 minutes for GPS lock outdoor")
    else:
        print("\n✅ GPS data is flowing correctly!")

def test_motor_channels(connection):
    """Test PWM output channels (READ ONLY, tidak gerakkan motor!)"""
    print("\n⚡ Testing PWM Channel Configuration...")
    print("=" * 50)
    
    try:
        # Request parameter list
        connection.mav.param_request_list_send(
            connection.target_system,
            connection.target_component
        )
        
        print("Waiting for SERVO parameters...")
        
        servo_params = {}
        timeout = time.time() + 5
        
        while time.time() < timeout:
            msg = connection.recv_match(type='PARAM_VALUE', blocking=False)
            if msg:
                param_id = msg.param_id
                if param_id.startswith('SERVO'):
                    servo_params[param_id] = msg.param_value
            time.sleep(0.01)
        
        print("\n📋 Relevant SERVO Parameters:")
        for param in sorted(servo_params.keys()):
            if any(x in param for x in ['SERVO4', 'SERVO5']):
                print(f"   {param}: {servo_params[param]}")
        
        if not servo_params:
            print("⚠️ No SERVO parameters received (timeout)")
        
    except Exception as e:
        print(f"❌ Error reading parameters: {e}")

def main():
    print("\n" + "=" * 50)
    print("   PIXHAWK CONNECTION TEST")
    print("   SAIBATIN AZURA 1.0")
    print("=" * 50)
    
    # Test 1: Connection
    connection = test_pixhawk_connection()
    if not connection:
        sys.exit(1)
    
    # Test 2: GPS Data
    test_gps_data(connection, duration=10)
    
    # Test 3: Motor Channels (optional)
    print("\nTest PWM channels? (y/n): ", end='')
    try:
        answer = input().strip().lower()
        if answer == 'y':
            test_motor_channels(connection)
    except EOFError:
        pass
    
    print("\n✅ Test completed!")
    print("\nNext steps:")
    print("1. If GPS count = 0, test outdoor (need clear sky)")
    print("2. If all tests pass, ready for outdoor mission test")
    print("3. Run: python3 saibatin_production_fixed.py --server <IP>")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️ Test interrupted")
        sys.exit(0)
