"""
Test script untuk verifikasi koneksi Pixhawk dan Dashboard
"""

import time
import socketio
from datetime import datetime

# Konfigurasi
SOCKETIO_SERVER = "http://10.93.210.110:5000"
PIXHAWK_PORT = "/dev/ttyACM0"

def test_dashboard_connection():
    """Test koneksi ke dashboard"""
    print("\n" + "="*60)
    print("🌐 Testing Dashboard Connection")
    print("="*60)
    
    sio = socketio.Client()
    
    @sio.event
    def connect():
        print("✅ Connected to Dashboard!")
    
    @sio.event
    def disconnect():
        print("❌ Disconnected from Dashboard")
    
    try:
        print(f"📡 Connecting to: {SOCKETIO_SERVER}")
        sio.connect(SOCKETIO_SERVER, wait_timeout=10)
        
        # Send test data
        test_data = {
            'Day': 'Monday',
            'Date': '2024-01-01',
            'Time': datetime.now().strftime('%H:%M:%S'),
            'Latitude_DD': '-5.397200',
            'Longitude_DD': '105.266800',
            'Latitude_DM': "5°23.832'S",
            'Longitude_DM': "105°16.008'E",
            'SOG_knot': '1.50',
            'SOG_kmh': '2.78',
            'COG_deg': '45',
            'Pitch': '2.5',
            'Roll': '-1.2',
            'Battery_': '95',
            'Mission_Status': 'Testing Connection'
        }
        
        print("📤 Sending test data...")
        sio.emit('update_dashboard', test_data)
        print("✅ Test data sent successfully!")
        
        time.sleep(2)
        sio.disconnect()
        return True
        
    except Exception as e:
        print(f"❌ Dashboard connection failed: {e}")
        return False

def test_pixhawk_connection():
    """Test koneksi ke Pixhawk"""
    print("\n" + "="*60)
    print("🛰️ Testing Pixhawk Connection")
    print("="*60)
    
    try:
        from pymavlink import mavutil
        print(f"📡 Connecting to: {PIXHAWK_PORT}")
        
        connection = mavutil.mavlink_connection(PIXHAWK_PORT, baud=57600)
        print("⏳ Waiting for heartbeat...")
        connection.wait_heartbeat(timeout=10)
        
        print("✅ Pixhawk connected!")
        
        # Read some test data
        print("\n📊 Reading test data (10 seconds)...")
        start_time = time.time()
        data_count = 0
        
        while time.time() - start_time < 10:
            msg = connection.recv_match(blocking=False)
            if msg:
                data_count += 1
                msg_type = msg.get_type()
                
                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    print(f"📍 GPS: Lat={lat:.6f}, Lon={lon:.6f}")
                
                elif msg_type == "VFR_HUD":
                    speed = msg.groundspeed * 1.94384
                    heading = msg.heading
                    print(f"🧭 Speed={speed:.2f} knots, Heading={heading}°")
                
                elif msg_type == "SYS_STATUS":
                    battery = msg.battery_remaining
                    print(f"🔋 Battery={battery}%")
            
            time.sleep(0.1)
        
        print(f"\n✅ Received {data_count} messages from Pixhawk")
        return True
        
    except ImportError:
        print("❌ pymavlink not installed")
        print("   Install: pip3 install pymavlink")
        return False
    except Exception as e:
        print(f"❌ Pixhawk connection failed: {e}")
        return False

def main():
    print("\n🚀 SAIBATIN AZURA 1.0 - Connection Test")
    print("="*60)
    
    # Test 1: Dashboard
    dashboard_ok = test_dashboard_connection()
    
    # Test 2: Pixhawk
    pixhawk_ok = test_pixhawk_connection()
    
    # Summary
    print("\n" + "="*60)
    print("📋 TEST SUMMARY")
    print("="*60)
    print(f"Dashboard Connection: {'✅ PASS' if dashboard_ok else '❌ FAIL'}")
    print(f"Pixhawk Connection:   {'✅ PASS' if pixhawk_ok else '❌ FAIL'}")
    print("="*60)
    
    if dashboard_ok and pixhawk_ok:
        print("\n🎉 All tests passed! System ready for deployment!")
    elif dashboard_ok:
        print("\n⚠️ Dashboard OK, but Pixhawk not connected.")
        print("   System will use DUMMY DATA mode.")
    else:
        print("\n❌ Connection issues detected. Please check:")
        if not dashboard_ok:
            print("   - Dashboard server IP and port")
            print("   - Network connection")
        if not pixhawk_ok:
            print("   - Pixhawk USB/Serial connection")
            print("   - Pixhawk port (/dev/ttyACM0 or /dev/ttyUSB0)")

if __name__ == "__main__":
    main()
