"""
SAIBATIN AZURA 1.0 - ALL-IN-ONE MISSION SYSTEM
Complete autonomous navigation with ball detection, GPS, and mission tracking
"""

import cv2
import numpy as np
import socketio
import base64
import time
import threading
from datetime import datetime
import random
import math

# Try import Pixhawk support
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("⚠️ pymavlink not available - GPS dummy mode")

# ========== KONFIGURASI ==========
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
SOCKETIO_SERVER = "http://192.168.43.1:5000"

# Pixhawk ports
PIXHAWK_PORTS = ["/dev/ttyAMA0", "/dev/ttyACM0", "/dev/ttyUSB0"]
PIXHAWK_BAUD = 57600

# Ball detection HSV threshold
RED_LOWER1 = np.array([0, 120, 70])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 70])
RED_UPPER2 = np.array([180, 255, 255])
GREEN_LOWER = np.array([40, 40, 40])
GREEN_UPPER = np.array([80, 255, 255])
MIN_BALL_RADIUS = 15

# Navigation parameters
AUTONOMOUS_MODE = False
TARGET_SPEED = 2.0  # m/s
HEADING_THRESHOLD = 10  # degrees
OFFSET_THRESHOLD = 50  # pixels

# ========== GLOBAL ==========
sio = socketio.Client()
camera = None
mavlink_connection = None
latest_gps_data = {}
mission_status = "Standby"
data_send_count = 0
current_heading = 0
target_heading = 0
balls_passed = 0

# ========== SOCKET.IO ==========
@sio.event
def connect():
    print("✅ Connected to Dashboard")

@sio.event
def disconnect():
    print("❌ Disconnected")

def connect_to_server():
    while True:
        try:
            sio.connect(SOCKETIO_SERVER)
            break
        except Exception as e:
            print(f"⚠️ Connection failed: {e}")
            time.sleep(5)

# ========== PIXHAWK CONNECTION ==========
def connect_pixhawk():
    global mavlink_connection
    if not MAVLINK_AVAILABLE:
        return False
    
    for port in PIXHAWK_PORTS:
        try:
            print(f"   Trying {port}...")
            connection = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD)
            connection.wait_heartbeat(timeout=5)
            mavlink_connection = connection
            print(f"✅ Pixhawk connected on {port}")
            return True
        except:
            continue
    
    print("⚠️ Pixhawk not found - GPS dummy mode")
    return False

# ========== GPS DATA ==========
def generate_dummy_gps_data():
    base_lat = -5.3972
    base_lon = 105.2668
    lat = base_lat + random.uniform(-0.0001, 0.0001)
    lon = base_lon + random.uniform(-0.0001, 0.0001)
    now = datetime.now()
    speed_knot = random.uniform(0, 2.5)
    heading = random.randint(0, 359)
    battery = random.randint(85, 100)
    
    return {
        'Day': now.strftime('%A'),
        'Date': now.strftime('%Y-%m-%d'),
        'Time': now.strftime('%H:%M:%S'),
        'Latitude_DD': f"{lat:.6f}",
        'Longitude_DD': f"{lon:.6f}",
        'Latitude_DM': f"5°23.{random.randint(800,900)}'S",
        'Longitude_DM': f"105°16.{random.randint(0,99):02d}'E",
        'SOG_knot': f"{speed_knot:.2f}",
        'SOG_kmh': f"{speed_knot * 1.852:.2f}",
        'COG_deg': f"{heading}",
        'Pitch': f"{random.uniform(-5, 5):.2f}",
        'Roll': f"{random.uniform(-5, 5):.2f}",
        'Battery_': f"{battery}",
        'Mission_Status': mission_status
    }

def read_gps_data():
    global latest_gps_data, current_heading, mavlink_connection
    
    if not mavlink_connection:
        # Dummy GPS
        while True:
            latest_gps_data = generate_dummy_gps_data()
            time.sleep(1)
        return
    
    # Real Pixhawk GPS
    while True:
        try:
            msg = mavlink_connection.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    latest_gps_data['Latitude_DD'] = f"{lat:.6f}"
                    latest_gps_data['Longitude_DD'] = f"{lon:.6f}"
                
                elif msg_type == "VFR_HUD":
                    current_heading = msg.heading
                    latest_gps_data['SOG_knot'] = f"{msg.groundspeed * 1.94384:.2f}"
                    latest_gps_data['COG_deg'] = f"{msg.heading}"
                
                elif msg_type == "ATTITUDE":
                    latest_gps_data['Pitch'] = f"{math.degrees(msg.pitch):.2f}"
                    latest_gps_data['Roll'] = f"{math.degrees(msg.roll):.2f}"
                
                elif msg_type == "SYS_STATUS":
                    latest_gps_data['Battery_'] = f"{msg.battery_remaining}"
                
                now = datetime.now()
                latest_gps_data['Day'] = now.strftime('%A')
                latest_gps_data['Date'] = now.strftime('%Y-%m-%d')
                latest_gps_data['Time'] = now.strftime('%H:%M:%S')
                latest_gps_data['Mission_Status'] = mission_status
            
            time.sleep(0.1)
        except:
            time.sleep(1)

def send_dashboard_update():
    global data_send_count
    while True:
        try:
            if sio.connected and latest_gps_data:
                sio.emit('update_dashboard', latest_gps_data)
                data_send_count += 1
                if data_send_count % 10 == 0:
                    print(f"📡 Sent {data_send_count} GPS updates")
            time.sleep(1)
        except:
            time.sleep(1)

# ========== CAMERA ==========
def init_usb_camera():
    global camera
    try:
        camera = cv2.VideoCapture(0)
        if not camera.isOpened():
            return False
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, 30)
        ret, _ = camera.read()
        if not ret:
            return False
        print(f"✅ USB Camera initialized")
        return True
    except Exception as e:
        print(f"❌ Camera error: {e}")
        return False

def detect_ball(frame_bgr, lower, upper, lower2=None, upper2=None):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    if lower2 is not None and upper2 is not None:
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower, upper), cv2.inRange(hsv, lower2, upper2))
    else:
        mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)
        if radius > MIN_BALL_RADIUS:
            return {'x': int(x), 'y': int(y), 'radius': int(radius)}
    return None

def calculate_navigation(red_ball, green_ball):
    if not (red_ball and green_ball):
        return None, None, None
    mid_x = (red_ball['x'] + green_ball['x']) / 2
    mid_y = (red_ball['y'] + green_ball['y']) / 2
    offset_x = mid_x - CAMERA_WIDTH / 2
    heading_adj = (offset_x / CAMERA_WIDTH) * 60
    heading_adj = max(-30, min(30, heading_adj))
    return mid_x, mid_y, heading_adj

# ========== MISSION CONTROL ==========
def capture_surface_photo(frame_bgr):
    """Capture surface photo for mission"""
    _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
    photo_data = {
        'image': f"data:image/jpeg;base64,{jpg_as_text}",
        'location': f"{latest_gps_data.get('Latitude_DD', 'N/A')}, {latest_gps_data.get('Longitude_DD', 'N/A')}",
        'timestamp': datetime.now().strftime('%H:%M:%S')
    }
    sio.emit('surface_photo', photo_data)
    print("📸 Surface photo captured!")

def camera_stream_loop():
    global mission_status, camera, balls_passed
    frame_count = 0
    
    while True:
        try:
            if not camera or not camera.isOpened():
                time.sleep(1)
                continue
            
            ret, frame_bgr = camera.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            frame_height = frame_bgr.shape[0]
            center_frame = CAMERA_WIDTH // 2
            
            # Detect balls
            red_ball = detect_ball(frame_bgr, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
            green_ball = detect_ball(frame_bgr, GREEN_LOWER, GREEN_UPPER)
            mid_x, mid_y, heading_adj = calculate_navigation(red_ball, green_ball)
            
            # Draw detection
            if red_ball:
                cv2.circle(frame_bgr, (red_ball['x'], red_ball['y']), red_ball['radius'], (0, 0, 255), 3)
                cv2.putText(frame_bgr, "RED", (red_ball['x'] - 20, red_ball['y'] - red_ball['radius'] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.line(frame_bgr, (red_ball['x'], 0), (red_ball['x'], frame_height), (0, 0, 255), 2)
            
            if green_ball:
                cv2.circle(frame_bgr, (green_ball['x'], green_ball['y']), green_ball['radius'], (0, 255, 0), 3)
                cv2.putText(frame_bgr, "GREEN", (green_ball['x'] - 30, green_ball['y'] - green_ball['radius'] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.line(frame_bgr, (green_ball['x'], 0), (green_ball['x'], frame_height), (0, 255, 0), 2)
            
            # Draw midpoint & navigation
            direction = ""
            if mid_x is not None:
                midpoint_x = int(mid_x)
                offset_x = midpoint_x - center_frame
                
                cv2.line(frame_bgr, (midpoint_x, 0), (midpoint_x, frame_height), (255, 0, 0), 3)
                cv2.circle(frame_bgr, (midpoint_x, int(mid_y)), 10, (255, 217, 61), 3)
                cv2.line(frame_bgr, (center_frame - 20, frame_height // 2), 
                        (center_frame + 20, frame_height // 2), (255, 255, 255), 2)
                
                if offset_x < -OFFSET_THRESHOLD:
                    direction = "⬅️ Belok Kiri"
                elif offset_x > OFFSET_THRESHOLD:
                    direction = "➡️ Belok Kanan"
                else:
                    direction = "⬆️ Lurus"
                
                cv2.putText(frame_bgr, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame_bgr, f"Offset: {offset_x}px", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 217, 61), 2)
                cv2.putText(frame_bgr, f"Balls: {balls_passed}/10", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 217, 61), 2)
                
                mission_status = f"{direction} | Balls: {balls_passed}/10" if abs(offset_x) >= OFFSET_THRESHOLD else f"ON TARGET | Balls: {balls_passed}/10"
            else:
                mission_status = "Searching for Balls"
            
            cv2.line(frame_bgr, (center_frame, 0), (center_frame, frame_height), (255, 255, 255), 1)
            
            # Encode & send
            _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            if sio.connected:
                camera_data = {
                    'image': f"data:image/jpeg;base64,{jpg_as_text}",
                    'red_ball': red_ball,
                    'green_ball': green_ball,
                    'midpoint': {'x': int(mid_x) if mid_x else None, 'y': int(mid_y) if mid_y else None},
                    'heading_adjustment': heading_adj,
                    'direction': direction,
                    'balls_passed': balls_passed
                }
                sio.emit('camera_frame', camera_data)
                
                frame_count += 1
                if frame_count % 100 == 0:
                    print(f"📷 {frame_count} frames | {mission_status}")
            
            time.sleep(0.033)
        except Exception as e:
            print(f"❌ Camera error: {e}")
            time.sleep(1)

# ========== MAIN ==========
def main():
    print("🚀 SAIBATIN AZURA 1.0 - ALL-IN-ONE MISSION SYSTEM")
    print("=" * 70)
    
    connect_to_server()
    pixhawk_ok = connect_pixhawk()
    camera_ok = init_usb_camera()
    
    if not camera_ok:
        print("❌ Camera failed!")
        return
    
    gps_thread = threading.Thread(target=read_gps_data, daemon=True)
    gps_thread.start()
    print("✅ GPS started")
    
    dash_thread = threading.Thread(target=send_dashboard_update, daemon=True)
    dash_thread.start()
    print("✅ Dashboard started")
    
    print("🎥 Camera streaming...")
    print("=" * 70)
    print(f"📊 Pixhawk: {'✅ Connected' if pixhawk_ok else '⚠️ Dummy GPS'}")
    print("✅ Ready! Ctrl+C to stop")
    
    try:
        camera_stream_loop()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        if camera:
            camera.release()
        sio.disconnect()
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()
