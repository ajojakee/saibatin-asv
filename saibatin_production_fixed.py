#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - ALL-IN-ONE MISSION SYSTEM (fixed & improved)
Now with Mission State Machine integration and fixes:
- non-blocking Socket.IO connect (background, backoff)
- obstacle_lock and latest_obstacles defined
- single (updated) camera_stream_loop (with obstacle detection)
- responsive stop_event checks in gps loop
- minor robustness improvements
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
import argparse
import sys
import traceback
import logging
import os
from typing import Tuple
from mission_state_machine import MissionStateMachine, MissionConfig, MissionState

# ========== LOGGING SETUP ==========
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler('saibatin.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Try import Pixhawk support
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except Exception:
    MAVLINK_AVAILABLE = False
    logger.warning("⚠️ pymavlink not available - GPS dummy mode")

# Try import GPIO support for direct motor control
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    # GPIO Motor Pins
    MOTOR_LEFT_GPIO = 18   # GPIO 18 - Motor Kiri
    MOTOR_RIGHT_GPIO = 13  # GPIO 13 - Motor Kanan
    PWM_FREQ_GPIO = 50     # 50Hz for ESC
except ImportError:
    GPIO_AVAILABLE = False
    logger.warning("⚠️ RPi.GPIO not available - motor control disabled")

# ========== KONFIGURASI ==========
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
SOCKETIO_SERVER = "http://10.132.119.157:5000"  # ← FIXED! IP laptop yang BENAR!

# Dual camera setup
CAMERA_NAVIGATION = 0  # Logitech C270 for navigation + surface
CAMERA_UNDERWATER = 1  # Brica PRO for underwater (USB mode)

# Brica PRO WiFi streaming (if not using USB)
BRICA_WIFI_STREAM = "http://192.168.0.1:8080/video"
BRICA_PHOTO_CAPTURE_URL = "http://192.168.0.1:8080/photo"
USE_BRICA_WIFI = False  # ← CHANGED to False! Disable WiFi untuk test dulu

# Pixhawk ports
PIXHAWK_PORTS = ["/dev/ttyAMA0", "/dev/ttyACM0", "/dev/ttyUSB0"]
PIXHAWK_BAUD = 57600

# Motor control parameters (FINAL FIX!)
PWM_MOTOR_LEFT_CHANNEL = 4   # ← Channel 4 = Motor Kiri (SERVO4_FUNCTION = 73)
PWM_MOTOR_RIGHT_CHANNEL = 5  # ← Channel 5 = Motor Kanan (SERVO5_FUNCTION = 74)

PWM_MIN = 1100  # Minimum PWM (full reverse)
PWM_MID = 1500  # Neutral PWM (stop)
PWM_MAX = 1900  # Maximum PWM (full forward)

# Ball detection HSV threshold (UPDATED dari kode kalibr asi Anda!)
# Pylox Red 33 - Range diperketat
RED_LOWER1 = np.array([0, 120, 150])    # ← UPDATED! Dari kode Anda
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 150])  # ← UPDATED!
RED_UPPER2 = np.array([180, 255, 255])

# Pylox Green 105 - Range diperketat
GREEN_LOWER = np.array([45, 120, 120])  # ← UPDATED! Dari kode Anda
GREEN_UPPER = np.array([70, 255, 255])  # Bright, high saturation

# NEW: Blue ball detection for underwater photo trigger
BLUE_LOWER = np.array([100, 150, 50])   # Blue hue (100-130°)
BLUE_UPPER = np.array([130, 255, 255])  # Bright blue

MIN_BALL_RADIUS = 15  # Minimum radius in pixels to consider as valid ball

# Underwater photo trigger parameters (NEW!)
UNDERWATER_PHOTO_COOLDOWN = 5.0  # seconds - cooldown between captures
last_underwater_photo_time = 0

# Gate detection parameters
MIN_GATE_DISTANCE = 5.0  # Minimum jarak ke gate untuk mulai deteksi (meter)
MAX_GATE_DISTANCE = 50.0  # Maximum jarak deteksi gate (meter)
MIN_BALL_SEPARATION = 30  # Minimum jarak pixel antar red-green ball (untuk validasi gate)
MAX_BALL_SEPARATION = 400  # Maximum jarak pixel antar red-green ball (terlalu jauh = bukan gate)

# Navigation parameters
AUTONOMOUS_MODE = True  # Enable autonomous navigation
TARGET_SPEED = 2.0  # m/s
HEADING_THRESHOLD = 10  # degrees
OFFSET_THRESHOLD = 50  # pixels

# Mission control flag (NEW!)
mission_active = False  # Flag untuk kontrol motor (False = standby, True = mission running)
mission_control_lock = threading.Lock()

# Rate limiting
SEND_FPS = 10
SEND_INTERVAL = 1 / SEND_FPS

# Obstacle detection parameters
OBSTACLE_MIN_AREA = 800  # ← UPDATED! Lebih besar dari bola (500)
OBSTACLE_MIN_WIDTH = 40  # ← NEW! Minimum width in pixels
OBSTACLE_REFERENCE_WIDTH = 0.3  # ← UPDATED! reference width (meter) - lebih realistic
OBSTACLE_REFERENCE_PIXELS = 100
FOCAL_LENGTH = 600  # approximate focal length (calibrate)
MAX_OBSTACLE_DISTANCE = 20.0  # meters

# Docking parameters (GPS-based only)
DOCKING_DISTANCE_THRESHOLD = 5.0  # meters - switch to precision mode
DOCKING_FINAL_THRESHOLD = 1.0     # meters - consider docked
DOCKING_SPEED_SLOW = 20           # percent - slow approach speed
DOCKING_HEADING_PRECISION = 5     # degrees - precision heading tolerance

# ========== GLOBAL ==========
sio = socketio.Client()
camera_nav = None  # Logitech for navigation
camera_underwater = None  # Brica for underwater
mavlink_connection = None
latest_gps_data = {}
mission_status = "Standby"
data_send_count = 0
current_heading = 0
target_heading = 0
balls_passed = 0
mission_sm = None  # State machine instance

# Thread control
stop_event = threading.Event()
frame_lock = threading.Lock()
latest_frame = None

# For thread-safe GPS data access
gps_data_lock = threading.Lock()
gps_data_event = threading.Event()

# Obstacle shared data & lock (fixed: previously missing)
obstacle_lock = threading.Lock()
latest_obstacles = []

# ========== MISSION CONFIG & GATES ==========
GATES_LIST = [
    (-5.39720, 105.26680, False),  # Gate 1: surface only
    (-5.39725, 105.26685, True),   # Gate 2: surface + underwater
    (-5.39730, 105.26690, True),   # Gate 3: surface + underwater
    (-5.39735, 105.26695, False),  # Gate 4: surface only
    (-5.39740, 105.26700, True),   # Gate 5: surface + underwater
    (-5.39745, 105.26705, True),   # Gate 6: surface + underwater
    (-5.39750, 105.26710, False),  # Gate 7: surface only
    (-5.39755, 105.26715, True),   # Gate 8: surface + underwater
    (-5.39760, 105.26720, True),   # Gate 9: surface + underwater
    (-5.39765, 105.26725, False),  # Gate 10: surface only
    (-5.39770, 105.26730, False),  # Dock (final position)
]

MISSION_CFG = MissionConfig(
    gate_trigger_dist_m=3.0,
    speed_threshold_m_s=0.5,
    heading_threshold_deg=15.0,
    settle_time_s=1.0,
    burst_count=3,
    burst_interval_s=0.5,
    depth_threshold_m=0.5,
    lower_timeout_s=20.0,
    max_retries=2,
    loop_interval_s=0.25
)

# ========== SOCKET.IO ==========
@sio.event
def connect():
    logger.info("✅ Connected to Dashboard")

@sio.event
def disconnect():
    logger.warning("❌ Disconnected from Dashboard")

def connect_to_server_background(backoff_base=1.0, max_backoff=30.0):
    """Background thread to connect to Socket.IO with backoff and stop_event support."""
    attempt = 0
    while not stop_event.is_set():
        try:
            if sio.connected:
                logger.info("Socket.IO already connected")
                return
            attempt += 1
            logger.info(f"🔌 Attempting Socket.IO connect (attempt {attempt}) to {SOCKETIO_SERVER}")
            sio.connect(SOCKETIO_SERVER, wait=True, transports=['websocket'])
            logger.info("🔌 Connected to Socket.IO server")
            return
        except Exception as e:
            wait = min(max_backoff, backoff_base * (2 ** (attempt - 1)))
            logger.warning(f"⚠️ Socket.IO connect failed ({e}). Retrying in {wait:.1f}s")
            if stop_event.wait(wait):
                break

# Handler untuk menerima command dari dashboard
@sio.event
def mission_command(data):
    """Handle mission commands from dashboard"""
    global mission_sm, mission_active
    
    command = data.get('command')
    logger.info(f"📥 Received mission_command: {command}")
    
    if command == 'start':
        # Activate mission (enable motor control)
        with mission_control_lock:
            mission_active = True
        
        if mission_sm and mission_sm.state == MissionState.IDLE:
            mission_sm.start()
            logger.info("🚀 Mission STARTED!")
            logger.info("🚧 Obstacle detection: ENABLED")
            logger.info("📷 Auto photo capture: ENABLED")
            logger.info("⚡ Motor control: ENABLED")
            sio.emit('mission_status_update', {
                'state': 'NAVIGATING',
                'status': '🚀 Mission Started',
                'message': 'ASV mulai navigasi autonomous ke Gate 1'
            })
        else:
            logger.warning("Mission already running or not initialized")
    
    elif command == 'stop':
        # Deactivate mission (disable motor control)
        with mission_control_lock:
            mission_active = False
        
        # Stop motors immediately
        stop_motors()
        
        if mission_sm:
            mission_sm.stop()
            logger.info("🛑 Mission STOPPED")
            logger.info("⚡ Motor control: DISABLED")
            sio.emit('mission_status_update', {
                'state': 'IDLE',
                'status': '🛑 Mission Stopped',
                'message': 'ASV standby - motor disabled'
            })
    
    elif command == 'abort':
        # Emergency abort
        with mission_control_lock:
            mission_active = False
        
        # Stop motors immediately
        stop_motors()
        
        reason = data.get('reason', 'Manual abort from dashboard')
        if mission_sm:
            mission_sm.abort(reason)
            logger.warning(f"⚠️ Mission ABORTED: {reason}")
            sio.emit('mission_status_update', {
                'state': 'ABORTED',
                'status': '⚠️ Mission Aborted',
                'message': reason
            })

# ========== PIXHAWK CONNECTION ==========
def connect_pixhawk():
    global mavlink_connection
    if not MAVLINK_AVAILABLE:
        return False

    for port in PIXHAWK_PORTS:
        try:
            logger.info(f"   Trying {port}...")
            connection = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD)
            connection.wait_heartbeat(timeout=5)
            mavlink_connection = connection
            logger.info(f"✅ Pixhawk connected on {port}")
            return True
        except Exception:
            continue

    logger.warning("⚠️ Pixhawk not found - GPS dummy mode")
    return False

# ========== GPS DATA ==========
def generate_dummy_gps_data():
    base_lat = -5.3972
    base_lon = 105.2668
    lat = base_lat + random.uniform(-0.0001, 0.0001)
    lon = base_lon + random.uniform(-0.0001, 0.0001)
    now = datetime.utcnow()
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
    global latest_gps_data, current_heading, mavlink_connection, gps_data_event

    if not mavlink_connection:
        # Dummy GPS - responsive to stop_event
        while not stop_event.is_set():
            with gps_data_lock:
                latest_gps_data = generate_dummy_gps_data()
            gps_data_event.set()
            if stop_event.wait(1.0):
                break
        return

    # Real Pixhawk GPS - INITIALIZE data structure first
    logger.info("📡 Reading GPS data from Pixhawk...")
    
    with gps_data_lock:
        # Initialize with default values
        now = datetime.utcnow()
        latest_gps_data = {
            'Day': now.strftime('%A'),
            'Date': now.strftime('%Y-%m-%d'),
            'Time': now.strftime('%H:%M:%S'),
            'Latitude_DD': '0.000000',
            'Longitude_DD': '0.000000',
            'Latitude_DM': "0°00.000'N",
            'Longitude_DM': "0°00.000'E",
            'SOG_knot': '0.00',
            'SOG_kmh': '0.00',
            'COG_deg': '0',
            'Pitch': '0.00',
            'Roll': '0.00',
            'Battery_': '0',
            'Mission_Status': mission_status
        }

    while not stop_event.is_set():
        try:
            msg = mavlink_connection.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()

                # Update timestamp on every message
                now = datetime.utcnow()

                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    
                    # Convert to DM format
                    lat_dm = f"{abs(int(lat))}°{abs((lat - int(lat)) * 60):.3f}'" + ("N" if lat >= 0 else "S")
                    lon_dm = f"{abs(int(lon))}°{abs((lon - int(lon)) * 60):.3f}'" + ("E" if lon >= 0 else "W")
                    
                    with gps_data_lock:
                        latest_gps_data['Latitude_DD'] = f"{lat:.6f}"
                        latest_gps_data['Longitude_DD'] = f"{lon:.6f}"
                        latest_gps_data['Latitude_DM'] = lat_dm
                        latest_gps_data['Longitude_DM'] = lon_dm
                        latest_gps_data['Date'] = now.strftime('%Y-%m-%d')
                        latest_gps_data['Time'] = now.strftime('%H:%M:%S')
                        latest_gps_data['Day'] = now.strftime('%A')

                elif msg_type == "VFR_HUD":
                    current_heading = msg.heading
                    speed_ms = msg.groundspeed  # m/s
                    speed_knot = speed_ms * 1.94384
                    speed_kmh = speed_ms * 3.6
                    
                    with gps_data_lock:
                        latest_gps_data['SOG_knot'] = f"{speed_knot:.2f}"
                        latest_gps_data['SOG_kmh'] = f"{speed_kmh:.2f}"
                        latest_gps_data['COG_deg'] = f"{int(msg.heading)}"

                elif msg_type == "ATTITUDE":
                    pitch_deg = math.degrees(msg.pitch)
                    roll_deg = math.degrees(msg.roll)
                    
                    with gps_data_lock:
                        latest_gps_data['Pitch'] = f"{pitch_deg:.2f}"
                        latest_gps_data['Roll'] = f"{roll_deg:.2f}"

                elif msg_type == "SYS_STATUS":
                    battery_percent = msg.battery_remaining
                    
                    with gps_data_lock:
                        latest_gps_data['Battery_'] = f"{battery_percent}"

                # Always update mission status and time
                with gps_data_lock:
                    latest_gps_data['Mission_Status'] = mission_status
                    latest_gps_data['Time'] = now.strftime('%H:%M:%S')

            gps_data_event.set()
            if stop_event.wait(0.1):
                break
                
        except Exception as e:
            logger.warning(f"⚠️ GPS error: {e}")
            if stop_event.wait(1.0):
                break

    logger.info("GPS thread stopped")

def send_dashboard_update():
    global data_send_count
    while not stop_event.is_set():
        try:
            gps_data_event.wait(timeout=1.0)
            gps_data_event.clear()

            with gps_data_lock:
                if sio.connected and latest_gps_data:
                    sio.emit('update_dashboard', latest_gps_data)
                    data_send_count += 1
                    if data_send_count % 10 == 0:
                        logger.info(f"📡 Sent {data_send_count} GPS updates")

            if stop_event.wait(SEND_INTERVAL):
                break
        except Exception as e:
            logger.error(f"⚠️ Dashboard update error: {e}")
            if stop_event.wait(1.0):
                break

# ========== CAMERA INITIALIZATION ==========
def init_dual_cameras():
    """Initialize both cameras: navigation (Logitech) and underwater (Brica PRO)"""
    global camera_nav, camera_underwater

    # Initialize navigation camera (Logitech)
    try:
        cam_nav = cv2.VideoCapture(CAMERA_NAVIGATION)
        if cam_nav.isOpened():
            cam_nav.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cam_nav.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cam_nav.set(cv2.CAP_PROP_FPS, 30)
            ret, _ = cam_nav.read()
            if ret:
                camera_nav = cam_nav
                logger.info(f"✅ Navigation Camera (index {CAMERA_NAVIGATION}) initialized")
            else:
                cam_nav.release()
                logger.error("❌ Navigation camera read failed")
        else:
            logger.error("❌ Cannot open navigation camera")
    except Exception as e:
        logger.exception(f"❌ Navigation camera error: {e}")

    # Initialize underwater camera (Brica PRO)
    try:
        if USE_BRICA_WIFI:
            logger.info(f"   Trying Brica PRO WiFi stream: {BRICA_WIFI_STREAM}")
            cam_under = cv2.VideoCapture(BRICA_WIFI_STREAM)
        else:
            logger.info(f"   Trying Brica PRO USB mode: index {CAMERA_UNDERWATER}")
            cam_under = cv2.VideoCapture(CAMERA_UNDERWATER)

        if cam_under.isOpened():
            cam_under.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cam_under.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cam_under.set(cv2.CAP_PROP_FPS, 30)
            ret, test_frame = cam_under.read()
            if ret:
                camera_underwater = cam_under
                mode = "WiFi" if USE_BRICA_WIFI else "USB"
                logger.info(f"✅ Underwater Camera (Brica PRO {mode}) initialized")
            else:
                cam_under.release()
                logger.warning("⚠️ Brica PRO read failed (optional)")
        else:
            logger.warning("⚠️ Cannot open Brica PRO (optional)")
    except Exception as e:
        logger.exception(f"⚠️ Brica PRO error (optional): {e}")

    return camera_nav is not None

# ========== BALL DETECTION ==========
def detect_ball(frame_bgr, lower, upper, lower2=None, upper2=None):
    # TAMBAHKAN GAUSSIAN BLUR! (dari kode Anda)
    blurred = cv2.GaussianBlur(frame_bgr, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    if lower2 is not None and upper2 is not None:
        mask = cv2.bitwise_or(cv2.inRange(hsv, lower, upper), cv2.inRange(hsv, lower2, upper2))
    else:
        mask = cv2.inRange(hsv, lower, upper)
    
    kernel = np.ones((5, 5), np.uint8)
    # TAMBAHKAN MORPHOLOGY! (dari kode Anda)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)
        
        # TAMBAHKAN MIN AREA CHECK! (dari kode Anda)
        area = cv2.contourArea(largest)
        if radius > MIN_BALL_RADIUS and area > 500:  # min_area = 500
            return {'x': int(x), 'y': int(y), 'radius': int(radius)}
    return None

def calculate_navigation(red_ball, green_ball):
    """Calculate navigation with gate position validation"""
    if not (red_ball and green_ball):
        return None, None, None, None
    
    # Calculate distance between red and green ball
    dx = red_ball['x'] - green_ball['x']
    dy = red_ball['y'] - green_ball['y']
    ball_distance = math.sqrt(dx**2 + dy**2)
    
    # Validate: balls harus berdekatan tapi tidak terlalu dekat/jauh
    if ball_distance < MIN_BALL_SEPARATION:
        logger.debug(f"⚠️ Balls too close: {ball_distance:.1f}px (min: {MIN_BALL_SEPARATION})")
        return None, None, None, None
    
    if ball_distance > MAX_BALL_SEPARATION:
        logger.debug(f"⚠️ Balls too far: {ball_distance:.1f}px (max: {MAX_BALL_SEPARATION})")
        return None, None, None, None
    
    # Determine gate position (red-green or green-red)
    if red_ball['x'] < green_ball['x']:
        gate_type = "RED_LEFT_GREEN_RIGHT"
    else:
        gate_type = "GREEN_LEFT_RED_RIGHT"
    
    # Calculate midpoint (RAW - belum diratakan)
    mid_x = (red_ball['x'] + green_ball['x']) / 2
    mid_y = (red_ball['y'] + green_ball['y']) / 2
    
    # Calculate heading adjustment
    offset_x = mid_x - CAMERA_WIDTH / 2
    heading_adj = (offset_x / CAMERA_WIDTH) * 60
    heading_adj = max(-30, min(30, heading_adj))
    
    return mid_x, mid_y, heading_adj, gate_type

# ...existing code...

# ========== CAMERA STREAM LOOP ==========
def camera_stream_loop():
    """Main camera streaming loop with obstacle detection"""
    global latest_frame, latest_obstacles, mission_active, last_underwater_photo_time

    if not camera_nav:
        logger.error("❌ No camera available")
        return

    last_send_time = 0
    last_obstacle_send = 0
    frame_count = 0
    OBSTACLE_SEND_INTERVAL = 2.0
    
    # Gate detection state
    last_gate_detection_time = 0
    GATE_DETECTION_COOLDOWN = 3.0
    
    # Gate navigation state
    gate_centered = False
    gate_passed = False
    current_gate_index = 0
    
    # Midpoint smoothing (NEW! - untuk stabilkan deteksi)
    mid_history = []
    max_history = 5  # Rata-rata dari 5 frame terakhir

    # Docking state
    docking_mode = False
    docked = False

    while not stop_event.is_set():
        try:
            ret, frame = camera_nav.read()
            if not ret:
                logger.warning("⚠️ Camera read failed")
                time.sleep(0.1)
                continue

            # Store frame for capture callbacks
            with frame_lock:
                latest_frame = frame.copy()

            # Create clean frame for web display (NO OVERLAY!)
            clean_frame = frame.copy()

            # ========== OBSTACLE DETECTION ==========
            obstacles = detect_obstacles(frame)
            with obstacle_lock:
                latest_obstacles = obstacles

            # ========== BALL DETECTION & AUTO-NAVIGATION ==========
            current_time = time.time()
            
            # Only detect gate if cooldown expired
            if current_time - last_gate_detection_time > GATE_DETECTION_COOLDOWN:
                red_ball = detect_ball(frame, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
                green_ball = detect_ball(frame, GREEN_LOWER, GREEN_UPPER)
                blue_ball = detect_ball(frame, BLUE_LOWER, BLUE_UPPER)

                # Calculate navigation (RAW midpoint)
                mid_x_raw, mid_y, heading_adj, gate_type = calculate_navigation(red_ball, green_ball)
                
                # Smoothing midpoint dengan history buffer (NEW!)
                if mid_x_raw is not None:
                    # Tambah ke history
                    mid_history.append(mid_x_raw)
                    
                    # Keep only last N frames
                    if len(mid_history) > max_history:
                        mid_history.pop(0)
                    
                    # Calculate averaged midpoint (smooth!)
                    mid_x = sum(mid_history) / len(mid_history)
                    
                    logger.debug(f"Midpoint: raw={mid_x_raw:.1f}, smoothed={mid_x:.1f}, history={len(mid_history)}")
                else:
                    # No gate detected, clear history
                    mid_history = []
                    mid_x = None
                
                if mid_x is not None and gate_type is not None:
                    with mission_control_lock:
                        is_mission_active = mission_active
                    
                    if is_mission_active and not gate_passed:
                        # Use SMOOTHED midpoint for navigation!
                        gate_centered = navigate_to_gate_center(mid_x, heading_adj)
                    
                    # Check if gate passed
                    if red_ball and green_ball:
                        avg_radius = (red_ball['radius'] + green_ball['radius']) / 2
                        estimated_distance = (FOCAL_LENGTH * 0.1) / avg_radius if avg_radius > 0 else 999
                        
                        if estimated_distance < 3.0 and is_mission_active:
                            last_gate_detection_time = current_time
                            gate_passed = True
                            current_gate_index += 1
                            
                            # Clear midpoint history after gate passed
                            mid_history = []
                            
                            logger.info(f"🚪 Gate {current_gate_index} passed: {gate_type}")
                            stop_motors()
                            
                            if sio.connected:
                                sio.emit('gate_passed', {
                                    'gate_type': gate_type,
                                    'gate_index': current_gate_index,
                                    'distance': round(estimated_distance, 2),
                                    'timestamp': datetime.now().isoformat()
                                })

            # ========== STREAM VIDEO KE DASHBOARD (CLEAN - NO OVERLAY!) ==========
            current_time = time.time()
            if current_time - last_send_time >= SEND_INTERVAL:
                try:
                    # Encode CLEAN frame (tanpa overlay apapun)
                    _, buffer = cv2.imencode('.jpg', clean_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    jpg_base64 = base64.b64encode(buffer).decode('utf-8')

                    # Kirim ke dashboard via Socket.IO
                    if sio.connected:
                        sio.emit('video_frame', {'image': jpg_base64})
                        frame_count += 1

                    last_send_time = current_time
                except Exception as e:
                    logger.error(f"⚠️ Frame emit error: {e}")

            # Send obstacles to dashboard (less frequent)
            if current_time - last_obstacle_send >= OBSTACLE_SEND_INTERVAL:
                send_obstacles_to_dashboard()
                last_obstacle_send = current_time

            # Log progress every 100 frames
            if frame_count % 100 == 0 and frame_count > 0:
                logger.info(f"📹 Streamed {frame_count} frames to dashboard")

        except Exception as e:
            logger.error(f"❌ camera_stream_loop error: {e}")
            stop_motors()
            time.sleep(0.1)

    logger.info("Camera stream loop stopped")

# ========== OBSTACLE DETECTION ==========
def detect_obstacles(frame_bgr):
    """
    Detect obstacles (NON-BALL objects) using edge detection
    Excludes red & green balls (gate markers)
    Returns: list of obstacles with estimated distance and bearing
    """
    obstacles = []

    try:
        # Convert to grayscale
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection (Canny)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Dilate edges to connect nearby edges
        kernel = np.ones((5, 5), np.uint8)
        edges_dilated = cv2.dilate(edges, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(edges_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create mask untuk filter out red & green balls (gate markers)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LOWER1, RED_UPPER1),
            cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
        )
        mask_green = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        mask_balls = cv2.bitwise_or(mask_red, mask_green)
        mask_balls = cv2.dilate(mask_balls, kernel, iterations=3)  # Expand ball mask

        for contour in contours:
            area = cv2.contourArea(contour)

            # Filter by minimum area
            if area < OBSTACLE_MIN_AREA:
                continue

            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Filter by minimum width (avoid thin vertical lines)
            if w < OBSTACLE_MIN_WIDTH:
                continue
            
            # Check if contour overlaps with ball mask (FILTER OUT BALLS!)
            cx, cy = x + w // 2, y + h // 2
            if mask_balls[cy, cx] > 0:
                # This contour is a ball (red/green), skip it!
                continue

            # Calculate center point
            center_x = cx
            center_y = cy

            # Estimate distance using size-based method
            if w > 0:
                distance = (OBSTACLE_REFERENCE_WIDTH * FOCAL_LENGTH) / w
                distance = min(distance, MAX_OBSTACLE_DISTANCE)
            else:
                distance = MAX_OBSTACLE_DISTANCE

            # Calculate bearing (angle from center)
            offset_from_center = center_x - (CAMERA_WIDTH / 2)
            bearing_deg = (offset_from_center / CAMERA_WIDTH) * 60  # FOV ~60 degrees

            obstacles.append({
                'bbox': (x, y, w, h),
                'center': (center_x, center_y),
                'distance': round(distance, 2),
                'bearing': round(bearing_deg, 1),
                'area': int(area)
            })

        # Sort by distance (closest first)
        obstacles.sort(key=lambda o: o['distance'])

        # Keep only top 3 closest obstacles
        obstacles = obstacles[:3]

    except Exception as e:
        logger.error(f"Obstacle detection error: {e}")

    return obstacles

def calculate_obstacle_gps_position(obstacle_distance, obstacle_bearing):
    """
    Calculate GPS position of obstacle based on ASV position, heading, and obstacle bearing

    Args:
        obstacle_distance: distance in meters
        obstacle_bearing: bearing relative to ASV heading (degrees, -30 to +30)

    Returns:
        (lat, lon) of obstacle or None
    """
    try:
        with gps_data_lock:
            asv_lat = latest_gps_data.get('Latitude_DD')
            asv_lon = latest_gps_data.get('Longitude_DD')
            asv_heading = latest_gps_data.get('COG_deg', '0')

        if not asv_lat or not asv_lon:
            return None

        asv_lat = float(asv_lat)
        asv_lon = float(asv_lon)
        asv_heading = float(asv_heading)

        # Absolute bearing = ASV heading + relative bearing
        absolute_bearing = (asv_heading + obstacle_bearing) % 360

        # Convert to radians
        bearing_rad = math.radians(absolute_bearing)

        # Earth radius in meters
        R = 6371000

        # Calculate new position
        # Formula: Haversine forward calculation
        angular_distance = obstacle_distance / R

        lat1 = math.radians(asv_lat)
        lon1 = math.radians(asv_lon)

        lat2 = math.asin(
            math.sin(lat1) * math.cos(angular_distance) +
            math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing_rad)
        )

        lon2 = lon1 + math.atan2(
            math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat1),
            math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2)
        )

        # Convert back to degrees
        obstacle_lat = math.degrees(lat2)
        obstacle_lon = math.degrees(lon2)

        return (obstacle_lat, obstacle_lon)

    except Exception as e:
        logger.error(f"Calculate obstacle GPS error: {e}")
        return None

def send_obstacles_to_dashboard():
    """Send detected obstacles to dashboard for map visualization"""
    global latest_obstacles

    try:
        with obstacle_lock:
            if not latest_obstacles:
                return

            # Calculate GPS positions for each obstacle
            obstacles_with_gps = []
            for obs in latest_obstacles:
                gps_pos = calculate_obstacle_gps_position(
                    obs['distance'],
                    obs['bearing']
                )

                if gps_pos:
                    obstacles_with_gps.append({
                        'lat': gps_pos[0],
                        'lon': gps_pos[1],
                        'distance': obs['distance'],
                        'bearing': obs['bearing'],
                        'type': 'unknown',  # Could be 'buoy', 'boat', 'debris', etc.
                        'timestamp': datetime.now().isoformat()
                    })

            # Emit to dashboard
            if sio.connected and obstacles_with_gps:
                sio.emit('obstacles_detected', {
                    'obstacles': obstacles_with_gps,
                    'count': len(obstacles_with_gps)
                })
                logger.info(f"📍 Sent {len(obstacles_with_gps)} obstacles to dashboard")

    except Exception as e:
        logger.error(f"Send obstacles error: {e}")

# ========== COMMAND INTERFACE ==========
def command_interface():
    """Interactive command interface"""
    global mission_sm

    logger.info("Command interface started (type 'help' for commands)")
    print("\n🎮 Command Interface Active")
    print("   Commands: help, surface, underwater, status, quit")
    print("   Mission: mission start, mission stop, mission status, mission abort")

    while not stop_event.is_set():
        try:
            cmd = input("\n> ").strip().lower()

            if cmd == "help":
                print("Commands:")
                print("  surface          - Take surface photo manually")
                print("  underwater       - Take underwater photo manually")
                print("  status           - Show system status")
                print("  mission start    - Start autonomous mission")
                print("  mission stop     - Stop mission gracefully")
                print("  mission abort    - Emergency abort mission")
                print("  mission status   - Show mission state")
                print("  quit/exit        - Shutdown system")

            elif cmd == "surface":
                with frame_lock:
                    if latest_frame is not None:
                        save_and_emit_photo(latest_frame.copy(), "surface", 0)
                        print("✅ Surface photo captured")
                    else:
                        print("❌ No frame available")

            elif cmd == "underwater":
                if camera_underwater and camera_underwater.isOpened():
                    ret, frame = camera_underwater.read()
                    if ret:
                        save_and_emit_photo(frame, "underwater", 0)
                        print("✅ Underwater photo captured")
                    else:
                        print("❌ Underwater camera read failed")
                else:
                    print("❌ Underwater camera not available")

            elif cmd == "status":
                with gps_data_lock:
                    print(f"   GPS: {latest_gps_data.get('Latitude_DD', 'N/A')}, {latest_gps_data.get('Longitude_DD', 'N/A')}")
                    print(f"   Speed: {latest_gps_data.get('SOG_knot', 'N/A')} knots")
                    print(f"   Heading: {latest_gps_data.get('COG_deg', 'N/A')}°")
                    print(f"   Battery: {latest_gps_data.get('Battery_', 'N/A')}%")
                print(f"   Camera Nav: {'OK' if camera_nav else 'N/A'}")
                print(f"   Camera Under: {'OK' if camera_underwater else 'N/A'}")
                print(f"   Socket.IO: {'Connected' if sio.connected else 'Disconnected'}")

            elif cmd == "mission start":
                if mission_sm and mission_sm.state.name != 'IDLE':
                    print("❌ Mission already running")
                else:
                    mission_sm = MissionStateMachine(
                        gates=GATES_LIST,
                        config=MISSION_CFG,
                        on_capture=on_capture_callback,
                        on_lower=on_lower_callback,
                        on_raise=on_raise_callback,
                        on_navigate=on_navigate_callback,
                        emit_event=on_emit_event,
                    )
                    mission_sm.start()
                    print("✅ Mission STARTED!")

            elif cmd == "mission stop":
                if mission_sm:
                    mission_sm.stop()
                    print("✅ Mission STOPPED")
                else:
                    print("❌ No mission running")

            elif cmd == "mission abort":
                if mission_sm:
                    mission_sm.abort("Manual abort from command interface")
                    print("⚠️ Mission ABORTED")
                else:
                    print("❌ No mission running")

            elif cmd == "mission status":
                if mission_sm:
                    print(f"   State: {mission_sm.state.name}")
                    print(f"   Gate: {mission_sm.current_gate_idx}/{len(GATES_LIST)-1}")
                    print(f"   Position: ({mission_sm.lat:.6f}, {mission_sm.lon:.6f})" if mission_sm.lat else "   Position: N/A")
                    print(f"   Speed: {mission_sm.speed:.2f} m/s")
                    print(f"   Heading: {mission_sm.heading:.1f}°")
                else:
                    print("❌ Mission not initialized")

            elif cmd in ["quit", "exit"]:
                print("🛑 Shutting down...")
                stop_event.set()
                break

            else:
                print(f"❌ Unknown command: {cmd}")

        except EOFError:
            stop_event.set()
            break
        except Exception:
            logger.exception("❌ command_interface error")
            time.sleep(0.2)

# ========== MAIN ==========
def main():
    global SOCKETIO_SERVER, CAMERA_NAVIGATION, CAMERA_UNDERWATER, SEND_FPS, mission_sm

    parser = argparse.ArgumentParser(description='SAIBATIN AZURA 1.0')
    parser.add_argument('--server', default=SOCKETIO_SERVER, help='Socket.IO server URL')
    parser.add_argument('--camera-nav', type=int, default=CAMERA_NAVIGATION, help='Navigation camera index')
    parser.add_argument('--camera-under', type=int, default=CAMERA_UNDERWATER, help='Underwater camera index')
    parser.add_argument('--send-fps', type=int, default=SEND_FPS, help='Video streaming FPS')
    args = parser.parse_args()

    SOCKETIO_SERVER = args.server
    CAMERA_NAVIGATION = args.camera_nav
    CAMERA_UNDERWATER = args.camera_under  # ← FIX: Ini yang hilang!
    SEND_FPS = max(1, args.send_fps)      # ← FIX: Ini yang hilang!

    logger.info("🚀 SAIBATIN AZURA 1.0 - STARTING (with Mission SM)")
    logger.info("=" * 60)

    # Connect to dashboard (background)
    logger.info("📡 Starting background connection to Dashboard...")
    threading.Thread(target=connect_to_server_background, daemon=True).start()

    # Connect to Pixhawk
    logger.info("🔌 Connecting to Pixhawk...")
    connect_pixhawk()
    
    # Initialize GPIO motors (NEW!)
    logger.info("⚡ Initializing GPIO motor control...")
    init_gpio_motors()

    # Initialize cameras
    logger.info("🎥 Initializing cameras...")
    if not init_dual_cameras():
        logger.error("❌ Failed to initialize navigation camera - cannot proceed")
        return

    # Start GPS thread
    gps_thread = threading.Thread(target=read_gps_data, daemon=True)
    gps_thread.start()
    logger.info("✅ GPS thread started")

    # Start dashboard update thread
    dash_thread = threading.Thread(target=send_dashboard_update, daemon=True)
    dash_thread.start()
    logger.info("✅ Dashboard update thread started")
    
    # Start telemetry forwarder
    tf_thread = threading.Thread(target=telemetry_forwarder, daemon=True)
    tf_thread.start()
    logger.info("✅ Telemetry forwarder started")
    
    # Initialize mission state machine
    mission_sm = MissionStateMachine(
        gates=GATES_LIST,
        config=MISSION_CFG,
        on_capture=on_capture_callback,
        on_lower=on_lower_callback,
        on_raise=on_raise_callback,
        on_navigate=on_navigate_callback,
        emit_event=on_emit_event,
    )
    logger.info("✅ Mission state machine initialized (not started yet)")

    # Start command interface thread
    cmd_thread = threading.Thread(target=command_interface, daemon=True)
    cmd_thread.start()
    logger.info("✅ Command interface started")

    # Start camera stream loop in main thread
    try:
        logger.info("🎥 Camera streaming started")
        camera_stream_loop()
    except KeyboardInterrupt:
        logger.info("🛑 Ctrl+C received - shutting down...")
        stop_event.set()
    except Exception:
        logger.exception("❌ main loop unexpected error")
        stop_event.set()
    finally:
        # Signal threads to stop
        stop_event.set()
        time.sleep(0.5)
        
        # Stop mission state machine
        if mission_sm:
            try:
                mission_sm.stop()
                logger.info("✅ Mission state machine stopped")
            except Exception:
                pass
        
        # Release cameras
        if camera_nav:
            camera_nav.release()
            logger.info("✅ Navigation camera released")
        if camera_underwater:
            camera_underwater.release()
            logger.info("✅ Underwater camera released")
        
        # Disconnect socket
        if sio.connected:
            sio.disconnect()
            logger.info("✅ Disconnected from dashboard")
        
        # Cleanup GPIO motors (NEW!)
        if GPIO_AVAILABLE and pwm_left_gpio and pwm_right_gpio:
            try:
                pwm_left_gpio.stop()
                pwm_right_gpio.stop()
                GPIO.cleanup()
                logger.info("✅ GPIO motors cleaned up")
            except Exception:
                pass
        
        logger.info("👋 Shutdown complete")

if __name__ == "__main__":
    main()