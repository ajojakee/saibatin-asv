#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - COMPETITION SIMPLE MODE
Fokus: Deteksi gate (red+green balls) → Navigate ke tengah → Pass through
NO complex GPS waypoints, just vision-guided gate passing!
"""

import cv2
import numpy as np
import socketio
import base64
import time
import threading
from datetime import datetime
import math
import sys
import logging
import os

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# ========== LOGGING ==========
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler('saibatin_competition.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# ========== CONFIG ==========
SOCKETIO_SERVER = "http://saibatinazura.site:5000"
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Motor GPIO
MOTOR_LEFT_GPIO = 18
MOTOR_RIGHT_GPIO = 13
PWM_FREQ = 50

# Ball detection (TUNE DI ARENA!)
RED_LOWER1 = np.array([0, 120, 150])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 150])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([45, 100, 100])
GREEN_UPPER = np.array([75, 255, 255])

MIN_BALL_RADIUS = 15
MIN_CIRCULARITY = 0.65  # Relax sedikit untuk outdoor
MIN_BALL_SEPARATION = 30
MAX_BALL_SEPARATION = 400

# Navigation
OFFSET_THRESHOLD = 30  # pixels - gate dianggap centered
BASE_SPEED = 50  # percent
TURN_SPEED_MULTIPLIER = 0.6  # reduce speed saat belok

# Mission control
mission_active = False
mission_lock = threading.Lock()
stop_event = threading.Event()

# Globals
sio = socketio.Client()
camera = None
pwm_left = None
pwm_right = None
latest_frame = None
frame_lock = threading.Lock()
gate_count = 0

# ========== SOCKET.IO ==========
@sio.event
def connect():
    logger.info("✅ Connected to Dashboard")

@sio.event
def disconnect():
    logger.warning("❌ Disconnected from Dashboard")

@sio.event
def mission_command(data):
    global mission_active
    command = data.get('command')
    logger.info(f"📥 Command: {command}")
    
    with mission_lock:
        if command == 'start':
            mission_active = True
            logger.info("🚀 MISSION STARTED!")
        elif command in ['stop', 'abort']:
            mission_active = False
            stop_motors()
            logger.info("🛑 MISSION STOPPED")

def connect_dashboard():
    """Background connection to dashboard"""
    attempt = 0
    while not stop_event.is_set():
        try:
            attempt += 1
            logger.info(f"🔌 Connecting to dashboard (attempt {attempt})...")
            sio.connect(SOCKETIO_SERVER, wait=True, transports=['websocket'])
            logger.info("✅ Connected!")
            return
        except Exception as e:
            wait = min(30, 2 ** (attempt - 1))
            logger.warning(f"⚠️ Failed: {e}. Retry in {wait}s")
            if stop_event.wait(wait):
                break

# ========== MOTOR CONTROL ==========
def init_motors():
    global pwm_left, pwm_right
    if not GPIO_AVAILABLE:
        logger.warning("⚠️ GPIO not available")
        return False
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(MOTOR_LEFT_GPIO, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_GPIO, GPIO.OUT)
        
        pwm_left = GPIO.PWM(MOTOR_LEFT_GPIO, PWM_FREQ)
        pwm_right = GPIO.PWM(MOTOR_RIGHT_GPIO, PWM_FREQ)
        
        pwm_left.start(7.5)  # Neutral
        pwm_right.start(7.5)
        
        logger.info("✅ Motors initialized")
        return True
    except Exception as e:
        logger.error(f"❌ Motor init error: {e}")
        return False

def set_motor_speed(pwm, speed_percent):
    """Set motor speed (-100 to +100)"""
    if pwm:
        duty = 7.5 + (speed_percent / 100.0) * 2.5
        duty = max(5.0, min(10.0, duty))
        pwm.ChangeDutyCycle(duty)

def stop_motors():
    """Stop both motors"""
    set_motor_speed(pwm_left, 0)
    set_motor_speed(pwm_right, 0)

def drive(throttle, steering):
    """
    Differential drive
    throttle: -100 to +100
    steering: -100 (full left) to +100 (full right)
    """
    if steering < 0:  # Turn left
        left = throttle * (1 + steering / 100.0)
        right = throttle
    elif steering > 0:  # Turn right
        left = throttle
        right = throttle * (1 - steering / 100.0)
    else:  # Straight
        left = throttle
        right = throttle
    
    left = max(-100, min(100, left))
    right = max(-100, min(100, right))
    
    set_motor_speed(pwm_left, left)
    set_motor_speed(pwm_right, right)
    
    logger.debug(f"🚢 Drive: T={throttle}% S={steering}% | L={left:.0f}% R={right:.0f}%")

# ========== BALL DETECTION ==========
def detect_ball(frame, lower, upper, lower2=None, upper2=None):
    """Detect ball and return center position"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (7, 7), 0)
    
    if lower2 is not None:
        mask = cv2.bitwise_or(
            cv2.inRange(blurred, lower, upper),
            cv2.inRange(blurred, lower2, upper2)
        )
    else:
        mask = cv2.inRange(blurred, lower, upper)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_ball = None
    best_score = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 500:
            continue
        
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        
        if radius < MIN_BALL_RADIUS or circularity < MIN_CIRCULARITY:
            continue
        
        score = circularity * area
        
        if score > best_score:
            best_score = score
            best_ball = {'x': int(x), 'y': int(y), 'radius': int(radius)}
    
    return best_ball

def detect_gate(frame):
    """
    Detect gate (red + green balls) and return navigation info
    Returns: (midpoint_x, distance_estimate, gate_valid)
    """
    red_ball = detect_ball(frame, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
    green_ball = detect_ball(frame, GREEN_LOWER, GREEN_UPPER)
    
    if not (red_ball and green_ball):
        return None, None, False
    
    # Validate gate
    dx = red_ball['x'] - green_ball['x']
    dy = red_ball['y'] - green_ball['y']
    distance = math.sqrt(dx**2 + dy**2)
    
    if distance < MIN_BALL_SEPARATION or distance > MAX_BALL_SEPARATION:
        return None, None, False
    
    # Calculate midpoint
    mid_x = (red_ball['x'] + green_ball['x']) / 2
    
    # Estimate distance (simple: larger balls = closer)
    avg_radius = (red_ball['radius'] + green_ball['radius']) / 2
    distance_estimate = 600.0 / avg_radius if avg_radius > 0 else 999  # focal_length / radius
    
    return mid_x, distance_estimate, True

# ========== NAVIGATION ==========
def navigate_to_gate(mid_x, distance):
    """
    Navigate ASV to gate center
    Returns: True if gate is centered, False otherwise
    """
    offset = mid_x - (CAMERA_WIDTH / 2)
    
    # Calculate steering (-100 to +100)
    steering = (offset / (CAMERA_WIDTH / 2)) * 100
    steering = max(-100, min(100, steering))
    
    # Calculate throttle based on offset
    if abs(offset) > 100:  # Far from center
        throttle = BASE_SPEED * TURN_SPEED_MULTIPLIER
    elif abs(offset) > 50:  # Medium offset
        throttle = BASE_SPEED * 0.8
    else:  # Centered
        throttle = BASE_SPEED
    
    # Slow down if very close
    if distance < 3.0:
        throttle *= 0.5
    
    drive(throttle, steering)
    
    logger.info(f"🎯 Gate: offset={offset:.0f}px, dist={distance:.1f}m, steer={steering:.0f}%")
    
    return abs(offset) < OFFSET_THRESHOLD

# ========== PHOTO CAPTURE ==========
def capture_photo(frame, gate_number):
    """Capture and save photo"""
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"gate{gate_number}_surface_{timestamp}.jpg"
        filepath = f"captures/{filename}"
        
        os.makedirs("captures", exist_ok=True)
        cv2.imwrite(filepath, frame)
        logger.info(f"📸 Saved: {filename}")
        
        # Send to dashboard
        if sio.connected:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            jpg_base64 = base64.b64encode(buffer).decode('utf-8')
            sio.emit('photo_captured', {
                'type': 'surface',
                'gate': gate_number,
                'filename': filename,
                'image': jpg_base64,
                'timestamp': timestamp
            })
    except Exception as e:
        logger.error(f"❌ Photo error: {e}")

# ========== MAIN MISSION LOOP ==========
def mission_loop():
    """Main autonomous mission loop"""
    global latest_frame, gate_count, mission_active
    
    if not camera:
        logger.error("❌ No camera!")
        return
    
    logger.info("🎥 Mission loop started")
    
    # Smoothing buffer
    mid_history = []
    max_history = 5
    
    # State
    last_gate_time = 0
    gate_cooldown = 5.0  # seconds between gates
    
    frame_count = 0
    
    while not stop_event.is_set():
        try:
            ret, frame = camera.read()
            if not ret:
                logger.warning("⚠️ Camera read failed")
                time.sleep(0.1)
                continue
            
            with frame_lock:
                latest_frame = frame.copy()
            
            # Check if mission active
            with mission_lock:
                is_active = mission_active
            
            if not is_active:
                # Standby mode - just stream video
                if sio.connected and frame_count % 3 == 0:  # Send every 3rd frame
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    jpg_base64 = base64.b64encode(buffer).decode('utf-8')
                    sio.emit('video_frame', {'image': jpg_base64})
                
                frame_count += 1
                time.sleep(0.1)
                continue
            
            # === ACTIVE MISSION MODE ===
            
            # Detect gate
            mid_x_raw, distance, gate_valid = detect_gate(frame)
            
            if not gate_valid:
                # No gate detected - stop and wait
                stop_motors()
                logger.debug("❌ No gate detected")
                time.sleep(0.1)
                continue
            
            # Smooth midpoint
            mid_history.append(mid_x_raw)
            if len(mid_history) > max_history:
                mid_history.pop(0)
            mid_x = sum(mid_history) / len(mid_history)
            
            # Navigate to gate
            centered = navigate_to_gate(mid_x, distance)
            
            # Check if passed through gate
            current_time = time.time()
            if distance < 2.5 and (current_time - last_gate_time) > gate_cooldown:
                # Gate passed!
                gate_count += 1
                last_gate_time = current_time
                mid_history = []  # Clear smoothing
                
                logger.info(f"🚪✅ GATE {gate_count} PASSED!")
                
                # Stop briefly for photo
                stop_motors()
                time.sleep(0.5)
                
                # Capture photo
                capture_photo(frame, gate_count)
                
                # Send event to dashboard
                if sio.connected:
                    sio.emit('gate_passed', {
                        'gate_index': gate_count,
                        'distance': round(distance, 2),
                        'timestamp': datetime.now().isoformat()
                    })
                
                # Brief pause before next gate
                time.sleep(2.0)
            
            # Stream video to dashboard
            if sio.connected and frame_count % 2 == 0:
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                jpg_base64 = base64.b64encode(buffer).decode('utf-8')
                sio.emit('video_frame', {'image': jpg_base64})
            
            frame_count += 1
            
            if frame_count % 100 == 0:
                logger.info(f"📹 Frame {frame_count}, Gates: {gate_count}")
        
        except Exception as e:
            logger.error(f"❌ Mission loop error: {e}")
            stop_motors()
            time.sleep(0.5)
    
    logger.info("Mission loop stopped")

# ========== MAIN ==========
def main():
    global camera
    
    logger.info("🚀 SAIBATIN AZURA 1.0 - COMPETITION MODE")
    logger.info("=" * 60)
    
    try:
        # Connect to dashboard (background)
        threading.Thread(target=connect_dashboard, daemon=True).start()
        
        # Init motors
        logger.info("⚡ Initializing motors...")
        if not init_motors():
            logger.error("❌ Motor init failed - cannot proceed")
            return
        
        # Init camera
        logger.info("🎥 Initializing camera...")
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if not camera.isOpened():
            logger.error("❌ Cannot open camera")
            return
        
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, 30)
        
        ret, _ = camera.read()
        if not ret:
            logger.error("❌ Camera read failed")
            return
        
        logger.info("✅ Camera OK")
        
        logger.info("\n" + "=" * 60)
        logger.info("✅ SYSTEM READY FOR COMPETITION!")
        logger.info("   Waiting for START command from dashboard...")
        logger.info("   Or type 'start' here to begin mission")
        logger.info("=" * 60 + "\n")
        
        # Command interface thread
        def command_interface():
            global mission_active
            while not stop_event.is_set():
                try:
                    cmd = input("> ").strip().lower()
                    if cmd == 'start':
                        with mission_lock:
                            mission_active = True
                        logger.info("🚀 MISSION STARTED (manual)")
                    elif cmd in ['stop', 'quit']:
                        with mission_lock:
                            mission_active = False
                        stop_motors()
                        logger.info("🛑 MISSION STOPPED")
                        if cmd == 'quit':
                            stop_event.set()
                            break
                    elif cmd == 'status':
                        with mission_lock:
                            status = "ACTIVE" if mission_active else "STANDBY"
                        logger.info(f"Status: {status}, Gates: {gate_count}")
                except:
                    break
        
        threading.Thread(target=command_interface, daemon=True).start()
        
        # Run mission loop
        mission_loop()
    
    except KeyboardInterrupt:
        logger.info("\n🛑 Ctrl+C pressed")
    
    except Exception as e:
        logger.exception(f"❌ Fatal error: {e}")
    
    finally:
        logger.info("🧹 Cleanup...")
        stop_event.set()
        stop_motors()
        
        if camera:
            camera.release()
        
        if sio.connected:
            sio.disconnect()
        
        if GPIO_AVAILABLE:
            try:
                if pwm_left:
                    pwm_left.stop()
                if pwm_right:
                    pwm_right.stop()
                GPIO.cleanup()
            except:
                pass
        
        logger.info("👋 Shutdown complete")

if __name__ == "__main__":
    main()
