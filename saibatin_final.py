#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - FINAL VERSION FOR COMPETITION
Mode: LINTASAN-A (3→4→3 obstacles, RED=right GREEN=left)
Full autonomous: START → 10 obstacles → return to START
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
        logging.FileHandler('saibatin.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# ========== CONFIG ==========
SOCKETIO_SERVER = "http://saibatinazura.site:5000"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_INDEX = 0

# Motor GPIO
MOTOR_LEFT_GPIO = 18
MOTOR_RIGHT_GPIO = 13
PWM_FREQ = 50

# Ball detection (OUTDOOR optimized)
RED_LOWER1 = np.array([0, 100, 120])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 100, 120])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([40, 80, 80])
GREEN_UPPER = np.array([80, 255, 255])

MIN_BALL_RADIUS = 12
MIN_CIRCULARITY = 0.6
MIN_BALL_SEPARATION = 25
MAX_BALL_SEPARATION = 450

# Navigation
OFFSET_THRESHOLD = 40
BASE_SPEED = 55
TURN_SPEED = 0.7

# LINTASAN-A structure
SEGMENT_OBSTACLES = [3, 4, 3]
TOTAL_OBSTACLES = sum(SEGMENT_OBSTACLES)

# Mission state
mission_active = False
mission_lock = threading.Lock()
stop_event = threading.Event()

current_segment = 0
obstacles_in_segment = 0
total_obstacles_passed = 0

# Globals
sio = socketio.Client()
camera = None
pwm_left = None
pwm_right = None
latest_frame = None
frame_lock = threading.Lock()

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
    attempt = 0
    while not stop_event.is_set():
        try:
            attempt += 1
            logger.info(f"🔌 Connecting to dashboard (attempt {attempt})...")
            sio.connect(SOCKETIO_SERVER, wait=True, transports=['websocket'])
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
        
        pwm_left.start(7.5)
        pwm_right.start(7.5)
        
        logger.info("✅ Motors initialized")
        return True
    except Exception as e:
        logger.error(f"❌ Motor init: {e}")
        return False

def set_motor(pwm, speed_percent):
    if pwm:
        duty = 7.5 + (speed_percent / 100.0) * 2.5
        duty = max(5.0, min(10.0, duty))
        pwm.ChangeDutyCycle(duty)

def stop_motors():
    set_motor(pwm_left, 0)
    set_motor(pwm_right, 0)

def drive(throttle, steering):
    if steering < 0:
        left = throttle * (1 + steering / 100.0)
        right = throttle
    elif steering > 0:
        left = throttle
        right = throttle * (1 - steering / 100.0)
    else:
        left = throttle
        right = throttle
    
    left = max(-100, min(100, left))
    right = max(-100, min(100, right))
    
    set_motor(pwm_left, left)
    set_motor(pwm_right, right)

# ========== BALL DETECTION ==========
def detect_ball(frame, lower, upper, lower2=None, upper2=None):
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
        if area < 400:
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

def validate_gate(red_ball, green_ball):
    if not (red_ball and green_ball):
        return False, None, None
    
    dx = red_ball['x'] - green_ball['x']
    dy = red_ball['y'] - green_ball['y']
    distance_px = math.sqrt(dx**2 + dy**2)
    
    if distance_px < MIN_BALL_SEPARATION or distance_px > MAX_BALL_SEPARATION:
        return False, None, None
    
    if red_ball['x'] <= green_ball['x']:
        logger.warning(f"⚠️ Gate orientation: RED={red_ball['x']}, GREEN={green_ball['x']}")
    
    mid_x = (red_ball['x'] + green_ball['x']) / 2
    avg_radius = (red_ball['radius'] + green_ball['radius']) / 2
    distance = 600.0 / avg_radius if avg_radius > 0 else 999
    
    return True, mid_x, distance

def navigate_to_gate(mid_x, distance):
    offset = mid_x - (CAMERA_WIDTH / 2)
    steering = (offset / (CAMERA_WIDTH / 2)) * 100
    steering = max(-100, min(100, steering))
    
    if abs(offset) > 120:
        throttle = BASE_SPEED * TURN_SPEED
    elif abs(offset) > 60:
        throttle = BASE_SPEED * 0.85
    else:
        throttle = BASE_SPEED
    
    if distance < 3.5:
        throttle *= 0.6
    elif distance < 5.0:
        throttle *= 0.75
    
    drive(throttle, steering)
    logger.info(f"🎯 offset={offset:.0f}px, dist={distance:.1f}m, steer={steering:.0f}%")
    return abs(offset) < OFFSET_THRESHOLD

# ========== SEGMENT TRANSITION ==========
def move_to_next_segment():
    global current_segment, obstacles_in_segment
    
    current_segment += 1
    obstacles_in_segment = 0
    
    logger.info(f"🔄 Moving to SEGMENT {current_segment + 1}")
    
    # Move LEFT
    lateral_time = 8.0
    logger.info("   Lateral movement LEFT...")
    drive(40, -50)
    time.sleep(lateral_time)
    
    stop_motors()
    time.sleep(1.0)
    
    # Sweep
    logger.info("   Sweeping for obstacles...")
    for _ in range(3):
        drive(0, 30)
        time.sleep(1.5)
        stop_motors()
        time.sleep(0.5)
    
    logger.info(f"✅ Ready for SEGMENT {current_segment + 1}")

def return_to_start():
    logger.info("🏁 Returning to START/FINISH...")
    
    drive(-50, 0)
    time.sleep(5.0)
    
    drive(0, 80)
    time.sleep(4.0)
    
    drive(60, 0)
    time.sleep(8.0)
    
    stop_motors()
    logger.info("🏁 ARRIVED AT START/FINISH!")

# ========== PHOTO CAPTURE ==========
def capture_photo(frame, num):
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"obstacle{num:02d}_{timestamp}.jpg"
        filepath = f"captures/{filename}"
        
        os.makedirs("captures", exist_ok=True)
        cv2.imwrite(filepath, frame)
        logger.info(f"📸 {filename}")
        
        if sio.connected:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            jpg_base64 = base64.b64encode(buffer).decode('utf-8')
            sio.emit('photo_captured', {
                'type': 'surface',
                'obstacle': num,
                'filename': filename,
                'image': jpg_base64,
                'timestamp': timestamp
            })
    except Exception as e:
        logger.error(f"❌ Photo: {e}")

# ========== MAIN MISSION LOOP ==========
def mission_loop():
    global latest_frame, mission_active
    global current_segment, obstacles_in_segment, total_obstacles_passed
    
    logger.info("🚀 LINTASAN-A Mission Started")
    logger.info(f"   Structure: {SEGMENT_OBSTACLES} obstacles ({TOTAL_OBSTACLES} total)")
    
    mid_history = []
    max_history = 5
    
    last_obstacle_time = 0
    obstacle_cooldown = 4.0
    
    no_detection_count = 0
    max_no_detection = 30
    
    frame_count = 0
    
    while not stop_event.is_set():
        try:
            ret, frame = camera.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            with frame_lock:
                latest_frame = frame.copy()
            
            with mission_lock:
                is_active = mission_active
            
            if not is_active:
                # Standby mode
                if sio.connected and frame_count % 3 == 0:
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    jpg_base64 = base64.b64encode(buffer).decode('utf-8')
                    sio.emit('video_frame', {'image': jpg_base64})
                
                frame_count += 1
                time.sleep(0.1)
                continue
            
            # === ACTIVE MISSION ===
            red_ball = detect_ball(frame, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
            green_ball = detect_ball(frame, GREEN_LOWER, GREEN_UPPER)
            
            gate_valid, mid_x_raw, distance = validate_gate(red_ball, green_ball)
            
            if not gate_valid:
                no_detection_count += 1
                
                if no_detection_count > max_no_detection:
                    logger.warning("⚠️ No gate detected - sweeping...")
                    drive(0, 30)
                    time.sleep(0.5)
                    no_detection_count = 0
                
                stop_motors()
                time.sleep(0.05)
                continue
            
            no_detection_count = 0
            
            # Smooth midpoint
            mid_history.append(mid_x_raw)
            if len(mid_history) > max_history:
                mid_history.pop(0)
            mid_x = sum(mid_history) / len(mid_history)
            
            # Navigate
            navigate_to_gate(mid_x, distance)
            
            # Check if passed through
            current_time = time.time()
            if distance < 2.0 and (current_time - last_obstacle_time) > obstacle_cooldown:
                obstacles_in_segment += 1
                total_obstacles_passed += 1
                last_obstacle_time = current_time
                mid_history = []
                
                logger.info("=" * 60)
                logger.info(f"🚪✅ OBSTACLE {total_obstacles_passed}/{TOTAL_OBSTACLES} PASSED!")
                logger.info(f"   Segment {current_segment + 1}: {obstacles_in_segment}/{SEGMENT_OBSTACLES[current_segment]}")
                logger.info("=" * 60)
                
                stop_motors()
                time.sleep(0.3)
                
                capture_photo(frame, total_obstacles_passed)
                
                if sio.connected:
                    sio.emit('obstacle_passed', {
                        'obstacle_num': total_obstacles_passed,
                        'segment': current_segment + 1,
                        'distance': round(distance, 2),
                        'timestamp': datetime.now().isoformat()
                    })
                
                time.sleep(1.5)
                
                # Check segment completion
                if obstacles_in_segment >= SEGMENT_OBSTACLES[current_segment]:
                    logger.info(f"✅ SEGMENT {current_segment + 1} COMPLETE!")
                    
                    if current_segment < len(SEGMENT_OBSTACLES) - 1:
                        move_to_next_segment()
                    else:
                        logger.info("=" * 60)
                        logger.info("🏆 ALL SEGMENTS COMPLETE!")
                        logger.info("=" * 60)
                        return_to_start()
                        
                        with mission_lock:
                            mission_active = False
                        
                        if sio.connected:
                            sio.emit('mission_complete', {
                                'total_obstacles': total_obstacles_passed,
                                'timestamp': datetime.now().isoformat()
                            })
                        
                        logger.info("🎉 MISSION SUCCESS! 🎉")
                        break
            
            # Stream video
            if sio.connected and frame_count % 2 == 0:
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                jpg_base64 = base64.b64encode(buffer).decode('utf-8')
                sio.emit('video_frame', {'image': jpg_base64})
            
            frame_count += 1
            
            if frame_count % 50 == 0:
                logger.info(f"📹 Frame {frame_count}, Progress: {total_obstacles_passed}/{TOTAL_OBSTACLES}")
        
        except Exception as e:
            logger.error(f"❌ Mission error: {e}")
            stop_motors()
            time.sleep(0.5)

# ========== MAIN ==========
def main():
    global camera
    
    logger.info("=" * 60)
    logger.info("🚀 SAIBATIN AZURA 1.0 - COMPETITION FINAL")
    logger.info("   Mode: LINTASAN-A (3→4→3 obstacles)")
    logger.info("   RED=right, GREEN=left")
    logger.info("=" * 60)
    
    try:
        # Dashboard connection
        threading.Thread(target=connect_dashboard, daemon=True).start()
        
        # Motors
        logger.info("⚡ Initializing motors...")
        if not init_motors():
            logger.error("❌ Motor init failed")
            return
        
        # Camera
        logger.info("🎥 Initializing camera...")
        camera = cv2.VideoCapture(CAMERA_INDEX)
        if not camera.isOpened():
            logger.error("❌ Camera failed")
            return
        
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        camera.set(cv2.CAP_PROP_FPS, 30)
        
        logger.info("✅ Camera OK")
        
        logger.info("\n" + "=" * 60)
        logger.info("✅ SYSTEM READY FOR COMPETITION!")
        logger.info("   Commands:")
        logger.info("   • Type 'start' to begin mission")
        logger.info("   • Type 'stop' to stop mission")
        logger.info("   • Type 'quit' to exit")
        logger.info("   • Or use dashboard START button")
        logger.info("=" * 60 + "\n")
        
        # Command interface
        def command_interface():
            global mission_active
            while not stop_event.is_set():
                try:
                    cmd = input("> ").strip().lower()
                    if cmd == 'start':
                        with mission_lock:
                            mission_active = True
                        logger.info("🚀 MISSION STARTED!")
                    elif cmd == 'stop':
                        with mission_lock:
                            mission_active = False
                        stop_motors()
                        logger.info("🛑 MISSION STOPPED")
                    elif cmd == 'quit':
                        stop_event.set()
                        break
                    elif cmd == 'status':
                        with mission_lock:
                            status = "ACTIVE" if mission_active else "STANDBY"
                        logger.info(f"Status: {status}, Obstacles: {total_obstacles_passed}/{TOTAL_OBSTACLES}")
                except:
                    break
        
        threading.Thread(target=command_interface, daemon=True).start()
        
        # Run mission
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
