#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - FINAL COMPETITION VERSION
Default Mode: LINTASAN-A (3→4→3 obstacles, RED=right GREEN=left)
+ Emergency Return to Start feature
+ Dashboard return command
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
# DASHBOARD SERVER - CHOOSE ONE:

# MODE 1: PRODUCTION (Hosting - untuk lomba)
SOCKETIO_SERVER = "http://saibatinazura.site:5000"  # ← UNCOMMENT untuk lomba

# MODE 2: LOCAL TESTING (Laptop - untuk development)
# SOCKETIO_SERVER = "http://10.115.96.157:5000"  # ← UNCOMMENT untuk test local

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_NAVIGATION = 0

MOTOR_LEFT_GPIO = 18
MOTOR_RIGHT_GPIO = 13
PWM_FREQ_GPIO = 50

# Motor capabilities (IMPORTANT!)
MOTOR_FORWARD_ONLY = True  # ← Motors CANNOT reverse (brushless ESC)

# Ball detection
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

OFFSET_THRESHOLD = 40
BASE_SPEED = 55
TURN_SPEED = 0.7

SEGMENT_OBSTACLES = [3, 4, 3]
TOTAL_OBSTACLES = sum(SEGMENT_OBSTACLES)

# Emergency return parameters
MAX_NO_GATE_TIME = 60.0  # seconds - if no gate detected for 60s, return to start
MAX_STUCK_TIME = 90.0    # seconds - if stuck at same obstacle, return to start
EMERGENCY_RETURN_ENABLED = True

mission_active = False
mission_lock = threading.Lock()
stop_event = threading.Event()

current_segment = 0
obstacles_in_segment = 0
total_obstacles_passed = 0

sio = socketio.Client()
camera_nav = None
pwm_left_gpio = None
pwm_right_gpio = None
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
            if sio.connected:
                sio.emit('mission_status_update', {
                    'status': 'active',
                    'message': 'Mission started - navigating to obstacles'
                })
        
        elif command == 'stop':
            mission_active = False
            stop_motors()
            logger.info("🛑 MISSION STOPPED")
            if sio.connected:
                sio.emit('mission_status_update', {
                    'status': 'stopped',
                    'message': 'Mission stopped by user'
                })
        
        elif command == 'return':
            # Return to start command from dashboard
            logger.info("🏁 RETURN TO START command received from dashboard")
            mission_active = False
            stop_motors()
            time.sleep(0.5)
            emergency_return_to_start()
            
            if sio.connected:
                sio.emit('mission_status_update', {
                    'status': 'returned',
                    'message': 'ASV returned to START position'
                })
        
        elif command == 'abort':
            # Emergency abort
            logger.warning("⚠️ EMERGENCY ABORT from dashboard")
            mission_active = False
            stop_motors()
            
            if sio.connected:
                sio.emit('mission_status_update', {
                    'status': 'aborted',
                    'message': 'Mission aborted - motors stopped'
                })

def connect_to_server_background(backoff_base=1.0, max_backoff=30.0):
    attempt = 0
    while not stop_event.is_set():
        try:
            if sio.connected:
                return
            attempt += 1
            logger.info(f"🔌 Connecting (attempt {attempt})...")
            sio.connect(SOCKETIO_SERVER, wait=True, transports=['websocket'])
            return
        except Exception as e:
            wait = min(max_backoff, backoff_base * (2 ** (attempt - 1)))
            logger.warning(f"⚠️ Failed: {e}. Retry in {wait}s")
            if stop_event.wait(wait):
                break

# ========== MOTOR CONTROL ==========
def init_motors():
    global pwm_left_gpio, pwm_right_gpio
    if not GPIO_AVAILABLE:
        logger.warning("⚠️ GPIO not available")
        return False
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(MOTOR_LEFT_GPIO, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_GPIO, GPIO.OUT)
        
        pwm_left_gpio = GPIO.PWM(MOTOR_LEFT_GPIO, PWM_FREQ_GPIO)
        pwm_right_gpio = GPIO.PWM(MOTOR_RIGHT_GPIO, PWM_FREQ_GPIO)
        
        pwm_left_gpio.start(7.5)
        pwm_right_gpio.start(7.5)
        
        logger.info("✅ Motors initialized")
        return True
    except Exception as e:
        logger.error(f"❌ Motor init: {e}")
        return False

def set_motor(pwm, speed_percent):
    """
    Set motor speed (FORWARD ONLY motors)
    speed_percent: 0-100 (ONLY positive values)
    """
    if pwm:
        # For FORWARD-ONLY motors: 0% = stop (7.5%), 100% = full forward (10%)
        # NO REVERSE! ESC tidak support reverse
        speed_percent = max(0, min(100, speed_percent))  # Clamp to 0-100
        duty = 7.5 + (speed_percent / 100.0) * 2.5  # 7.5% (stop) to 10% (full)
        pwm.ChangeDutyCycle(duty)

def stop_motors():
    set_motor(pwm_left_gpio, 0)
    set_motor(pwm_right_gpio, 0)

def drive(throttle, steering):
    """
    Differential drive for FORWARD-ONLY motors
    throttle: 0-100 (base speed, ALWAYS positive)
    steering: -100 (left) to +100 (right)
    
    Strategy untuk belok tanpa reverse:
    - Belok kiri: motor kanan full, motor kiri slow/stop
    - Belok kanan: motor kiri full, motor kanan slow/stop
    """
    # Ensure throttle is positive (forward only)
    throttle = max(0, min(100, abs(throttle)))
    
    if steering < 0:  # Turn LEFT
        # Motor kanan cepat, motor kiri lambat
        left = throttle * (1 + steering / 100.0)  # Reduce left motor
        right = throttle  # Right motor full
    elif steering > 0:  # Turn RIGHT
        # Motor kiri cepat, motor kanan lambat
        left = throttle  # Left motor full
        right = throttle * (1 - steering / 100.0)  # Reduce right motor
    else:  # STRAIGHT
        left = throttle
        right = throttle
    
    # Clamp to 0-100 (NO NEGATIVE values!)
    left = max(0, min(100, left))
    right = max(0, min(100, right))
    
    set_motor(pwm_left_gpio, left)
    set_motor(pwm_right_gpio, right)
    
    logger.debug(f"🚢 Drive: T={throttle}%, S={steering}% | L={left:.0f}% R={right:.0f}%")

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
        logger.warning(f"⚠️ Gate wrong: RED={red_ball['x']}, GREEN={green_ball['x']}")
    
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
    logger.info(f"🎯 offset={offset:.0f}px, dist={distance:.1f}m")
    return abs(offset) < OFFSET_THRESHOLD

# ========== SEGMENT TRANSITION ==========
def move_to_next_segment():
    global current_segment, obstacles_in_segment
    
    current_segment += 1
    obstacles_in_segment = 0
    
    logger.info(f"🔄 SEGMENT {current_segment + 1}")
    
    # Lateral movement LEFT (for FORWARD-ONLY motors)
    # Strategy: Motor kanan ON, motor kiri OFF/SLOW → belok kiri sambil maju
    logger.info("   Lateral LEFT...")
    drive(40, -80)  # Throttle 40%, steering LEFT 80%
    time.sleep(8.0)
    
    stop_motors()
    time.sleep(1.0)
    
    # Sweep for obstacles (rotate in place)
    logger.info("   Sweeping...")
    for _ in range(3):
        drive(30, 70)  # Slow forward + right turn
        time.sleep(1.5)
        stop_motors()
        time.sleep(0.5)
    
    logger.info(f"✅ Ready SEGMENT {current_segment + 1}")

def return_to_start():
    """
    Return to START (MODIFIED for FORWARD-ONLY motors)
    Strategy: Turn around (pivot) then go straight
    """
    logger.info("🏁 Returning to START...")
    
    # Turn around (180 degrees) - pivot turn
    logger.info("   Turning around...")
    drive(35, 100)  # Slow forward + full right turn (pivot)
    time.sleep(6.0)  # Adjust timing for 180° turn
    
    stop_motors()
    time.sleep(1.0)
    
    # Go straight to START area
    logger.info("   Going to START...")
    drive(60, 0)  # Straight forward
    time.sleep(10.0)
    
    stop_motors()
    logger.info("🏁 ARRIVED!")

def emergency_return_to_start():
    """
    Emergency return (MODIFIED for FORWARD-ONLY motors)
    NO REVERSE! Only forward + turning
    """
    logger.warning("🚨 EMERGENCY RETURN TO START!")
    
    stop_motors()
    time.sleep(1.0)
    
    # Turn around (180 degrees) using pivot turn
    logger.info("   Emergency turn around...")
    drive(40, 100)  # Slow forward + full right turn
    time.sleep(7.0)  # Time for 180° turn
    
    stop_motors()
    time.sleep(1.0)
    
    # Move forward to START area
    logger.info("   Moving to START...")
    drive(70, 0)  # Straight forward, faster
    time.sleep(12.0)
    
    stop_motors()
    logger.info("🏁 EMERGENCY RETURN COMPLETE!")
    logger.info("   Please check system and restart mission")

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
def mission_loop_lintasan_a():
    global latest_frame, mission_active
    global current_segment, obstacles_in_segment, total_obstacles_passed
    
    logger.info("🚀 LINTASAN-A Started")
    logger.info(f"   Emergency return: {'ENABLED' if EMERGENCY_RETURN_ENABLED else 'DISABLED'}")
    
    mid_history = []
    last_obstacle_time = 0
    no_detection_count = 0
    frame_count = 0
    
    # Emergency tracking
    last_gate_seen_time = time.time()
    obstacle_start_time = None
    last_obstacle_distance = None
    
    while not stop_event.is_set():
        try:
            ret, frame = camera_nav.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            with frame_lock:
                latest_frame = frame.copy()
            
            with mission_lock:
                is_active = mission_active
            
            if not is_active:
                if sio.connected and frame_count % 3 == 0:
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    jpg_base64 = base64.b64encode(buffer).decode('utf-8')
                    sio.emit('video_frame', {'image': jpg_base64})
                
                frame_count += 1
                time.sleep(0.1)
                continue
            
            # ACTIVE MISSION
            red_ball = detect_ball(frame, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
            green_ball = detect_ball(frame, GREEN_LOWER, GREEN_UPPER)
            
            gate_valid, mid_x_raw, distance = validate_gate(red_ball, green_ball)
            
            if not gate_valid:
                no_detection_count += 1
                
                # === EMERGENCY CHECK 1: No gate detected for too long ===
                if EMERGENCY_RETURN_ENABLED:
                    time_since_last_gate = time.time() - last_gate_seen_time
                    if time_since_last_gate > MAX_NO_GATE_TIME:
                        logger.error(f"🚨 EMERGENCY: No gate for {time_since_last_gate:.0f}s!")
                        emergency_return_to_start()
                        
                        with mission_lock:
                            mission_active = False
                        
                        if sio.connected:
                            sio.emit('emergency_abort', {
                                'reason': 'No gate detected for too long',
                                'time': time_since_last_gate,
                                'timestamp': datetime.now().isoformat()
                            })
                        
                        break
                
                if no_detection_count > 30:
                    logger.warning("⚠️ Sweeping...")
                    drive(0, 30)
                    time.sleep(0.5)
                    no_detection_count = 0
                
                stop_motors()
                time.sleep(0.05)
                continue
            
            # Gate detected - reset timer
            last_gate_seen_time = time.time()
            no_detection_count = 0
            
            # === EMERGENCY CHECK 2: Stuck at same obstacle ===
            if EMERGENCY_RETURN_ENABLED:
                if distance < 5.0:  # Near obstacle
                    if obstacle_start_time is None:
                        obstacle_start_time = time.time()
                        last_obstacle_distance = distance
                    else:
                        # Check if stuck (distance not decreasing significantly)
                        time_at_obstacle = time.time() - obstacle_start_time
                        distance_change = abs(last_obstacle_distance - distance)
                        
                        if time_at_obstacle > MAX_STUCK_TIME and distance_change < 0.5:
                            logger.error(f"🚨 EMERGENCY: Stuck at obstacle for {time_at_obstacle:.0f}s!")
                            emergency_return_to_start()
                            
                            with mission_lock:
                                mission_active = False
                            
                            if sio.connected:
                                sio.emit('emergency_abort', {
                                    'reason': 'Stuck at obstacle',
                                    'time': time_at_obstacle,
                                    'distance': distance,
                                    'timestamp': datetime.now().isoformat()
                                })
                            
                            break
                else:
                    # Not near obstacle - reset tracker
                    obstacle_start_time = None
                    last_obstacle_distance = None
            
            mid_history.append(mid_x_raw)
            if len(mid_history) > 5:
                mid_history.pop(0)
            mid_x = sum(mid_history) / len(mid_history)
            
            navigate_to_gate(mid_x, distance)
            
            current_time = time.time()
            if distance < 2.0 and (current_time - last_obstacle_time) > 4.0:
                obstacles_in_segment += 1
                total_obstacles_passed += 1
                last_obstacle_time = current_time
                mid_history = []
                
                # Reset emergency trackers
                obstacle_start_time = None
                last_obstacle_distance = None
                
                logger.info("=" * 60)
                logger.info(f"🚪✅ OBSTACLE {total_obstacles_passed}/{TOTAL_OBSTACLES}")
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
                
                if obstacles_in_segment >= SEGMENT_OBSTACLES[current_segment]:
                    logger.info(f"✅ SEGMENT {current_segment + 1} COMPLETE!")
                    
                    if current_segment < len(SEGMENT_OBSTACLES) - 1:
                        move_to_next_segment()
                    else:
                        logger.info("🏆 ALL COMPLETE!")
                        return_to_start()
                        
                        with mission_lock:
                            mission_active = False
                        
                        if sio.connected:
                            sio.emit('mission_complete', {
                                'total_obstacles': total_obstacles_passed,
                                'timestamp': datetime.now().isoformat()
                            })
                        
                        logger.info("🎉 SUCCESS!")
                        break
            
            if sio.connected and frame_count % 2 == 0:
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                jpg_base64 = base64.b64encode(buffer).decode('utf-8')
                sio.emit('video_frame', {'image': jpg_base64})
            
            frame_count += 1
            
            if frame_count % 50 == 0:
                logger.info(f"📹 Frame {frame_count}, Progress: {total_obstacles_passed}/{TOTAL_OBSTACLES}")
        
        except Exception as e:
            logger.error(f"❌ Error: {e}")
            stop_motors()
            time.sleep(0.5)

# ========== MAIN ==========
def main():
    global camera_nav
    
    logger.info("=" * 60)
    logger.info("🚀 SAIBATIN AZURA 1.0 - COMPETITION")
    logger.info(f"   Emergency Return: {'ENABLED ✅' if EMERGENCY_RETURN_ENABLED else 'DISABLED ❌'}")
    logger.info("=" * 60)
    
    try:
        threading.Thread(target=connect_to_server_background, daemon=True).start()
        
        logger.info("⚡ Initializing motors...")
        if not init_motors():
            logger.error("❌ Motor failed")
            return
        
        logger.info("🎥 Initializing camera...")
        camera_nav = cv2.VideoCapture(CAMERA_NAVIGATION)
        if not camera_nav.isOpened():
            logger.error("❌ Camera failed")
            return
        
        camera_nav.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera_nav.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        camera_nav.set(cv2.CAP_PROP_FPS, 30)
        
        logger.info("✅ Camera OK")
        
        logger.info("\n" + "=" * 60)
        logger.info("✅ SYSTEM READY!")
        logger.info("   Commands:")
        logger.info("   • start     - Start mission")
        logger.info("   • stop      - Stop mission")
        logger.info("   • return    - Return to START")
        logger.info("   • emergency - Emergency return")
        logger.info("   • status    - Show status")
        logger.info("   • quit      - Exit program")
        logger.info("   Or use dashboard buttons")
        logger.info("=" * 60 + "\n")
        
        def command_interface():
            global mission_active
            while not stop_event.is_set():
                try:
                    cmd = input("> ").strip().lower()
                    if cmd == 'start':
                        with mission_lock:
                            mission_active = True
                        logger.info("🚀 STARTED!")
                    
                    elif cmd == 'stop':
                        with mission_lock:
                            mission_active = False
                        stop_motors()
                        logger.info("🛑 STOPPED")
                    
                    elif cmd == 'return':
                        # Manual return to start
                        logger.info("🏁 RETURN TO START command")
                        with mission_lock:
                            mission_active = False
                        stop_motors()
                        time.sleep(0.5)
                        emergency_return_to_start()
                    
                    elif cmd == 'emergency':
                        # Manual emergency return
                        logger.info("🚨 Manual emergency return!")
                        emergency_return_to_start()
                        with mission_lock:
                            mission_active = False
                    
                    elif cmd == 'quit':
                        stop_event.set()
                        break
                    
                    elif cmd == 'status':
                        with mission_lock:
                            status = "ACTIVE" if mission_active else "STANDBY"
                        logger.info(f"Status: {status}, Obstacles: {total_obstacles_passed}/{TOTAL_OBSTACLES}")
                    
                    elif cmd == 'help':
                        logger.info("Commands:")
                        logger.info("  start     - Start mission")
                        logger.info("  stop      - Stop mission")
                        logger.info("  return    - Return to START")
                        logger.info("  emergency - Emergency return")
                        logger.info("  status    - Show status")
                        logger.info("  quit      - Exit")
                except:
                    break
        
        threading.Thread(target=command_interface, daemon=True).start()
        
        mission_loop_lintasan_a()
    
    except KeyboardInterrupt:
        logger.info("\n🛑 Ctrl+C")
    
    except Exception as e:
        logger.exception(f"❌ Fatal: {e}")
    
    finally:
        logger.info("🧹 Cleanup...")
        stop_event.set()
        stop_motors()
        
        if camera_nav:
            camera_nav.release()
        
        if sio.connected:
            sio.disconnect()
        
        if GPIO_AVAILABLE:
            try:
                if pwm_left_gpio:
                    pwm_left_gpio.stop()
                if pwm_right_gpio:
                    pwm_right_gpio.stop()
                GPIO.cleanup()
            except:
                pass
        
        logger.info("👋 Shutdown complete")

if __name__ == "__main__":
    main()