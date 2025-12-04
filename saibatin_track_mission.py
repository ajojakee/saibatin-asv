#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - TRACK A/B MISSION SYSTEM
Complete implementation for 10-obstacle ball mission with track inference
"""

import cv2
import numpy as np
import socketio
import base64
import time
import threading
from datetime import datetime
import logging
import os
import math
from enum import Enum
from collections import deque

# ========== LOGGING ==========
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler('track_mission.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ========== GPIO IMPORT ==========
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    MOTOR_LEFT_GPIO = 18
    MOTOR_RIGHT_GPIO = 13
    PWM_FREQ_GPIO = 50
except ImportError:
    GPIO_AVAILABLE = False
    logger.warning("⚠️ GPIO not available")

# ========== CONFIGURATION ==========
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CENTER_X = CAMERA_WIDTH // 2
CENTER_TOLERANCE = int(0.05 * CAMERA_WIDTH)  # 5% tolerance

DETECT_CONFIDENCE_THRESHOLD = 0.6
TIMEOUT_DETECT = 8.0  # seconds
SWEEP_ANGLE = 30  # degrees
APPROACH_DISTANCE_PIXELS = 80
DEBOUNCE_TIME = 1.0  # seconds
MIN_BALL_RADIUS = 15
MIN_DETECTION_FRAMES = 2  # Must detect in 2 consecutive frames

# HSV Thresholds
RED_LOWER1 = np.array([0, 120, 150])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 150])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([45, 120, 120])
GREEN_UPPER = np.array([70, 255, 255])

# Mission parameters
SEGMENT_PLAN = [3, 4, 3]  # Total 10 balls in 3 segments
FOCAL_LENGTH = 600

# ========== STATE MACHINE ==========
class MissionState(Enum):
    INIT = "INIT"
    IDENTIFY_TRACK = "IDENTIFY_TRACK"
    RUN_SEGMENT = "RUN_SEGMENT"
    MOVE_TO_NEXT_SEGMENT = "MOVE_TO_NEXT_SEGMENT"
    RETURN_START = "RETURN_START"
    COMPLETED = "COMPLETED"
    ABORTED = "ABORTED"

class BallSide(Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    CENTER = "CENTER"

class Track(Enum):
    A = "A"  # Red RIGHT, Green LEFT
    B = "B"  # Red LEFT, Green RIGHT
    UNKNOWN = "UNKNOWN"

# ========== GLOBAL STATE ==========
current_state = MissionState.INIT
determined_track = Track.UNKNOWN
segment_index = 0
passed_total = 0
segment_passed = 0
mission_active = False

# Thread control
stop_event = threading.Event()
frame_lock = threading.Lock()
latest_frame = None

# Socket.IO
sio = socketio.Client()
SOCKETIO_SERVER = "http://10.132.119.157:5000"

# Detection history (for debouncing)
detection_history = deque(maxlen=5)

# ========== MOTOR CONTROL ==========
pwm_left_gpio = None
pwm_right_gpio = None

def init_gpio_motors():
    """Initialize GPIO motors"""
    global pwm_left_gpio, pwm_right_gpio
    if not GPIO_AVAILABLE:
        return False
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_LEFT_GPIO, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_GPIO, GPIO.OUT)
        pwm_left_gpio = GPIO.PWM(MOTOR_LEFT_GPIO, PWM_FREQ_GPIO)
        pwm_right_gpio = GPIO.PWM(MOTOR_RIGHT_GPIO, PWM_FREQ_GPIO)
        pwm_left_gpio.start(7.5)
        pwm_right_gpio.start(7.5)
        logger.info("✅ GPIO motors initialized")
        return True
    except Exception as e:
        logger.error(f"❌ GPIO init error: {e}")
        return False

def set_motor_speeds(left_percent, right_percent):
    """Set motor speeds (-100 to +100)"""
    if not GPIO_AVAILABLE:
        return
    try:
        left_duty = 7.5 + (left_percent / 100.0) * 2.5
        right_duty = 7.5 + (right_percent / 100.0) * 2.5
        left_duty = max(5.0, min(10.0, left_duty))
        right_duty = max(5.0, min(10.0, right_duty))
        pwm_left_gpio.ChangeDutyCycle(left_duty)
        pwm_right_gpio.ChangeDutyCycle(right_duty)
    except Exception as e:
        logger.error(f"Motor error: {e}")

def stop_motors():
    """Stop all motors"""
    set_motor_speeds(0, 0)
    logger.info("🛑 Motors stopped")

def turn_relative(angle):
    """Turn relative angle (positive = right, negative = left)"""
    logger.info(f"🔄 Turning {angle}°")
    if angle > 0:
        set_motor_speeds(50, -50)  # Turn right
    else:
        set_motor_speeds(-50, 50)  # Turn left
    time.sleep(abs(angle) / 30.0)  # Simple time-based turn
    stop_motors()

def move_forward(duration=1.0):
    """Move forward for duration"""
    logger.info(f"➡️ Moving forward {duration}s")
    set_motor_speeds(60, 60)
    time.sleep(duration)
    stop_motors()

def approach_ball(ball_x, ball_y):
    """Approach ball using differential steering"""
    offset = ball_x - CENTER_X
    steering = (offset / CENTER_X) * 100
    steering = max(-100, min(100, steering))
    
    if abs(steering) > 50:
        throttle = 30
    elif abs(steering) > 20:
        throttle = 50
    else:
        throttle = 70
    
    left = throttle * (1 - steering / 200.0) if steering > 0 else throttle
    right = throttle * (1 + steering / 200.0) if steering < 0 else throttle
    
    set_motor_speeds(int(left), int(right))

# ========== BALL DETECTION ==========
def detect_ball(frame):
    """
    Detect ball and return {color, x, y, radius, confidence, side}
    Returns None if no ball detected
    """
    if frame is None:
        return None
    
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    kernel = np.ones((5, 5), np.uint8)
    
    # Detect RED ball
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, RED_LOWER1, RED_UPPER1),
        cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    )
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.dilate(mask_red, kernel, iterations=2)
    
    # Detect GREEN ball
    mask_green = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.dilate(mask_green, kernel, iterations=2)
    
    # Find contours
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_detection = None
    best_area = 0
    
    # Check RED balls
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if area > 500 and area > best_area:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > MIN_BALL_RADIUS:
                confidence = min(1.0, area / 5000.0)
                side = determine_side(int(x))
                best_detection = {
                    'color': 'red',
                    'x': int(x),
                    'y': int(y),
                    'radius': int(radius),
                    'confidence': confidence,
                    'side': side,
                    'area': int(area)
                }
                best_area = area
    
    # Check GREEN balls
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if area > 500 and area > best_area:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > MIN_BALL_RADIUS:
                confidence = min(1.0, area / 5000.0)
                side = determine_side(int(x))
                best_detection = {
                    'color': 'green',
                    'x': int(x),
                    'y': int(y),
                    'radius': int(radius),
                    'confidence': confidence,
                    'side': side,
                    'area': int(area)
                }
                best_area = area
    
    return best_detection

def determine_side(x):
    """Determine ball side based on x position"""
    if x < CENTER_X - CENTER_TOLERANCE:
        return BallSide.LEFT
    elif x > CENTER_X + CENTER_TOLERANCE:
        return BallSide.RIGHT
    else:
        return BallSide.CENTER

def is_stable_detection(detection):
    """Check if detection is stable (debouncing)"""
    if detection is None:
        return False
    
    detection_history.append(detection)
    
    if len(detection_history) < MIN_DETECTION_FRAMES:
        return False
    
    # Check if last N detections have same color and similar position
    recent = list(detection_history)[-MIN_DETECTION_FRAMES:]
    colors = [d['color'] for d in recent]
    sides = [d['side'] for d in recent]
    
    if len(set(colors)) == 1 and len(set(sides)) == 1:
        return True
    
    return False

# ========== TRACK INFERENCE ==========
def infer_track_from_detections():
    """
    Infer track A or B from detection history
    Track A: Red RIGHT, Green LEFT
    Track B: Red LEFT, Green RIGHT
    """
    global determined_track
    
    logger.info("🔍 Inferring track from detections...")
    
    evidence = []
    timeout = time.time() + 6.0
    
    while time.time() < timeout and len(evidence) < 10:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            frame = latest_frame.copy()
        
        detection = detect_ball(frame)
        if detection and detection['confidence'] >= DETECT_CONFIDENCE_THRESHOLD:
            evidence.append(detection)
            logger.info(f"   Evidence: {detection['color']} on {detection['side'].value}")
            time.sleep(0.2)
    
    if len(evidence) < 2:
        logger.warning("⚠️ Insufficient evidence, performing sweep...")
        # Sweep scan
        turn_relative(-SWEEP_ANGLE)
        time.sleep(0.5)
        turn_relative(SWEEP_ANGLE * 2)
        time.sleep(0.5)
        turn_relative(-SWEEP_ANGLE)
        return infer_track_from_detections()  # Retry
    
    # Voting logic
    votes_A = 0
    votes_B = 0
    
    for det in evidence:
        if det['color'] == 'red' and det['side'] == BallSide.RIGHT:
            votes_A += 1
        elif det['color'] == 'red' and det['side'] == BallSide.LEFT:
            votes_B += 1
        elif det['color'] == 'green' and det['side'] == BallSide.LEFT:
            votes_A += 1
        elif det['color'] == 'green' and det['side'] == BallSide.RIGHT:
            votes_B += 1
    
    if votes_A > votes_B:
        determined_track = Track.A
        logger.info("✅ Track identified: A (Red RIGHT, Green LEFT)")
    elif votes_B > votes_A:
        determined_track = Track.B
        logger.info("✅ Track identified: B (Red LEFT, Green RIGHT)")
    else:
        logger.warning("⚠️ Ambiguous track, defaulting to A")
        determined_track = Track.A
    
    if sio.connected:
        sio.emit('mission_event', {
            'event': 'track_identified',
            'track': determined_track.value,
            'evidence': [{'color': e['color'], 'side': e['side'].value, 'confidence': e['confidence']} for e in evidence]
        })

# ========== DETECT AND PASS ONE ==========
def detect_and_pass_one():
    """
    Detect and pass one ball
    Returns True if successful, False if timeout/failure
    """
    logger.info(f"🎯 Detecting ball #{passed_total + 1}...")
    
    start_time = time.time()
    ball_detected = False
    approach_start = None
    
    while (time.time() - start_time) < TIMEOUT_DETECT:
        with frame_lock:
            if latest_frame is None:
                time.sleep(0.1)
                continue
            frame = latest_frame.copy()
        
        detection = detect_ball(frame)
        
        if detection and detection['confidence'] >= DETECT_CONFIDENCE_THRESHOLD:
            if is_stable_detection(detection):
                ball_detected = True
                logger.info(f"   Ball detected: {detection['color']} at ({detection['x']},{detection['y']}) side={detection['side'].value}")
                
                # Approach ball
                if approach_start is None:
                    approach_start = time.time()
                
                approach_ball(detection['x'], detection['y'])
                
                # Check if close enough (radius growing or close to bottom)
                if detection['radius'] > APPROACH_DISTANCE_PIXELS or detection['y'] > CAMERA_HEIGHT - 100:
                    # Pass through
                    logger.info("   Ball close! Passing through...")
                    move_forward(2.0)
                    stop_motors()
                    time.sleep(DEBOUNCE_TIME)
                    
                    # Report success
                    if sio.connected:
                        sio.emit('mission_event', {
                            'event': 'ball_passed',
                            'color': detection['color'],
                            'side': detection['side'].value,
                            'segment_index': segment_index,
                            'segment_passed': segment_passed + 1,
                            'total_passed': passed_total + 1
                        })
                    
                    logger.info(f"✅ Ball #{passed_total + 1} PASSED!")
                    return True
        else:
            if ball_detected and (time.time() - approach_start) > 3.0:
                # Lost ball during approach, retry
                logger.warning("⚠️ Lost ball during approach")
                stop_motors()
                time.sleep(0.5)
                return False
        
        time.sleep(0.05)
    
    # Timeout
    logger.warning("⚠️ Ball detection timeout")
    stop_motors()
    return False

# ========== MISSION STATE MACHINE ==========
def run_mission_state_machine():
    """Main mission state machine"""
    global current_state, segment_index, passed_total, segment_passed
    
    while not stop_event.is_set() and mission_active:
        try:
            if current_state == MissionState.INIT:
                logger.info("📍 State: INIT")
                time.sleep(1.0)
                current_state = MissionState.IDENTIFY_TRACK
            
            elif current_state == MissionState.IDENTIFY_TRACK:
                logger.info("🔍 State: IDENTIFY_TRACK")
                infer_track_from_detections()
                current_state = MissionState.RUN_SEGMENT
            
            elif current_state == MissionState.RUN_SEGMENT:
                target_count = SEGMENT_PLAN[segment_index]
                logger.info(f"🎯 State: RUN_SEGMENT {segment_index + 1} (Target: {target_count} balls)")
                
                segment_passed = 0
                retry_count = 0
                
                while segment_passed < target_count:
                    success = detect_and_pass_one()
                    
                    if success:
                        segment_passed += 1
                        passed_total += 1
                        retry_count = 0
                        logger.info(f"   Progress: {segment_passed}/{target_count} (Total: {passed_total}/10)")
                    else:
                        retry_count += 1
                        logger.warning(f"   Retry {retry_count}/3...")
                        
                        if retry_count >= 3:
                            logger.error("❌ Max retries exceeded, skipping ball")
                            break
                        
                        # Recovery: sweep scan
                        turn_relative(-SWEEP_ANGLE)
                        time.sleep(0.5)
                        turn_relative(SWEEP_ANGLE * 2)
                        time.sleep(0.5)
                        turn_relative(-SWEEP_ANGLE)
                
                logger.info(f"✅ Segment {segment_index + 1} COMPLETE")
                
                if sio.connected:
                    sio.emit('mission_event', {
                        'event': 'segment_complete',
                        'segment_index': segment_index,
                        'passed': segment_passed,
                        'total_passed': passed_total
                    })
                
                segment_index += 1
                
                if segment_index < len(SEGMENT_PLAN):
                    current_state = MissionState.MOVE_TO_NEXT_SEGMENT
                else:
                    current_state = MissionState.RETURN_START
            
            elif current_state == MissionState.MOVE_TO_NEXT_SEGMENT:
                logger.info(f"🔄 State: MOVE_TO_NEXT_SEGMENT (to segment {segment_index + 1})")
                
                # Determine turn direction based on track
                if determined_track == Track.A:
                    # Track A: turn LEFT to next segment
                    turn_relative(-90)
                else:
                    # Track B: turn RIGHT to next segment
                    turn_relative(90)
                
                move_forward(3.0)  # Move to next segment area
                time.sleep(1.0)
                
                current_state = MissionState.RUN_SEGMENT
            
            elif current_state == MissionState.RETURN_START:
                logger.info("🏁 State: RETURN_START")
                
                # Simple return: turn around and move forward
                turn_relative(180)
                move_forward(5.0)
                stop_motors()
                
                current_state = MissionState.COMPLETED
            
            elif current_state == MissionState.COMPLETED:
                logger.info("🎉 MISSION COMPLETED!")
                logger.info(f"   Total balls passed: {passed_total}/10")
                
                if sio.connected:
                    sio.emit('mission_event', {
                        'event': 'mission_complete',
                        'total_passed': passed_total,
                        'track': determined_track.value
                    })
                
                break
            
        except Exception as e:
            logger.exception(f"❌ State machine error: {e}")
            current_state = MissionState.ABORTED
            break
        
        time.sleep(0.1)

# ========== CAMERA LOOP ==========
camera_nav = None

def camera_loop():
    """Camera capture loop"""
    global latest_frame, camera_nav
    
    camera_nav = cv2.VideoCapture(0)
    camera_nav.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    camera_nav.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    
    while not stop_event.is_set():
        ret, frame = camera_nav.read()
        if ret:
            with frame_lock:
                latest_frame = frame
        time.sleep(0.03)
    
    camera_nav.release()

# ========== MAIN ==========
def main():
    global mission_active
    
    logger.info("🚀 SAIBATIN AZURA - TRACK A/B MISSION")
    logger.info("=" * 60)
    
    # Connect Socket.IO
    try:
        sio.connect(SOCKETIO_SERVER, wait_timeout=5)
        logger.info("✅ Dashboard connected")
    except Exception:
        logger.warning("⚠️ Dashboard not available")
    
    # Initialize GPIO
    init_gpio_motors()
    
    # Start camera thread
    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()
    logger.info("✅ Camera started")
    
    time.sleep(2.0)  # Wait for camera warm-up
    
    # Start mission
    mission_active = True
    logger.info("🚀 MISSION START!")
    
    try:
        run_mission_state_machine()
    except KeyboardInterrupt:
        logger.info("🛑 Interrupted by user")
    finally:
        mission_active = False
        stop_event.set()
        stop_motors()
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        if camera_nav:
            camera_nav.release()
        logger.info("👋 Shutdown complete")

if __name__ == "__main__":
    main()
