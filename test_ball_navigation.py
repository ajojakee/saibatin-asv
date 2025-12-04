#!/usr/bin/env python3
"""
LATIHAN: Deteksi Bola Merah-Hijau + Motor Otomatis ke Tengah
Test sederhana untuk validasi:
1. Deteksi bola merah (HSV)
2. Deteksi bola hijau (HSV)
3. Hitung titik tengah (midpoint)
4. Motor otomatis gerak ke tengah (differential drive)
"""

import cv2
import numpy as np
import time
import math

# Try import GPIO
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    MOTOR_LEFT_GPIO = 18
    MOTOR_RIGHT_GPIO = 13
    PWM_FREQ = 50
    print("✅ GPIO available")
except ImportError:
    GPIO_AVAILABLE = False
    print("⚠️ GPIO not available - simulation mode")

# ========== KONFIGURASI ==========
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CENTER_X = CAMERA_WIDTH // 2
CENTER_TOLERANCE = 50

# NEW: Headless mode (no GUI for SSH)
HEADLESS_MODE = True  # ← Set True untuk SSH/remote, False untuk local dengan display

# HSV Color ranges (RELAX + ADD SHAPE FILTER)
RED_LOWER1 = np.array([0, 120, 150])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 150])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([45, 100, 100])  # ← RELAX untuk bola hijau muda
GREEN_UPPER = np.array([75, 255, 255])

MIN_BALL_RADIUS = 15
MIN_AREA = 500
MIN_CIRCULARITY = 0.7  # ← NEW! Filter non-circular shapes

# ========== GPIO MOTOR SETUP ==========
pwm_left = None
pwm_right = None

def init_motors():
    """Initialize GPIO motors"""
    global pwm_left, pwm_right
    
    if not GPIO_AVAILABLE:
        print("⚠️ Skipping motor init (GPIO not available)")
        return False
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_LEFT_GPIO, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT_GPIO, GPIO.OUT)
        
        pwm_left = GPIO.PWM(MOTOR_LEFT_GPIO, PWM_FREQ)
        pwm_right = GPIO.PWM(MOTOR_RIGHT_GPIO, PWM_FREQ)
        
        # Start at neutral
        pwm_left.start(7.5)
        pwm_right.start(7.5)
        
        print("✅ Motors initialized (GPIO 18 & 13)")
        return True
    except Exception as e:
        print(f"❌ Motor init error: {e}")
        return False

def set_motor_speeds(left_percent, right_percent):
    """
    Set motor speeds (-100 to +100)
    0 = stop, positive = forward, negative = reverse
    """
    if not GPIO_AVAILABLE or pwm_left is None:
        # Simulation mode
        print(f"🎮 MOTOR: Left={left_percent:+.0f}% | Right={right_percent:+.0f}%")
        return
    
    try:
        # Convert percent to duty cycle (5.0% to 10.0%)
        left_duty = 7.5 + (left_percent / 100.0) * 2.5
        right_duty = 7.5 + (right_percent / 100.0) * 2.5
        
        # Clamp
        left_duty = max(5.0, min(10.0, left_duty))
        right_duty = max(5.0, min(10.0, right_duty))
        
        pwm_left.ChangeDutyCycle(left_duty)
        pwm_right.ChangeDutyCycle(right_duty)
        
        print(f"⚡ MOTOR: L={left_percent:+.0f}% ({left_duty:.2f}dc) | R={right_percent:+.0f}% ({right_duty:.2f}dc)")
    except Exception as e:
        print(f"❌ Motor error: {e}")

def stop_motors():
    """Stop all motors"""
    set_motor_speeds(0, 0)
    print("🛑 Motors stopped")

# ========== BALL DETECTION ==========
def detect_ball(frame, lower, upper, lower2=None, upper2=None, color_name="ball"):
    """
    Detect ball with COLOR + SHAPE filtering
    """
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    if lower2 is not None and upper2 is not None:
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower, upper),
            cv2.inRange(hsv, lower2, upper2)
        )
    else:
        mask = cv2.inRange(hsv, lower, upper)
    
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Find BEST circular object
    best_ball = None
    best_score = 0
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
        
        # Calculate circularity
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        
        if radius < MIN_BALL_RADIUS:
            continue
        
        # CHECK CIRCULARITY!
        if circularity < MIN_CIRCULARITY:
            print(f"   {color_name.upper()} REJECTED: circ={circularity:.2f} < {MIN_CIRCULARITY}")
            continue
        
        # Score based on circularity + size
        score = circularity * area
        
        if score > best_score:
            best_score = score
            best_ball = {'x': int(x), 'y': int(y), 'radius': int(radius), 'circularity': circularity}
    
    if best_ball:
        print(f"   {color_name.upper()}: x={best_ball['x']}, y={best_ball['y']}, r={best_ball['radius']}, circ={best_ball['circularity']:.2f}")
    
    return best_ball

def calculate_midpoint(red_ball, green_ball):
    """Calculate midpoint between red and green balls"""
    if red_ball is None or green_ball is None:
        return None
    
    mid_x = (red_ball['x'] + green_ball['x']) // 2
    mid_y = (red_ball['y'] + green_ball['y']) // 2
    
    print(f"   MIDPOINT: x={mid_x}, y={mid_y}")
    return {'x': mid_x, 'y': mid_y}

def navigate_to_midpoint(mid_x):
    """
    Navigate ASV to midpoint using differential drive
    Returns: True if centered, False if still adjusting
    """
    if mid_x is None:
        stop_motors()
        return False
    
    # Calculate offset from center
    offset = mid_x - CENTER_X
    
    # Check if already centered
    if abs(offset) < CENTER_TOLERANCE:
        print(f"   ✅ CENTERED! (offset={offset}px)")
        set_motor_speeds(60, 60)  # Go straight forward
        return True
    
    # Calculate steering correction
    steering = (offset / CENTER_X) * 100
    steering = max(-100, min(100, steering))
    
    # Adjust throttle based on steering angle
    if abs(steering) > 50:
        throttle = 30  # Slow turn
    elif abs(steering) > 20:
        throttle = 50  # Medium turn
    else:
        throttle = 70  # Near center
    
    # Differential drive
    if steering < 0:
        # Turn LEFT (midpoint is to the left)
        left_speed = throttle * (1 + steering / 100.0)
        right_speed = throttle
        print(f"   ⬅️ TURN LEFT: offset={offset}px, steering={steering:.1f}%")
    else:
        # Turn RIGHT (midpoint is to the right)
        left_speed = throttle
        right_speed = throttle * (1 - steering / 100.0)
        print(f"   ➡️ TURN RIGHT: offset={offset}px, steering={steering:.1f}%")
    
    set_motor_speeds(int(left_speed), int(right_speed))
    return False

# ========== MAIN LOOP ==========
def main():
    print("=" * 60)
    print("🎯 LATIHAN: Deteksi Bola Merah-Hijau + Auto Navigation")
    if HEADLESS_MODE:
        print("   Mode: HEADLESS (no GUI - SSH compatible)")
    print("=" * 60)
    print("Controls:")
    print("  Ctrl+C = Quit")
    if not HEADLESS_MODE:
        print("  Q = Quit (GUI)")
        print("  S = Stop motors")
        print("  SPACE = Toggle auto navigation")
    print("=" * 60)
    
    # Initialize camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    
    if not cap.isOpened():
        print("❌ Cannot open camera")
        return
    
    print("✅ Camera initialized")
    
    # Initialize motors
    init_motors()
    
    # State
    auto_nav_enabled = True
    frame_count = 0
    last_log_time = time.time()
    
    print("\n🚀 Starting... (auto navigation: ON)")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️ Camera read failed")
                break
            
            frame_count += 1
            current_time = time.time()
            
            # Detect balls
            red_ball = detect_ball(frame, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2, "red")
            green_ball = detect_ball(frame, GREEN_LOWER, GREEN_UPPER, color_name="green")
            
            # Calculate midpoint
            if red_ball and green_ball:
                midpoint = calculate_midpoint(red_ball, green_ball)
                
                # Auto navigation
                if auto_nav_enabled:
                    is_centered = navigate_to_midpoint(midpoint['x'])
                
                # Draw visualizations ONLY if not headless
                if not HEADLESS_MODE:
                    # Red ball
                    cv2.circle(frame, (red_ball['x'], red_ball['y']), red_ball['radius'], (0, 0, 255), 2)
                    cv2.putText(frame, "RED", (red_ball['x']-20, red_ball['y']-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # Green ball
                    cv2.circle(frame, (green_ball['x'], green_ball['y']), green_ball['radius'], (0, 255, 0), 2)
                    cv2.putText(frame, "GREEN", (green_ball['x']-20, green_ball['y']-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Midpoint line
                    cv2.line(frame, (midpoint['x'], 0), (midpoint['x'], CAMERA_HEIGHT), (255, 0, 0), 2)
                    cv2.circle(frame, (midpoint['x'], midpoint['y']), 10, (255, 0, 0), -1)
            else:
                # No balls detected or only one ball
                if auto_nav_enabled:
                    stop_motors()
                
                if not HEADLESS_MODE:
                    if red_ball:
                        cv2.circle(frame, (red_ball['x'], red_ball['y']), red_ball['radius'], (0, 0, 255), 2)
                    elif green_ball:
                        cv2.circle(frame, (green_ball['x'], green_ball['y']), green_ball['radius'], (0, 255, 0), 2)
            
            # Show frame ONLY if not headless
            if not HEADLESS_MODE:
                # Draw center line
                cv2.line(frame, (CENTER_X, 0), (CENTER_X, CAMERA_HEIGHT), (255, 255, 255), 1)
                cv2.circle(frame, (CENTER_X, CAMERA_HEIGHT//2), 5, (255, 255, 255), -1)
                
                # Draw center tolerance zone
                cv2.rectangle(frame, 
                             (CENTER_X - CENTER_TOLERANCE, 0), 
                             (CENTER_X + CENTER_TOLERANCE, CAMERA_HEIGHT), 
                             (128, 128, 128), 1)
                
                # Status overlay
                nav_status = "AUTO: ON" if auto_nav_enabled else "AUTO: OFF"
                cv2.putText(frame, nav_status, (10, CAMERA_HEIGHT - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                cv2.putText(frame, f"Frame: {frame_count}", (CAMERA_WIDTH - 150, CAMERA_HEIGHT - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Show frame
                cv2.imshow('Ball Detection + Auto Navigation', frame)
                
                # Keyboard control
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("\n🛑 Quit command received")
                    break
                elif key == ord('s'):
                    print("\n🛑 Stop motors")
                    stop_motors()
                    auto_nav_enabled = False
                elif key == ord(' '):
                    auto_nav_enabled = not auto_nav_enabled
                    status = "ON" if auto_nav_enabled else "OFF"
                    print(f"\n🔄 Auto navigation: {status}")
                    if not auto_nav_enabled:
                        stop_motors()
            else:
                # Headless mode - log status periodically
                if current_time - last_log_time >= 2.0:
                    status = f"Frame {frame_count} | Auto: {auto_nav_enabled}"
                    if red_ball and green_ball:
                        status += " | BOTH BALLS DETECTED | NAVIGATING"
                    elif red_ball:
                        status += " | RED only"
                    elif green_ball:
                        status += " | GREEN only"
                    else:
                        status += " | NO BALLS"
                    print(f"📊 {status}")
                    last_log_time = current_time
                
                # Sleep to prevent excessive CPU usage
                time.sleep(0.03)
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user (Ctrl+C)")
    
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        stop_motors()
        cap.release()
        if not HEADLESS_MODE:
            cv2.destroyAllWindows()
        
        if GPIO_AVAILABLE and pwm_left:
            pwm_left.stop()
            pwm_right.stop()
            GPIO.cleanup()
        
        print("👋 Goodbye!")

if __name__ == "__main__":
    main()
