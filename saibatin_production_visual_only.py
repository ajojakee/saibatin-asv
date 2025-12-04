#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - PURE VISUAL NAVIGATION
Navigasi HANYA pakai deteksi bola merah-hijau (camera), TANPA GPS waypoint
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
import logging
import os

# ...existing code (logging, GPIO, camera setup)...

# ========== KONFIGURASI ==========
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
SOCKETIO_SERVER = "http://10.132.119.157:5000"

# Ball detection HSV
RED_LOWER1 = np.array([0, 120, 150])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 150])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([45, 120, 120])
GREEN_UPPER = np.array([70, 255, 255])

MIN_BALL_RADIUS = 15
MIN_BALL_SEPARATION = 30
MAX_BALL_SEPARATION = 400

# Navigation parameters
OFFSET_THRESHOLD = 50  # pixels tolerance untuk "centered"
FOCAL_LENGTH = 600

# Mission control
mission_active = False
mission_control_lock = threading.Lock()
gates_passed = 0  # Counter jumlah gate yang sudah dilewati

# ...existing code (GPIO motor setup, camera init)...

def calculate_navigation(red_ball, green_ball):
    """Calculate navigation - NO GPS, pure visual"""
    if not (red_ball and green_ball):
        return None, None, None, None
    
    dx = red_ball['x'] - green_ball['x']
    dy = red_ball['y'] - green_ball['y']
    ball_distance = math.sqrt(dx**2 + dy**2)
    
    # Validate ball separation
    if ball_distance < MIN_BALL_SEPARATION or ball_distance > MAX_BALL_SEPARATION:
        return None, None, None, None
    
    # Determine gate type
    gate_type = "RED_LEFT_GREEN_RIGHT" if red_ball['x'] < green_ball['x'] else "GREEN_LEFT_RED_RIGHT"
    
    # Calculate midpoint (RAW)
    mid_x = (red_ball['x'] + green_ball['x']) / 2
    mid_y = (red_ball['y'] + green_ball['y']) / 2
    
    # Calculate heading adjustment
    offset_x = mid_x - CAMERA_WIDTH / 2
    heading_adj = (offset_x / CAMERA_WIDTH) * 60
    heading_adj = max(-30, min(30, heading_adj))
    
    return mid_x, mid_y, heading_adj, gate_type

def camera_stream_loop():
    """Main loop - PURE VISUAL NAVIGATION"""
    global gates_passed, mission_active
    
    # Smoothing buffer
    mid_history = []
    max_history = 5
    
    # Gate state
    last_gate_time = 0
    GATE_COOLDOWN = 3.0
    gate_approaching = False
    
    while not stop_event.is_set():
        ret, frame = camera_nav.read()
        if not ret:
            time.sleep(0.1)
            continue
        
        # Store frame
        with frame_lock:
            latest_frame = frame.copy()
        
        # Detect balls
        red_ball = detect_ball(frame, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
        green_ball = detect_ball(frame, GREEN_LOWER, GREEN_UPPER)
        
        # Calculate navigation
        mid_x_raw, mid_y, heading_adj, gate_type = calculate_navigation(red_ball, green_ball)
        
        # Smooth midpoint
        if mid_x_raw is not None:
            mid_history.append(mid_x_raw)
            if len(mid_history) > max_history:
                mid_history.pop(0)
            mid_x = sum(mid_history) / len(mid_history)
        else:
            mid_history = []
            mid_x = None
        
        # Navigation logic
        current_time = time.time()
        
        with mission_control_lock:
            is_active = mission_active
        
        if mid_x is not None and gate_type is not None and is_active:
            # Calculate distance to gate (estimate from ball size)
            if red_ball and green_ball:
                avg_radius = (red_ball['radius'] + green_ball['radius']) / 2
                estimated_distance = (FOCAL_LENGTH * 0.1) / avg_radius if avg_radius > 0 else 999
                
                # Navigate to gate center
                navigate_to_gate_center(mid_x, heading_adj)
                
                # Check if passed gate
                if estimated_distance < 3.0 and (current_time - last_gate_time) > GATE_COOLDOWN:
                    gates_passed += 1
                    last_gate_time = current_time
                    mid_history = []  # Clear history
                    
                    logger.info(f"🚪 Gate {gates_passed} PASSED! ({gate_type})")
                    stop_motors()
                    
                    # Capture photo
                    with frame_lock:
                        if latest_frame is not None:
                            save_photo(latest_frame.copy(), gates_passed)
                    
                    # Send to dashboard
                    if sio.connected:
                        sio.emit('gate_passed', {
                            'gate_index': gates_passed,
                            'gate_type': gate_type,
                            'distance': round(estimated_distance, 2),
                            'timestamp': datetime.now().isoformat()
                        })
                    
                    time.sleep(1.0)  # Pause setelah lewat gate
        
        # Stream video to dashboard
        if current_time - last_send_time >= SEND_INTERVAL:
            stream_frame_to_dashboard(frame)
            last_send_time = current_time

def save_photo(frame, gate_number):
    """Save photo when passing gate"""
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"gate{gate_number}_{timestamp}.jpg"
        filepath = f"captures/{filename}"
        os.makedirs("captures", exist_ok=True)
        cv2.imwrite(filepath, frame)
        logger.info(f"📸 Saved: {filepath}")
    except Exception as e:
        logger.error(f"Photo save error: {e}")

# Mission control handlers
@sio.event
def mission_command(data):
    """Handle mission start/stop from dashboard"""
    global mission_active, gates_passed
    
    command = data.get('command')
    logger.info(f"📥 Mission command: {command}")
    
    if command == 'start':
        with mission_control_lock:
            mission_active = True
            gates_passed = 0  # Reset counter
        logger.info("🚀 Mission STARTED - Visual navigation active!")
    
    elif command == 'stop':
        with mission_control_lock:
            mission_active = False
        stop_motors()
        logger.info("🛑 Mission STOPPED")
    
    elif command == 'abort':
        with mission_control_lock:
            mission_active = False
        stop_motors()
        logger.warning("⚠️ Mission ABORTED")

# ...existing code (motor control, detect_ball, etc.)...

def main():
    logger.info("🚀 SAIBATIN AZURA 1.0 - VISUAL NAVIGATION ONLY")
    logger.info("=" * 60)
    logger.info("   Mode: Pure visual (NO GPS waypoints)")
    logger.info("   Navigation: Red-Green ball detection")
    logger.info("   Objective: Pass through gates (center between balls)")
    logger.info("=" * 60)
    
    # Connect to dashboard
    threading.Thread(target=connect_to_server_background, daemon=True).start()
    
    # Initialize GPIO motors
    init_gpio_motors()
    
    # Initialize camera
    if not init_dual_cameras():
        logger.error("❌ Camera init failed")
        return
    
    # Start camera loop
    try:
        camera_stream_loop()
    except KeyboardInterrupt:
        logger.info("🛑 Shutdown...")
        stop_event.set()
    finally:
        # Cleanup
        if camera_nav:
            camera_nav.release()
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        logger.info("👋 Goodbye!")

if __name__ == "__main__":
    main()
