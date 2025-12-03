#!/usr/bin/env python3
"""
Direct ESC control via Raspberry Pi GPIO (Bypass Pixhawk)
WARNING: Only use if Pixhawk control fails
"""
import time

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("ERROR: RPi.GPIO not installed!")
    print("Run: sudo apt-get install python3-rpi.gpio")
    exit(1)

# GPIO pins (BCM numbering)
MOTOR_LEFT_PIN = 12   # GPIO12 (PWM0)
MOTOR_RIGHT_PIN = 13  # GPIO13 (PWM1)

# PWM settings
PWM_FREQ = 50  # 50Hz for ESC
PWM_MIN = 5.5  # 1100µs (full reverse)
PWM_MID = 7.5  # 1500µs (stop)
PWM_MAX = 9.5  # 1900µs (full forward)

print("=" * 60)
print("   DIRECT GPIO MOTOR TEST (Bypass Pixhawk)")
print("=" * 60)
print("\n⚠️ WARNING: This bypasses Pixhawk!")
print("   Only use for emergency testing\n")

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_PIN, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_PIN, GPIO.OUT)

# Create PWM objects
pwm_left = GPIO.PWM(MOTOR_LEFT_PIN, PWM_FREQ)
pwm_right = GPIO.PWM(MOTOR_RIGHT_PIN, PWM_FREQ)

# Start PWM at neutral
pwm_left.start(PWM_MID)
pwm_right.start(PWM_MID)
time.sleep(2)

print("Testing motors...")
print("\n1. Left motor forward")
pwm_left.ChangeDutyCycle(8.0)  # 1600µs
time.sleep(3)

print("2. Left motor stop")
pwm_left.ChangeDutyCycle(PWM_MID)
time.sleep(1)

print("3. Right motor forward")
pwm_right.ChangeDutyCycle(8.0)
time.sleep(3)

print("4. Right motor stop")
pwm_right.ChangeDutyCycle(PWM_MID)
time.sleep(1)

print("\n✅ Test complete")

# Cleanup
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()
