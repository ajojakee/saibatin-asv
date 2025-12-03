#!/usr/bin/env python3
"""
DIRECT MOTOR TEST - Bypass Pixhawk, Control ESC via Raspberry Pi GPIO
Use this to verify ESC & motor hardware working independently
"""

import time
import sys

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("❌ ERROR: RPi.GPIO not installed!")
    print("Run: sudo apt-get install python3-rpi.gpio")
    sys.exit(1)

# ========== GPIO PIN CONFIGURATION ==========
MOTOR_LEFT_PIN = 12   # GPIO 12 (PWM0) - Connect ESC Left signal wire here
MOTOR_RIGHT_PIN = 13  # GPIO 13 (PWM1) - Connect ESC Right signal wire here

# PWM Configuration
PWM_FREQ = 50  # 50 Hz for ESC (standard)

# PWM Duty Cycle (for 50Hz):
# 1000µs = 5.0%  (Full Reverse)
# 1500µs = 7.5%  (Neutral/Stop)
# 2000µs = 10.0% (Full Forward)
DUTY_MIN = 5.0   # 1000µs
DUTY_MID = 7.5   # 1500µs
DUTY_MAX = 10.0  # 2000µs

print("=" * 60)
print("   SAIBATIN - DIRECT MOTOR TEST (GPIO)")
print("=" * 60)
print("\n⚠️ WARNING: This bypasses Pixhawk!")
print("   Only use for hardware verification\n")

print("📌 GPIO Pin Wiring:")
print(f"   Motor LEFT ESC signal  → GPIO {MOTOR_LEFT_PIN} (Pin 32)")
print(f"   Motor RIGHT ESC signal → GPIO {MOTOR_RIGHT_PIN} (Pin 33)")
print("   ESC Ground             → Raspberry Pi Ground")
print("   ESC Power              → Battery 12V (NOT Raspberry Pi!)\n")

input("Press ENTER when ready (Ctrl+C to cancel)...")

# ========== GPIO SETUP ==========
print("\n🔧 Setting up GPIO...")
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_PIN, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_PIN, GPIO.OUT)

# Create PWM objects
pwm_left = GPIO.PWM(MOTOR_LEFT_PIN, PWM_FREQ)
pwm_right = GPIO.PWM(MOTOR_RIGHT_PIN, PWM_FREQ)

# Start PWM at neutral (1500µs)
print("   Starting PWM at neutral (1500µs)...")
pwm_left.start(DUTY_MID)
pwm_right.start(DUTY_MID)
time.sleep(2)
print("✅ GPIO initialized\n")

# ========== MOTOR TEST FUNCTIONS ==========
def test_left_motor():
    """Test left motor"""
    print("=" * 60)
    print("TEST: LEFT MOTOR")
    print("=" * 60)
    
    print("\n1. Forward (1600µs)...")
    pwm_left.ChangeDutyCycle(8.0)  # 1600µs
    time.sleep(3)
    
    print("2. Stop (1500µs)...")
    pwm_left.ChangeDutyCycle(DUTY_MID)
    time.sleep(1)
    
    print("3. Reverse (1400µs)...")
    pwm_left.ChangeDutyCycle(7.0)  # 1400µs
    time.sleep(3)
    
    print("4. Stop (1500µs)...")
    pwm_left.ChangeDutyCycle(DUTY_MID)
    time.sleep(1)
    
    print("✅ Left motor test complete\n")

def test_right_motor():
    """Test right motor"""
    print("=" * 60)
    print("TEST: RIGHT MOTOR")
    print("=" * 60)
    
    print("\n1. Forward (1600µs)...")
    pwm_right.ChangeDutyCycle(8.0)
    time.sleep(3)
    
    print("2. Stop (1500µs)...")
    pwm_right.ChangeDutyCycle(DUTY_MID)
    time.sleep(1)
    
    print("3. Reverse (1400µs)...")
    pwm_right.ChangeDutyCycle(7.0)
    time.sleep(3)
    
    print("4. Stop (1500µs)...")
    pwm_right.ChangeDutyCycle(DUTY_MID)
    time.sleep(1)
    
    print("✅ Right motor test complete\n")

def test_both_motors():
    """Test both motors together"""
    print("=" * 60)
    print("TEST: BOTH MOTORS")
    print("=" * 60)
    
    print("\n1. Both Forward (1600µs)...")
    pwm_left.ChangeDutyCycle(8.0)
    pwm_right.ChangeDutyCycle(8.0)
    time.sleep(3)
    
    print("2. Both Stop...")
    pwm_left.ChangeDutyCycle(DUTY_MID)
    pwm_right.ChangeDutyCycle(DUTY_MID)
    time.sleep(1)
    
    print("3. Turn Left (left slow, right fast)...")
    pwm_left.ChangeDutyCycle(7.2)  # 1550µs
    pwm_right.ChangeDutyCycle(8.5)  # 1700µs
    time.sleep(3)
    
    print("4. Stop...")
    pwm_left.ChangeDutyCycle(DUTY_MID)
    pwm_right.ChangeDutyCycle(DUTY_MID)
    time.sleep(1)
    
    print("5. Turn Right (left fast, right slow)...")
    pwm_left.ChangeDutyCycle(8.5)
    pwm_right.ChangeDutyCycle(7.2)
    time.sleep(3)
    
    print("6. Stop...")
    pwm_left.ChangeDutyCycle(DUTY_MID)
    pwm_right.ChangeDutyCycle(DUTY_MID)
    time.sleep(1)
    
    print("✅ Both motors test complete\n")

# ========== INTERACTIVE MENU ==========
try:
    while True:
        print("=" * 60)
        print("MOTOR TEST MENU")
        print("=" * 60)
        print("1. Test Left Motor")
        print("2. Test Right Motor")
        print("3. Test Both Motors")
        print("4. Stop All Motors")
        print("5. Exit")
        print("=" * 60)
        
        choice = input("\nSelect test (1-5): ").strip()
        
        if choice == '1':
            test_left_motor()
        
        elif choice == '2':
            test_right_motor()
        
        elif choice == '3':
            test_both_motors()
        
        elif choice == '4':
            print("\n🛑 Stopping all motors...")
            pwm_left.ChangeDutyCycle(DUTY_MID)
            pwm_right.ChangeDutyCycle(DUTY_MID)
            print("✅ All motors stopped\n")
        
        elif choice == '5':
            print("\n🛑 Exiting...")
            break
        
        else:
            print("❌ Invalid choice. Enter 1-5\n")

except KeyboardInterrupt:
    print("\n\n🛑 Interrupted!")

finally:
    # Cleanup
    print("\n🧹 Cleaning up GPIO...")
    pwm_left.ChangeDutyCycle(DUTY_MID)
    pwm_right.ChangeDutyCycle(DUTY_MID)
    time.sleep(0.5)
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    print("✅ GPIO cleanup complete")
    print("👋 Goodbye!\n")
