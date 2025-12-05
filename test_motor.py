#!/usr/bin/env python3
"""
MOTOR TEST PROGRAM - SAIBATIN AZURA 1.0
Test motor control langsung tanpa dashboard
"""

import time
import sys

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("❌ RPi.GPIO not available - cannot test motors")
    sys.exit(1)

# ========== CONFIG ==========
MOTOR_LEFT_GPIO = 18   # GPIO 18 - Motor Kiri
MOTOR_RIGHT_GPIO = 13  # GPIO 13 - Motor Kanan
PWM_FREQ = 50          # 50Hz for ESC

# ========== INIT GPIO ==========
def init_motors():
    """Initialize GPIO motors"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(MOTOR_LEFT_GPIO, GPIO.OUT)
    GPIO.setup(MOTOR_RIGHT_GPIO, GPIO.OUT)
    
    pwm_left = GPIO.PWM(MOTOR_LEFT_GPIO, PWM_FREQ)
    pwm_right = GPIO.PWM(MOTOR_RIGHT_GPIO, PWM_FREQ)
    
    # Start at neutral (1500µs = 7.5% duty cycle)
    pwm_left.start(7.5)
    pwm_right.start(7.5)
    
    print("✅ Motors initialized")
    print(f"   Left: GPIO {MOTOR_LEFT_GPIO}")
    print(f"   Right: GPIO {MOTOR_RIGHT_GPIO}")
    
    return pwm_left, pwm_right

def set_motor(pwm, speed_percent):
    """Set motor speed (-100 to +100)"""
    duty_cycle = 7.5 + (speed_percent / 100.0) * 2.5
    duty_cycle = max(5.0, min(10.0, duty_cycle))
    pwm.ChangeDutyCycle(duty_cycle)

def stop_motors(pwm_left, pwm_right):
    """Stop both motors"""
    set_motor(pwm_left, 0)
    set_motor(pwm_right, 0)
    print("🛑 Motors stopped")

# ========== TEST SEQUENCES ==========
def test_individual_motors(pwm_left, pwm_right):
    """Test each motor individually"""
    print("\n=== TEST 1: Individual Motor Test ===")
    
    # Test LEFT motor
    print("\n🔄 Testing LEFT motor...")
    print("   Forward 30% for 3 seconds...")
    set_motor(pwm_left, 30)
    time.sleep(3)
    
    print("   Stop for 2 seconds...")
    set_motor(pwm_left, 0)
    time.sleep(2)
    
    print("   Reverse 30% for 3 seconds...")
    set_motor(pwm_left, -30)
    time.sleep(3)
    
    print("   Stop")
    set_motor(pwm_left, 0)
    time.sleep(2)
    
    # Test RIGHT motor
    print("\n🔄 Testing RIGHT motor...")
    print("   Forward 30% for 3 seconds...")
    set_motor(pwm_right, 30)
    time.sleep(3)
    
    print("   Stop for 2 seconds...")
    set_motor(pwm_right, 0)
    time.sleep(2)
    
    print("   Reverse 30% for 3 seconds...")
    set_motor(pwm_right, -30)
    time.sleep(3)
    
    print("   Stop")
    set_motor(pwm_right, 0)
    time.sleep(2)

def test_forward_backward(pwm_left, pwm_right):
    """Test forward and backward movement"""
    print("\n=== TEST 2: Forward/Backward Test ===")
    
    print("\n➡️ Forward 50% for 5 seconds...")
    set_motor(pwm_left, 50)
    set_motor(pwm_right, 50)
    time.sleep(5)
    
    print("   Stop for 2 seconds...")
    stop_motors(pwm_left, pwm_right)
    time.sleep(2)
    
    print("⬅️ Backward 50% for 5 seconds...")
    set_motor(pwm_left, -50)
    set_motor(pwm_right, -50)
    time.sleep(5)
    
    print("   Stop")
    stop_motors(pwm_left, pwm_right)
    time.sleep(2)

def test_turning(pwm_left, pwm_right):
    """Test turning movements"""
    print("\n=== TEST 3: Turning Test ===")
    
    print("\n↪️ Turn RIGHT (left motor 50%, right 0%) for 3 seconds...")
    set_motor(pwm_left, 50)
    set_motor(pwm_right, 0)
    time.sleep(3)
    
    print("   Stop for 2 seconds...")
    stop_motors(pwm_left, pwm_right)
    time.sleep(2)
    
    print("↩️ Turn LEFT (right motor 50%, left 0%) for 3 seconds...")
    set_motor(pwm_left, 0)
    set_motor(pwm_right, 50)
    time.sleep(3)
    
    print("   Stop")
    stop_motors(pwm_left, pwm_right)
    time.sleep(2)

def test_differential_drive(pwm_left, pwm_right):
    """Test differential drive patterns"""
    print("\n=== TEST 4: Differential Drive Test ===")
    
    sequences = [
        ("Forward straight", 50, 50, 3),
        ("Gentle right turn", 70, 50, 3),
        ("Gentle left turn", 50, 70, 3),
        ("Sharp right turn", 70, 30, 3),
        ("Sharp left turn", 30, 70, 3),
        ("Spin right (on spot)", 50, -50, 3),
        ("Spin left (on spot)", -50, 50, 3),
    ]
    
    for description, left_speed, right_speed, duration in sequences:
        print(f"\n🔄 {description}...")
        print(f"   Left: {left_speed}%, Right: {right_speed}% for {duration}s")
        set_motor(pwm_left, left_speed)
        set_motor(pwm_right, right_speed)
        time.sleep(duration)
        
        print("   Stop for 2 seconds...")
        stop_motors(pwm_left, pwm_right)
        time.sleep(2)

def test_speed_ramp(pwm_left, pwm_right):
    """Test gradual speed increase/decrease"""
    print("\n=== TEST 5: Speed Ramp Test ===")
    
    print("\n⬆️ Ramping up (0% → 80%)...")
    for speed in range(0, 81, 10):
        print(f"   Speed: {speed}%")
        set_motor(pwm_left, speed)
        set_motor(pwm_right, speed)
        time.sleep(1)
    
    print("\n⬇️ Ramping down (80% → 0%)...")
    for speed in range(80, -1, -10):
        print(f"   Speed: {speed}%")
        set_motor(pwm_left, speed)
        set_motor(pwm_right, speed)
        time.sleep(1)
    
    print("   Stop")
    stop_motors(pwm_left, pwm_right)

# ========== INTERACTIVE MODE ==========
def interactive_mode(pwm_left, pwm_right):
    """Manual control mode"""
    print("\n=== INTERACTIVE MODE ===")
    print("Commands:")
    print("  w = Forward    s = Backward")
    print("  a = Left       d = Right")
    print("  q = Spin left  e = Spin right")
    print("  x = Stop       + = Increase speed")
    print("  - = Decrease speed")
    print("  0 = Exit")
    
    speed = 50
    
    while True:
        try:
            cmd = input(f"\nSpeed: {speed}% > ").strip().lower()
            
            if cmd == 'w':
                print(f"➡️ Forward {speed}%")
                set_motor(pwm_left, speed)
                set_motor(pwm_right, speed)
            
            elif cmd == 's':
                print(f"⬅️ Backward {speed}%")
                set_motor(pwm_left, -speed)
                set_motor(pwm_right, -speed)
            
            elif cmd == 'a':
                print(f"↩️ Left turn (R:{speed}%, L:0%)")
                set_motor(pwm_left, 0)
                set_motor(pwm_right, speed)
            
            elif cmd == 'd':
                print(f"↪️ Right turn (L:{speed}%, R:0%)")
                set_motor(pwm_left, speed)
                set_motor(pwm_right, 0)
            
            elif cmd == 'q':
                print(f"🔄 Spin left")
                set_motor(pwm_left, -speed)
                set_motor(pwm_right, speed)
            
            elif cmd == 'e':
                print(f"🔄 Spin right")
                set_motor(pwm_left, speed)
                set_motor(pwm_right, -speed)
            
            elif cmd == 'x':
                print("🛑 Stop")
                stop_motors(pwm_left, pwm_right)
            
            elif cmd == '+':
                speed = min(100, speed + 10)
                print(f"⬆️ Speed increased to {speed}%")
            
            elif cmd == '-':
                speed = max(0, speed - 10)
                print(f"⬇️ Speed decreased to {speed}%")
            
            elif cmd == '0':
                print("👋 Exiting interactive mode")
                break
            
            else:
                print("❌ Unknown command")
        
        except KeyboardInterrupt:
            print("\n🛑 Ctrl+C pressed")
            break
        except EOFError:
            break

# ========== MAIN ==========
def main():
    print("=" * 60)
    print("🚀 MOTOR TEST PROGRAM - SAIBATIN AZURA 1.0")
    print("=" * 60)
    
    try:
        # Initialize motors
        pwm_left, pwm_right = init_motors()
        
        print("\n⚠️ WARNING: ESC akan aktif dalam 5 detik!")
        print("   Pastikan propeller sudah dipasang atau dilepas!")
        for i in range(5, 0, -1):
            print(f"   {i}...")
            time.sleep(1)
        
        print("\n🎯 Starting motor tests...\n")
        
        # Menu
        while True:
            print("\n" + "=" * 60)
            print("MOTOR TEST MENU:")
            print("  1. Individual motor test")
            print("  2. Forward/Backward test")
            print("  3. Turning test")
            print("  4. Differential drive test")
            print("  5. Speed ramp test")
            print("  6. Interactive mode")
            print("  9. Run ALL tests")
            print("  0. Exit")
            print("=" * 60)
            
            choice = input("\nSelect test [0-9]: ").strip()
            
            if choice == '1':
                test_individual_motors(pwm_left, pwm_right)
            
            elif choice == '2':
                test_forward_backward(pwm_left, pwm_right)
            
            elif choice == '3':
                test_turning(pwm_left, pwm_right)
            
            elif choice == '4':
                test_differential_drive(pwm_left, pwm_right)
            
            elif choice == '5':
                test_speed_ramp(pwm_left, pwm_right)
            
            elif choice == '6':
                interactive_mode(pwm_left, pwm_right)
            
            elif choice == '9':
                print("\n🚀 Running ALL tests...")
                test_individual_motors(pwm_left, pwm_right)
                test_forward_backward(pwm_left, pwm_right)
                test_turning(pwm_left, pwm_right)
                test_differential_drive(pwm_left, pwm_right)
                test_speed_ramp(pwm_left, pwm_right)
                print("\n✅ All tests completed!")
            
            elif choice == '0':
                print("\n👋 Exiting...")
                break
            
            else:
                print("❌ Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\n🛑 Ctrl+C pressed - stopping...")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
    
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        try:
            stop_motors(pwm_left, pwm_right)
            pwm_left.stop()
            pwm_right.stop()
            GPIO.cleanup()
            print("✅ GPIO cleaned up")
        except:
            pass
        
        print("👋 Program terminated")

if __name__ == "__main__":
    main()
