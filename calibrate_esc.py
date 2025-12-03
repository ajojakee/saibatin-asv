#!/usr/bin/env python3
"""
ESC Calibration Tool - Calibrate PWM range for ESC
MUST RUN BEFORE MOTOR TEST!
"""

import time
import sys

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("❌ ERROR: RPi.GPIO not installed!")
    sys.exit(1)

# GPIO Pins (sesuaikan dengan wiring Anda)
MOTOR_LEFT_PIN = 18   # GPIO 18 - Motor Kiri
MOTOR_RIGHT_PIN = 13  # GPIO 13 - Motor Kanan

PWM_FREQ = 50

print("=" * 60)
print("   ESC CALIBRATION TOOL")
print("=" * 60)
print("\n⚠️ CRITICAL: Follow these steps EXACTLY:\n")
print("1. Disconnect ESC power (battery OFF)")
print("2. Script akan set PWM ke MAX (2000µs)")
print("3. Connect ESC power (battery ON)")
print("4. Wait for ESC beep sequence")
print("5. Script akan set PWM ke MIN (1000µs)")
print("6. ESC akan beep konfirmasi = CALIBRATED!\n")

print("📌 Wiring Check:")
print(f"   Motor LEFT  → GPIO {MOTOR_LEFT_PIN}")
print(f"   Motor RIGHT → GPIO {MOTOR_RIGHT_PIN}")
print("   ESC Ground  → Raspberry Pi Ground")
print("   ESC Power   → Battery 12V (DISCONNECT NOW!)\n")

input("Press ENTER when battery DISCONNECTED...")

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_PIN, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_PIN, GPIO.OUT)

pwm_left = GPIO.PWM(MOTOR_LEFT_PIN, PWM_FREQ)
pwm_right = GPIO.PWM(MOTOR_RIGHT_PIN, PWM_FREQ)

print("\n" + "=" * 60)
print("STEP 1: Setting PWM to MAX (2000µs = 10% duty cycle)")
print("=" * 60)

# Start PWM at MAX
pwm_left.start(10.0)   # 2000µs
pwm_right.start(10.0)  # 2000µs

print("✅ PWM set to MAX")
print("\n⚠️ NOW: Connect ESC power (battery ON)")
print("   Wait for ESC beep sequence (usually 2-3 beeps)")

input("\nPress ENTER after hearing ESC beeps...")

print("\n" + "=" * 60)
print("STEP 2: Setting PWM to MIN (1000µs = 5% duty cycle)")
print("=" * 60)

# Set PWM to MIN
pwm_left.ChangeDutyCycle(5.0)   # 1000µs
pwm_right.ChangeDutyCycle(5.0)  # 1000µs

print("✅ PWM set to MIN")
print("\n🎵 Listen for confirmation beep from ESC...")
print("   (Usually 1 long beep = calibration success)")

time.sleep(3)

print("\n" + "=" * 60)
print("STEP 3: Setting PWM to NEUTRAL (1500µs = 7.5% duty cycle)")
print("=" * 60)

pwm_left.ChangeDutyCycle(7.5)   # 1500µs (neutral)
pwm_right.ChangeDutyCycle(7.5)

print("✅ PWM set to NEUTRAL")

print("\n" + "=" * 60)
print("ESC CALIBRATION COMPLETE!")
print("=" * 60)
print("\n✅ ESC now knows PWM range: 1000-2000µs")
print("✅ Motor ready for testing")
print("\nNext: Run python3 test_motor_gpio_direct.py")
print("=" * 60)

input("\nPress ENTER to exit and stop PWM...")

# Cleanup
pwm_left.stop()
pwm_right.stop()
GPIO.cleanup()

print("👋 Goodbye!")
