#!/usr/bin/env python3
"""
SAIBATIN AZURA 1.0 - Motor Test Script
Test 2 ESC motors connected to Pixhawk PWM Channel 4 & 5
"""

import time
import sys

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    print("ERROR: pymavlink not installed!")
    print("Run: pip3 install pymavlink")
    sys.exit(1)

# ========== CONFIGURATION ==========
PIXHAWK_PORTS = ["/dev/ttyAMA0", "/dev/ttyACM0", "/dev/ttyUSB0"]
PIXHAWK_BAUD = 57600

PWM_MIN = 1100  # Minimum PWM (full reverse)
PWM_MID = 1500  # Neutral PWM (stop)
PWM_MAX = 1900  # Maximum PWM (full forward)

# Motor channels
MOTOR_LEFT_CHANNEL = 4   # ESC 1 di channel 4
MOTOR_RIGHT_CHANNEL = 5  # ESC 2 di channel 5

# ========== CONNECT TO PIXHAWK ==========
def connect_pixhawk():
    print("=" * 60)
    print("   SAIBATIN MOTOR TEST")
    print("=" * 60)
    print("\nConnecting to Pixhawk...")
    
    for port in PIXHAWK_PORTS:
        try:
            print(f"  Trying {port}...")
            connection = mavutil.mavlink_connection(port, baud=PIXHAWK_BAUD)
            connection.wait_heartbeat(timeout=5)
            print(f"✅ Connected to Pixhawk on {port}")
            print(f"   System ID: {connection.target_system}")
            print(f"   Component ID: {connection.target_component}")
            return connection
        except Exception as e:
            print(f"  ❌ Failed: {e}")
            continue
    
    print("\n❌ ERROR: Cannot connect to Pixhawk!")
    print("Check:")
    print("  1. Pixhawk is powered on")
    print("  2. USB cable is connected")
    print("  3. Port permissions: sudo usermod -aG dialout $USER")
    sys.exit(1)

# ========== MOTOR CONTROL FUNCTIONS ==========
def set_motor_pwm(connection, channel, pwm_value):
    """
    Send PWM value to specific motor channel
    channel: 4 (left) or 5 (right)
    pwm_value: 1100-1900
    """
    pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
    
    # Create RC override array (18 channels for newer protocol)
    rc_channels = [65535] * 18  # 65535 = no change
    rc_channels[channel - 1] = pwm_value  # Set specific channel
    
    # Send RC override (UPDATED - use newer message format)
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        *rc_channels[:8]  # First 8 channels
    )
    
    # TAMBAHAN: Send servo command sebagai backup
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # confirmation
        channel,  # param1: servo number
        pwm_value,  # param2: PWM value
        0, 0, 0, 0, 0
    )
    
    return pwm_value

def stop_motors(connection):
    """Stop both motors (PWM = 1500)"""
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, PWM_MID)
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, PWM_MID)
    print("🛑 Motors STOPPED (PWM 1500)")

def test_motor_left(connection):
    """Test left motor (Channel 4)"""
    print("\n" + "=" * 60)
    print("TEST: LEFT MOTOR (Channel 4)")
    print("=" * 60)
    
    # Forward
    print("\n1. Forward (PWM 1600)...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1600)
    time.sleep(2)
    
    # Stop
    print("2. Stop (PWM 1500)...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, PWM_MID)
    time.sleep(1)
    
    # Reverse
    print("3. Reverse (PWM 1400)...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1400)
    time.sleep(2)
    
    # Stop
    print("4. Stop (PWM 1500)...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, PWM_MID)
    time.sleep(1)
    
    print("✅ Left motor test complete")

def test_motor_right(connection):
    """Test right motor (Channel 5)"""
    print("\n" + "=" * 60)
    print("TEST: RIGHT MOTOR (Channel 5)")
    print("=" * 60)
    
    # Forward
    print("\n1. Forward (PWM 1600)...")
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1600)
    time.sleep(2)
    
    # Stop
    print("2. Stop (PWM 1500)...")
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, PWM_MID)
    time.sleep(1)
    
    # Reverse
    print("3. Reverse (PWM 1400)...")
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1400)
    time.sleep(2)
    
    # Stop
    print("4. Stop (PWM 1500)...")
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, PWM_MID)
    time.sleep(1)
    
    print("✅ Right motor test complete")

def test_differential_drive(connection):
    """Test differential drive (both motors)"""
    print("\n" + "=" * 60)
    print("TEST: DIFFERENTIAL DRIVE (Both Motors)")
    print("=" * 60)
    
    # Straight forward
    print("\n1. Straight Forward...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1600)
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1600)
    time.sleep(3)
    stop_motors(connection)
    time.sleep(1)
    
    # Turn left (right motor faster)
    print("\n2. Turn Left...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1550)
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1650)
    time.sleep(3)
    stop_motors(connection)
    time.sleep(1)
    
    # Turn right (left motor faster)
    print("\n3. Turn Right...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1650)
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1550)
    time.sleep(3)
    stop_motors(connection)
    time.sleep(1)
    
    # Straight reverse
    print("\n4. Straight Reverse...")
    set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1400)
    set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1400)
    time.sleep(3)
    stop_motors(connection)
    
    print("✅ Differential drive test complete")

def manual_control(connection):
    """Manual motor control via keyboard"""
    print("\n" + "=" * 60)
    print("MANUAL CONTROL MODE")
    print("=" * 60)
    print("\nControls:")
    print("  W - Forward")
    print("  S - Reverse")
    print("  A - Turn Left")
    print("  D - Turn Right")
    print("  SPACE - Stop")
    print("  Q - Quit")
    print("\nPress keys to control motors...")
    
    try:
        import tty
        import termios
        
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        while True:
            char = sys.stdin.read(1).lower()
            
            if char == 'w':
                print("⬆️ Forward")
                set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1600)
                set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1600)
            
            elif char == 's':
                print("⬇️ Reverse")
                set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1400)
                set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1400)
            
            elif char == 'a':
                print("⬅️ Turn Left")
                set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1550)
                set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1650)
            
            elif char == 'd':
                print("➡️ Turn Right")
                set_motor_pwm(connection, MOTOR_LEFT_CHANNEL, 1650)
                set_motor_pwm(connection, MOTOR_RIGHT_CHANNEL, 1550)
            
            elif char == ' ':
                print("🛑 Stop")
                stop_motors(connection)
            
            elif char == 'q':
                print("\nExiting manual control...")
                stop_motors(connection)
                break
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
    except ImportError:
        print("⚠️ Manual control not available on this platform")
        print("Use automated tests instead")

def interactive_menu(connection):
    """Interactive test menu"""
    while True:
        print("\n" + "=" * 60)
        print("MOTOR TEST MENU")
        print("=" * 60)
        print("1. Test Left Motor (Channel 4)")
        print("2. Test Right Motor (Channel 5)")
        print("3. Test Differential Drive (Both Motors)")
        print("4. Manual Control (Keyboard)")
        print("5. Stop Motors")
        print("6. Exit")
        print("=" * 60)
        
        try:
            choice = input("\nSelect test (1-6): ").strip()
            
            if choice == '1':
                test_motor_left(connection)
            
            elif choice == '2':
                test_motor_right(connection)
            
            elif choice == '3':
                test_differential_drive(connection)
            
            elif choice == '4':
                manual_control(connection)
            
            elif choice == '5':
                stop_motors(connection)
            
            elif choice == '6':
                print("\n🛑 Stopping motors...")
                stop_motors(connection)
                print("👋 Goodbye!")
                break
            
            else:
                print("❌ Invalid choice. Enter 1-6")
        
        except KeyboardInterrupt:
            print("\n\n🛑 Interrupted! Stopping motors...")
            stop_motors(connection)
            break
        
        except Exception as e:
            print(f"❌ Error: {e}")

# ========== MAIN ==========
def main():
    try:
        # Connect to Pixhawk
        connection = connect_pixhawk()
        
        # Safety: Stop motors first
        print("\n🛑 Safety: Stopping motors...")
        stop_motors(connection)
        time.sleep(1)
        
        # Show menu
        interactive_menu(connection)
        
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted!")
        if 'connection' in locals():
            stop_motors(connection)
    
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n✅ Test completed")

if __name__ == "__main__":
    main()
