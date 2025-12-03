#!/usr/bin/env python3
"""
Test Brica PRO capture (WiFi atau USB mode)
"""

import cv2
import requests
import numpy as np

BRICA_WIFI_STREAM = "http://192.168.0.1:8080/video"
BRICA_PHOTO_URL = "http://192.168.0.1:8080/photo"
USE_WIFI = True  # Change to False for USB mode

def test_brica_wifi():
    """Test Brica WiFi capture"""
    print("📸 Testing Brica WiFi capture...")
    
    try:
        response = requests.get(BRICA_PHOTO_URL, timeout=5)
        
        if response.status_code == 200:
            # Convert to OpenCV image
            img_array = np.frombuffer(response.content, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            if img is not None:
                cv2.imwrite('brica_test_wifi.jpg', img)
                print(f"✅ Success! Image saved: {img.shape}")
                return True
            else:
                print("❌ Failed to decode image")
                return False
        else:
            print(f"❌ HTTP {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def test_brica_usb():
    """Test Brica USB capture"""
    print("📸 Testing Brica USB capture...")
    
    try:
        cap = cv2.VideoCapture(1)  # /dev/video1
        
        if not cap.isOpened():
            print("❌ Cannot open camera index 1")
            return False
        
        ret, frame = cap.read()
        cap.release()
        
        if ret:
            cv2.imwrite('brica_test_usb.jpg', frame)
            print(f"✅ Success! Image saved: {frame.shape}")
            return True
        else:
            print("❌ Failed to read frame")
            return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def test_brica_stream_wifi():
    """Test Brica WiFi MJPEG stream"""
    print("📹 Testing Brica WiFi stream...")
    
    try:
        cap = cv2.VideoCapture(BRICA_WIFI_STREAM)
        
        if not cap.isOpened():
            print("❌ Cannot open WiFi stream")
            return False
        
        ret, frame = cap.read()
        cap.release()
        
        if ret:
            cv2.imwrite('brica_test_stream.jpg', frame)
            print(f"✅ Success! Stream frame: {frame.shape}")
            return True
        else:
            print("❌ Failed to read stream frame")
            return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    print("=" * 50)
    print("   BRICA PRO CAPTURE TEST")
    print("=" * 50)
    print()
    
    if USE_WIFI:
        print("Mode: WiFi")
        print()
        
        # Test 1: HTTP snapshot API
        print("Test 1: HTTP Snapshot API")
        test_brica_wifi()
        print()
        
        # Test 2: MJPEG stream
        print("Test 2: MJPEG Stream")
        test_brica_stream_wifi()
    else:
        print("Mode: USB")
        print()
        
        # Test USB camera
        test_brica_usb()
    
    print()
    print("✅ Test completed!")
