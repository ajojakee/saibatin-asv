#!/bin/bash
echo "========================================="
echo "   BRICA PRO WiFi Setup Helper"
echo "========================================="
echo ""

# Step 1: Check WiFi interfaces
echo "Step 1: Checking WiFi interfaces..."
ifconfig | grep wlan

# Step 2: Scan Brica WiFi
echo ""
echo "Step 2: Scanning for Brica WiFi..."
sudo iwlist wlan1 scan 2>/dev/null | grep -E "ESSID|Quality"

# Step 3: Test connection
echo ""
echo "Step 3: Testing Brica connection..."
echo "   Trying to ping 192.168.0.1..."

if ping -c 3 -I wlan1 192.168.0.1 >/dev/null 2>&1; then
    echo "   ✅ Brica WiFi connected!"
    
    # Step 4: Test HTTP API
    echo ""
    echo "Step 4: Testing Brica HTTP API..."
    
    # Test video stream
    echo "   Testing MJPEG stream..."
    if curl -s -I http://192.168.0.1:8080/video | grep -q "200 OK"; then
        echo "   ✅ MJPEG stream: OK"
    else
        echo "   ❌ MJPEG stream: Failed"
    fi
    
    # Test photo capture
    echo "   Testing photo capture..."
    if curl -s -o /tmp/brica_test.jpg http://192.168.0.1:8080/photo; then
        if file /tmp/brica_test.jpg | grep -q JPEG; then
            echo "   ✅ Photo capture: OK"
        else
            echo "   ⚠️ Photo capture: Response not JPEG"
        fi
    else
        echo "   ❌ Photo capture: Failed"
    fi
    
else
    echo "   ❌ Cannot connect to Brica WiFi!"
    echo ""
    echo "Troubleshooting:"
    echo "1. Check if Brica WiFi is ON (LED blinking)"
    echo "2. Check SSID and password in wpa_supplicant"
    echo "3. Try: sudo systemctl restart wpa_supplicant@wlan1"
fi

echo ""
echo "========================================="
echo "   Setup Complete!"
echo "========================================="
