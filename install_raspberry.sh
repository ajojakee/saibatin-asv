#!/bin/bash

echo "🚀 SAIBATIN AZURA 1.0 - Raspberry Pi Setup"
echo "=========================================="

# Update system
echo "📦 Updating system..."
sudo apt update
sudo apt upgrade -y

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
sudo apt install -y python3-pip python3-opencv
pip3 install -r requirements_raspberry.txt

# Setup camera interface
echo "📷 Enabling camera interface..."
sudo raspi-config nonint do_camera 0

# Setup serial for Pixhawk
echo "🛰️ Configuring serial for Pixhawk..."
sudo raspi-config nonint do_serial 2

# Add user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

echo "✅ Installation complete!"
echo "⚠️ Please reboot your Raspberry Pi: sudo reboot"
