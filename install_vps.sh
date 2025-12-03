#!/bin/bash
# Auto-install SAIBATIN AZURA Dashboard di VPS

echo "🚀 Installing SAIBATIN AZURA Dashboard..."

# Update system
apt update && apt upgrade -y

# Install dependencies
apt install -y python3 python3-pip nginx ufw

# Install Python packages
pip3 install flask flask-socketio flask-cors python-socketio

# Create directory
mkdir -p /var/www/saibatin

# Set permissions
chmod -R 755 /var/www/saibatin

# Create systemd service
cat > /etc/systemd/system/saibatin.service << 'EOF'
[Unit]
Description=SAIBATIN AZURA Dashboard
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/var/www/saibatin
ExecStart=/usr/bin/python3 /var/www/saibatin/server_dashboard.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# Enable service
systemctl enable saibatin

# Setup firewall
ufw allow 22/tcp
ufw allow 80/tcp
ufw allow 443/tcp
ufw allow 5000/tcp
ufw --force enable

echo "✅ Installation complete!"
echo ""
echo "📋 Next steps:"
echo "1. Upload files: scp -r 'C:\Users\Zaky\Downloads\baru baru\*' root@YOUR_IP:/var/www/saibatin/"
echo "2. Start service: systemctl start saibatin"
echo "3. Check status: systemctl status saibatin"
echo "4. Access dashboard: http://YOUR_IP:5000/index.html"
