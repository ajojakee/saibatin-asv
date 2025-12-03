# SAIBATIN AZURA 1.0 - Raspberry Pi Setup Guide

## Hardware Requirements
- Raspberry Pi 4B (4GB RAM recommended)
- Raspberry Pi Camera Module v2 (CSI interface)
- Pixhawk Flight Controller
- USB to Serial adapter (untuk koneksi Pixhawk)
- Power supply 5V 3A

## Koneksi Hardware

### 1. Camera CSI
- Hubungkan Camera Module ke CSI port di Raspberry Pi
- Pastikan kabel terpasang dengan benar (kontak menghadap ke HDMI)

### 2. Pixhawk Connection
**Opsi A: USB Connection**
- Hubungkan Pixhawk USB port ke Raspberry Pi USB port
- Port akan muncul sebagai `/dev/ttyACM0`

**Opsi B: UART/Serial Connection**
- TX Pixhawk → RX Raspberry Pi (GPIO 15)
- RX Pixhawk → TX Raspberry Pi (GPIO 14)
- GND Pixhawk → GND Raspberry Pi
- Port akan muncul sebagai `/dev/ttyAMA0`

## Software Installation

### 1. Clone repository dan masuk ke folder
```bash
cd ~/
git clone <repository-url> saibatin-azura
cd saibatin-azura
```

### 2. Jalankan script instalasi
```bash
chmod +x install_raspberry.sh
./install_raspberry.sh
```

### 3. Reboot
```bash
sudo reboot
```

## Konfigurasi

### 1. Edit file `raspberry_pi_main.py`
```python
# Sesuaikan IP server dashboard
SOCKETIO_SERVER = "http://YOUR_DASHBOARD_IP:5000"

# Sesuaikan port Pixhawk
PIXHAWK_PORT = "/dev/ttyACM0"  # atau /dev/ttyAMA0
```

### 2. Kalibrasi Warna Bola
Jalankan script kalibrasi:
```bash
python3 calibrate_colors.py
```

## Menjalankan Program

### Manual Start
```bash
python3 raspberry_pi_main.py
```

### Auto Start on Boot
```bash
sudo nano /etc/rc.local
```

Tambahkan sebelum `exit 0`:
```bash
cd /home/pi/saibatin-azura
python3 raspberry_pi_main.py > /home/pi/azura.log 2>&1 &
```

## Testing

### 1. Test Camera
```bash
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Error')"
```

### 2. Test Pixhawk Connection
```bash
python3 -c "from pymavlink import mavutil; m = mavutil.mavlink_connection('/dev/ttyACM0'); m.wait_heartbeat(); print('Pixhawk OK')"
```

### 3. Test Socket.IO Connection
```bash
python3 -c "import socketio; sio = socketio.Client(); sio.connect('http://YOUR_IP:5000'); print('Socket.IO OK')"
```

## Troubleshooting

### Camera tidak terdeteksi
```bash
vcgencmd get_camera
# Output harus: supported=1 detected=1
```

### Pixhawk tidak connect
```bash
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*
# Pastikan port muncul dan user memiliki akses
```

### Permission denied untuk serial
```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

## Mission Flow

1. **Startup** → System initialization
2. **Searching** → Mencari bola merah dan hijau
3. **One Ball Detected** → Satu bola terdeteksi, cari bola kedua
4. **Both Balls Detected** → Kedua bola terdeteksi, navigasi di antara keduanya
5. **Navigating** → ASV bergerak menuju dan melewati gap antara 2 bola
6. **Mission Complete** → Berhasil melewati kedua bola

## Performance Tips

1. **Reduce Camera Resolution** untuk performa lebih cepat:
   ```python
   CAMERA_WIDTH = 480
   CAMERA_HEIGHT = 360
   ```

2. **Adjust JPEG Quality** untuk bandwidth lebih rendah:
   ```python
   cv2.IMWRITE_JPEG_QUALITY, 60  # default 80
   ```

3. **Increase Detection Threshold** jika terlalu banyak false positive:
   ```python
   MIN_BALL_RADIUS = 20  # default 15
   ```

## Support
Untuk bantuan lebih lanjut, hubungi tim SAIBATIN AZURA 1.0
Institut Teknologi Sumatera
