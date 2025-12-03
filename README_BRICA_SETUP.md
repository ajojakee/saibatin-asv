# BRICA PRO WiFi Setup Guide

## 📸 Brica PRO B-PRO 5 Alpha Edition 2 (AE2) WiFi Configuration

### 1. Connect to Brica WiFi

```bash
SSID: BPRO5_XXXXXX
Password: 1234567890 (default)
IP Address: 192.168.0.1 (default)
```

### 2. Brica HTTP API Endpoints

Brica PRO biasanya support HTTP API untuk capture photo:

**Option 1: Standard Endpoint**
```
http://192.168.0.1:8080/photo
http://192.168.0.1:8080/snapshot
```

**Option 2: CGI Proxy (untuk action cam berbasis AllWinner V3)**
```
http://192.168.0.1/cgi-bin/CGIProxy.fcgi?cmd=snapPicture2
```

**Option 3: MJPEG Stream + Frame Extraction**
```
http://192.168.0.1:8080/video
```

### 3. Test Brica API (dari Raspberry Pi)

```bash
# Test 1: Ping Brica
ping 192.168.0.1

# Test 2: Download snapshot
curl -o test.jpg http://192.168.0.1:8080/photo

# Test 3: Check HTTP response
curl -I http://192.168.0.1:8080/photo
```

### 4. Update Configuration

Edit file `saibatin_production_fixed.py`:

```python
# Line 65-66:
BRICA_WIFI_STREAM = "http://192.168.0.1:8080/video"
BRICA_PHOTO_CAPTURE_URL = "http://192.168.0.1:8080/photo"  # ← UPDATE INI!
USE_BRICA_WIFI = True  # Set True untuk WiFi mode
```

### 5. WiFi Network Configuration

**Raspberry Pi harus connect ke 2 WiFi:**
- WiFi 1: Hotspot laptop (untuk dashboard)
- WiFi 2: Brica WiFi (untuk underwater camera)

**Gunakan USB WiFi adapter tambahan untuk dual WiFi!**

```bash
# Cek WiFi interfaces
ifconfig

# Expected output:
# wlan0: connected to laptop hotspot (dashboard)
# wlan1: connected to Brica WiFi (camera)
```

### 6. HSV Calibration untuk Bola BIRU

```python
# Default threshold:
BLUE_LOWER = np.array([100, 150, 50])
BLUE_UPPER = np.array([130, 255, 255])

# Test di kolam real, adjust jika perlu!
```

### 7. Troubleshooting

| Problem | Solution |
|---------|----------|
| Cannot connect to Brica WiFi | Check password, restart Brica |
| HTTP 404 on photo endpoint | Try alternative endpoints |
| Timeout | Check Brica power, WiFi signal |
| Wrong color detection | Run HSV calibration script |

### 8. Test Auto-Trigger System

```bash
# Run system
python3 saibatin_production_fixed.py --server http://<LAPTOP_IP>:5000

# Expected behavior:
# 1. Camera navigasi deteksi bola BIRU
# 2. Log: "🔵 Blue ball detected! Triggering underwater photo"
# 3. Brica capture foto via WiFi
# 4. Foto saved ke captures/ folder
# 5. Foto emit ke dashboard
# 6. Cooldown 5 detik sebelum capture lagi
```

### 9. Alternative: Extract Frame dari MJPEG Stream

Jika Brica tidak support snapshot API:

```python
def capture_brica_photo_wifi():
    try:
        cap = cv2.VideoCapture('http://192.168.0.1:8080/video')
        ret, frame = cap.read()
        cap.release()
        
        if ret:
            return frame
        return None
    except:
        return None
```

---

## 🎯 Flow Sistem Auto Underwater Photo

```
1. Camera navigasi detect bola BIRU
   ↓
2. Check cooldown (5s)
   ↓
3. Trigger Brica capture via WiFi HTTP API
   ↓
4. Download foto dari Brica
   ↓
5. Save foto: captures/gateX_underwater_YYYYMMDD_HHMMSS.jpg
   ↓
6. Emit foto ke dashboard (base64 encoded)
   ↓
7. Set cooldown 5 detik
```

---

**Ready untuk outdoor test dengan auto-trigger underwater photo!** 🔵📸✅
