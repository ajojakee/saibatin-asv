import cv2
import numpy as np

# Gunakan kamera eksternal
cap = cv2.VideoCapture(0)  # Ganti ke 0 jika di Raspberry Pi

# Atur resolusi (lebih ringan dan stabil)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Ambil satu frame untuk hitung tengah
ret, frame = cap.read()
if not ret:
    print("ERROR: Cannot read camera")
    exit(1)

frame_width = int(cap.get(3))
center_frame = frame_width // 2
threshold = 50
min_area = 500  # area minimum biar objek kecil gak ke-detect

kernel = np.ones((5, 5), np.uint8)

print("=" * 60)
print("   HSV CALIBRATION - SAIBATIN AZURA 1.0")
print("=" * 60)
print("Controls:")
print("  Q - Quit")
print("  S - Screenshot current HSV values")
print("=" * 60)

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # Blur untuk mengurangi noise
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)

    # Konversi HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Warna merah (range diperketat)
    lower_red1 = np.array([0, 120, 150])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 150])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    # Warna hijau (range diperketat)
    lower_green = np.array([45, 120, 120])
    upper_green = np.array([70, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Bersihkan noise kecil
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_center_x = None
    green_center_x = None
    red_detected = False
    green_detected = False

    # Deteksi merah
    if contours_red:
        largest_red = max(contours_red, key=cv2.contourArea)
        area_red = cv2.contourArea(largest_red)
        if area_red > min_area:
            x, y, w, h = cv2.boundingRect(largest_red)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_center_x = x + w // 2
            cv2.line(frame, (red_center_x, 0), (red_center_x, frame.shape[0]), (0, 0, 255), 2)
            cv2.putText(frame, f"RED: {area_red}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            red_detected = True

    # Deteksi hijau
    if contours_green:
        largest_green = max(contours_green, key=cv2.contourArea)
        area_green = cv2.contourArea(largest_green)
        if area_green > min_area:
            x, y, w, h = cv2.boundingRect(largest_green)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            green_center_x = x + w // 2
            cv2.line(frame, (green_center_x, 0), (green_center_x, frame.shape[0]), (0, 255, 0), 2)
            cv2.putText(frame, f"GREEN: {area_green}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            green_detected = True

    # Logika arah
    direction = ""
    if red_center_x is not None and green_center_x is not None:
        midpoint = (red_center_x + green_center_x) // 2
        cv2.line(frame, (midpoint, 0), (midpoint, frame.shape[0]), (255, 0, 0), 3)
        cv2.circle(frame, (midpoint, frame.shape[0]//2), 10, (255, 0, 0), -1)

        offset = midpoint - center_frame

        if midpoint < center_frame - threshold:
            direction = f"KIRI ({offset}px)"
        elif midpoint > center_frame + threshold:
            direction = f"KANAN (+{offset}px)"
        else:
            direction = f"LURUS ({offset}px)"

    # Garis tengah putih
    cv2.line(frame, (center_frame, 0), (center_frame, frame.shape[0]), (255, 255, 255), 2)
    
    # Status overlay
    cv2.putText(frame, f"RED: {'DETECTED' if red_detected else 'NOT DETECTED'}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if red_detected else (128, 128, 128), 2)
    cv2.putText(frame, f"GREEN: {'DETECTED' if green_detected else 'NOT DETECTED'}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if green_detected else (128, 128, 128), 2)
    cv2.putText(frame, f"Direction: {direction}", 
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(frame, f"Frame: {frame_count}", 
                (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, "Q=Quit | S=Screenshot", 
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Show frame
    cv2.imshow('HSV Calibration - SAIBATIN', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\nExiting...")
        break
    elif key == ord('s'):
        filename = f"hsv_calibration_{frame_count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Screenshot saved: {filename}")
        print(f"  Red detected: {red_detected}")
        print(f"  Green detected: {green_detected}")
        print(f"  Direction: {direction}")

cap.release()
cv2.destroyAllWindows()
print("Calibration completed!")
