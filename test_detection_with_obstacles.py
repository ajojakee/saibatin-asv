import cv2
import numpy as np

# Gunakan kamera eksternal
cap = cv2.VideoCapture(0)  # ← Index 0 untuk Logitech C270

# Atur resolusi
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Ambil satu frame untuk hitung tengah
ret, frame = cap.read()
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
center_frame = frame_width // 2
threshold = 50
min_area = 500  # area minimum untuk bola

# Obstacle detection parameters
OBSTACLE_MIN_AREA = 800  # Area minimum untuk obstacle (lebih besar dari bola)
OBSTACLE_MIN_WIDTH = 40  # Lebar minimum obstacle dalam pixels
FOCAL_LENGTH = 600  # Focal length kamera (approximate)
OBSTACLE_REAL_WIDTH = 0.3  # Lebar obstacle dalam meter (estimate)

kernel = np.ones((5, 5), np.uint8)

# Color ranges untuk ball detection (SAMA dengan production code)
RED_LOWER1 = np.array([0, 120, 150])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 150])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([45, 120, 120])
GREEN_UPPER = np.array([70, 255, 255])

def detect_obstacles(frame):
    """
    Detect obstacles using edge detection + contours
    Returns: list of obstacles with bounding boxes and estimated distance
    """
    obstacles = []
    
    # Convert to grayscale for edge detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Edge detection (Canny)
    edges = cv2.Canny(blurred_gray, 50, 150)
    
    # Dilate edges to connect nearby edges
    edges_dilated = cv2.dilate(edges, kernel, iterations=1)
    
    # Find contours
    contours, _ = cv2.findContours(edges_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        area = cv2.contourArea(contour)
        
        # Filter by minimum area
        if area < OBSTACLE_MIN_AREA:
            continue
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(contour)
        
        # Filter by minimum width
        if w < OBSTACLE_MIN_WIDTH:
            continue
        
        # Calculate center
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Estimate distance (simple size-based)
        # Distance = (Real_Width * Focal_Length) / Pixel_Width
        if w > 0:
            distance = (OBSTACLE_REAL_WIDTH * FOCAL_LENGTH) / w
            distance = min(distance, 20.0)  # Cap at 20m
        else:
            distance = 20.0
        
        # Calculate bearing (angle from center)
        offset_from_center = center_x - (frame_width / 2)
        bearing_deg = (offset_from_center / frame_width) * 60  # FOV ~60 degrees
        
        obstacles.append({
            'bbox': (x, y, w, h),
            'center': (center_x, center_y),
            'distance': round(distance, 2),
            'bearing': round(bearing_deg, 1),
            'area': int(area)
        })
    
    # Sort by distance (closest first)
    obstacles.sort(key=lambda o: o['distance'])
    
    # Keep only top 3 closest obstacles
    return obstacles[:3]

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ========== BALL DETECTION (GATE) ==========
    # Blur untuk mengurangi noise
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Mask untuk merah dan hijau
    mask_red = cv2.bitwise_or(
        cv2.inRange(hsv, RED_LOWER1, RED_UPPER1),
        cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    )
    mask_green = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

    # Bersihkan noise
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.dilate(mask_red, kernel, iterations=2)
    mask_green = cv2.dilate(mask_green, kernel, iterations=2)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_center_x = None
    green_center_x = None

    # Deteksi merah
    if contours_red:
        largest_red = max(contours_red, key=cv2.contourArea)
        if cv2.contourArea(largest_red) > min_area:
            x, y, w, h = cv2.boundingRect(largest_red)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            red_center_x = x + w // 2
            cv2.line(frame, (red_center_x, 0), (red_center_x, frame_height), (0, 0, 255), 2)
            cv2.putText(frame, "RED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Deteksi hijau
    if contours_green:
        largest_green = max(contours_green, key=cv2.contourArea)
        if cv2.contourArea(largest_green) > min_area:
            x, y, w, h = cv2.boundingRect(largest_green)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            green_center_x = x + w // 2
            cv2.line(frame, (green_center_x, 0), (green_center_x, frame_height), (0, 255, 0), 2)
            cv2.putText(frame, "GREEN", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # ========== OBSTACLE DETECTION ==========
    obstacles = detect_obstacles(frame)
    
    # Draw obstacles
    for i, obs in enumerate(obstacles):
        x, y, w, h = obs['bbox']
        cx, cy = obs['center']
        
        # Draw bounding box (YELLOW untuk obstacle)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        
        # Draw center point
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
        
        # Draw distance & bearing info
        info_text = f"OBS {i+1}: {obs['distance']}m, {obs['bearing']:.0f}deg"
        cv2.putText(frame, info_text, (x, y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # ========== NAVIGATION LOGIC ==========
    direction = ""
    if red_center_x is not None and green_center_x is not None:
        midpoint = (red_center_x + green_center_x) // 2
        cv2.line(frame, (midpoint, 0), (midpoint, frame_height), (255, 0, 0), 3)
        cv2.circle(frame, (midpoint, frame_height // 2), 15, (255, 0, 0), -1)

        if midpoint < center_frame - threshold:
            direction = "Belok Kiri"
            color = (0, 165, 255)  # Orange
        elif midpoint > center_frame + threshold:
            direction = "Belok Kanan"
            color = (147, 20, 255)  # Deep Pink
        else:
            direction = "Lurus"
            color = (0, 255, 0)  # Green
        
        # Display direction with colored background
        cv2.putText(frame, direction, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)
    else:
        # No gate detected
        cv2.putText(frame, "Cari Gate...", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)

    # Garis tengah putih (target center)
    cv2.line(frame, (center_frame, 0), (center_frame, frame_height), (255, 255, 255), 2)
    
    # Threshold zone (gray rectangles)
    cv2.rectangle(frame, (center_frame - threshold, 0), 
                  (center_frame + threshold, frame_height), (128, 128, 128), 1)
    
    # Obstacle count indicator
    obstacle_text = f"Obstacles: {len(obstacles)}"
    cv2.putText(frame, obstacle_text, (10, frame_height - 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # Instructions
    cv2.putText(frame, "Q: Quit | S: Screenshot", (10, frame_height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow('SAIBATIN - Gate & Obstacle Detection', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        # Save screenshot
        timestamp = cv2.getTickCount()
        filename = f"detection_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"💾 Screenshot saved: {filename}")

cap.release()
cv2.destroyAllWindows()
