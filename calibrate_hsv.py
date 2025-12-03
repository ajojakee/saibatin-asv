import cv2
import numpy as np

# Buka kamera
cap = cv2.VideoCapture(0)

print("📸 HSV Calibration Tool")
print("Tekan 'r' untuk capture RED ball")
print("Tekan 'g' untuk capture GREEN ball")
print("Tekan 'q' untuk quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    cv2.imshow('Camera', frame)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('r'):
        # Capture red ball area (klik tengah bola)
        roi = frame[200:300, 200:300]  # Adjust area ini!
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        print("\n🔴 RED BALL HSV Range:")
        print(f"H: {hsv[:,:,0].min()}-{hsv[:,:,0].max()}")
        print(f"S: {hsv[:,:,1].min()}-{hsv[:,:,1].max()}")
        print(f"V: {hsv[:,:,2].min()}-{hsv[:,:,2].max()}")
        print("\nUpdate RED_LOWER1 dan RED_UPPER1 dengan nilai ini!")
        
    elif key == ord('g'):
        # Capture green ball area
        roi = frame[200:300, 200:300]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        print("\n🟢 GREEN BALL HSV Range:")
        print(f"H: {hsv[:,:,0].min()}-{hsv[:,:,0].max()}")
        print(f"S: {hsv[:,:,1].min()}-{hsv[:,:,1].max()}")
        print(f"V: {hsv[:,:,2].min()}-{hsv[:,:,2].max()}")
        print("\nUpdate GREEN_LOWER dan GREEN_UPPER dengan nilai ini!")
        
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
