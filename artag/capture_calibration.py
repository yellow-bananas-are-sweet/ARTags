import cv2
import os

# Create output directory if it doesn't exist
output_dir = "calib_images"
os.makedirs(output_dir, exist_ok=True)

# Start video capture (0 for PiCam, change if using USB cam)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

print("📷 Press SPACE to save image | Press Q to quit")

img_count = 1
while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Can't receive frame. Exiting...")
        break

    cv2.imshow('Live Camera - Calibration Mode', frame)

    key = cv2.waitKey(1)
    if key % 256 == 32:  # Spacebar
        img_name = f"{output_dir}/calib_img_{img_count}.jpg"
        cv2.imwrite(img_name, frame)
        print(f"✅ Saved {img_name}")
        img_count += 1
    elif key & 0xFF == ord('q'):
        print("👋 Quitting...")
        break

cap.release()
cv2.destroyAllWindows()