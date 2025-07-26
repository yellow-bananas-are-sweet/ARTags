from picamera2 import Picamera2
import cv2
import time

picam2 = Picamera2()
picam2.start()
time.sleep(2)  # Let camera warm up

i = 0
while True:
    frame = picam2.capture_array()
    cv2.imshow("Press 's' to save, 'q' to quit", frame)
    key = cv2.waitKey(1)
    if key == ord('s'):
        filename = f"calib_img_{i}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
        i += 1
    elif key == ord('q'):
        break

cv2.destroyAllWindows()