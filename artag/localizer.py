import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import os

# --- Load image ---
image_path = os.path.join("/Users/abhinavgupta/Abbi/img2.jpg")  # <-- updated to use the Abbi folder
if not os.path.isfile(image_path):
    raise FileNotFoundError(f"Image file not found: {image_path}")
img = cv.imread(image_path)
if img is None:
    raise ValueError(f"Failed to load image at {image_path}")
img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

# --- Default camera calibration parameters (for testing only) ---
focal_length = 800  # pixels (arbitrary, for demo)
cx = img.shape[1] / 2
cy = img.shape[0] / 2
camera_matrix = np.array([
    [focal_length, 0, cx],
    [0, focal_length, cy],
    [0, 0, 1]
], dtype=np.float32)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # No distortion
marker_length = 0.2667  # 10.5 inches in meters

def detect_and_draw_with_pose(img, aruco_name, aruco_dict_id):
    aruco_dict = cv.aruco.getPredefinedDictionary(aruco_dict_id)
    corners, ids, _ = cv.aruco.detectMarkers(img, aruco_dict)
    output = img.copy()

    if ids is not None and len(ids) > 0:
        cv.aruco.drawDetectedMarkers(output, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            cv.drawFrameAxes(output, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            print(f"[{aruco_name}] Marker {ids[i][0]}")
            print("  ➤ Position (tvec):", np.round(tvec.ravel(), 3))
            print("  ➤ Rotation (rvec):", np.round(rvec.ravel(), 3))

            R, _ = cv.Rodrigues(rvec)
            cam_in_marker = -R.T @ tvec.reshape(3, 1)
            print("  ➤ Camera in marker frame:", np.round(cam_in_marker.ravel(), 3))
            print()
    else:
        print(f"[{aruco_name}] No markers detected")
    return output

# --- Test multiple ArUco dictionaries ---
aruco_dicts = {
    "DICT_5X5_100": cv.aruco.DICT_5X5_100,
}

plt.figure(figsize=(15, 12))

for i, (name, dict_id) in enumerate(aruco_dicts.items(), 1):
    result_img = detect_and_draw_with_pose(img_rgb, name, dict_id)
    plt.subplot(2, 2, i)
    plt.imshow(result_img)
    plt.title(name)
    plt.axis('off')

plt.tight_layout()
plt.show()