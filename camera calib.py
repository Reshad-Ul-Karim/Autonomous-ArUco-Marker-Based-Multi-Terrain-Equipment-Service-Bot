import cv2
import numpy as np


def main():
    # INPUT SOURCE:
    #  - Use 0 for webcam
    #  - Or set video_path = "your_video.mp4"
    video_path = None  # e.g. "calib_video.mp4"
    cap = cv2.VideoCapture(0 if video_path is None else video_path)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera/video.")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    # Marker size in meters (4 inches = 0.1016 meters)
    marker_length_m = 0.1016

    # Object points for a single marker (4 corners in marker coordinate system)
    # Marker coordinate system: center at origin, Z=0, corners at ±marker_length/2
    obj_points_single = np.array([
        [-marker_length_m / 2, marker_length_m / 2, 0],
        [marker_length_m / 2, marker_length_m / 2, 0],
        [marker_length_m / 2, -marker_length_m / 2, 0],
        [-marker_length_m / 2, -marker_length_m / 2, 0]
    ], dtype=np.float32)

    all_obj_points = []  # 3D object points
    all_img_points = []  # 2D image points
    image_size = None

    print("Calibration mode (2 ArUco markers):")
    print(" - Show TWO ArUco markers to the camera")
    print(" - Press 'c' to capture a good frame (try 20–40 frames)")
    print(" - Press 'k' to calibrate")
    print(" - Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0])

        corners, ids, _ = detector.detectMarkers(frame)
        display = frame.copy()

        # Filter to only show and use exactly 2 markers
        if ids is not None and len(ids) == 2:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            cv2.putText(display, f"2 markers detected (IDs: {ids.flatten()})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        elif ids is not None:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            cv2.putText(display, f"Detected {len(ids)} markers - need exactly 2",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(display, "No markers detected - show 2 markers",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("ArUco Calibration - 2 Markers (c=save, k=calibrate, q=quit)", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            if ids is None or len(ids) != 2:
                print(f"Need exactly 2 markers detected. Currently: {len(ids) if ids is not None else 0} markers.")
                continue

            # Build object points and image points for both markers
            obj_points_frame = []
            img_points_frame = []

            for corner in corners:
                # Add the 4 corners of this marker
                obj_points_frame.append(obj_points_single)
                # Reshape corner to (4, 2)
                img_points_frame.append(corner[0].reshape(-1, 2).astype(np.float32))

            # Combine into single arrays
            obj_points_combined = np.vstack(obj_points_frame)
            img_points_combined = np.vstack(img_points_frame)

            all_obj_points.append(obj_points_combined)
            all_img_points.append(img_points_combined)
            print(f"Captured frame #{len(all_obj_points)} (2 markers, 8 corners total)")

        elif key == ord('k'):
            if len(all_obj_points) < 15:
                print("Capture at least ~15 frames first (20–40 recommended).")
                continue

            # Calibrate using standard calibrateCamera
            print("Calibrating camera...")
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objectPoints=all_obj_points,
                imagePoints=all_img_points,
                imageSize=image_size,
                cameraMatrix=None,
                distCoeffs=None
            )

            print("Calibration RMS error:", ret)
            print("Camera matrix:\n", camera_matrix)
            print("Dist coeffs:\n", dist_coeffs.ravel())

            # Save to file
            fs = cv2.FileStorage("calib_iPhone15.yml", cv2.FILE_STORAGE_WRITE)
            fs.write("camera_matrix", camera_matrix)
            fs.write("dist_coeffs", dist_coeffs)
            fs.release()
            print("Saved calibration to calib_iPhone15.yml")
            break

        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
