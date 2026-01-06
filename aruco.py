import cv2
import numpy as np


def load_calibration(path="calib_iPhone15.yml"):
    import os
    if not os.path.exists(path):
        print(f"Warning: Calibration file '{path}' not found. Using default values.")
        print("For accurate pose estimation, please run calibrate_camera.py first.")
        # Default camera matrix (approximate for common cameras)
        # You should replace these with your actual calibration values
        h, w = 480, 640  # Default resolution, will be updated from actual frame
        fx = fy = w * 0.7  # Approximate focal length
        cx, cy = w / 2, h / 2
        camera_matrix = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        return camera_matrix, dist_coeffs, True  # Return flag indicating default values

    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Could not open calibration file: {path}")
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("dist_coeffs").mat()
    fs.release()
    return camera_matrix, dist_coeffs, False


def main():
    camera_matrix, dist_coeffs, using_defaults = load_calibration("calib_iPhone15.yml")

    # Your marker is 4 inches per side => 0.1016 meters
    marker_length_m = 0.1016

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    # Webcam or video
    video_path = None  # e.g. "marker_video.mp4"
    cap = cv2.VideoCapture(0 if video_path is None else video_path)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera/video.")

    print("Press 'q' to quit.")
    frame_initialized = False
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Update camera matrix with actual frame size if using defaults (first frame only)
        if using_defaults and not frame_initialized:
            h, w = frame.shape[:2]
            fx = fy = w * 0.7
            cx, cy = w / 2, h / 2
            camera_matrix = np.array([[fx, 0, cx],
                                      [0, fy, cy],
                                      [0, 0, 1]], dtype=np.float32)
            frame_initialized = True

        corners, ids, _ = detector.detectMarkers(frame)

        # Define object points for a single marker (4 corners in marker coordinate system)
        obj_points = np.array([
            [-marker_length_m / 2, marker_length_m / 2, 0],
            [marker_length_m / 2, marker_length_m / 2, 0],
            [marker_length_m / 2, -marker_length_m / 2, 0],
            [-marker_length_m / 2, -marker_length_m / 2, 0]
        ], dtype=np.float32)

        if ids is not None:
            print(f"ArUco marker(s) detected! IDs: {ids.flatten()}")
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Store positions, angles, and image centers for each marker
            marker_positions = {}
            marker_angles = {}     # roll, pitch, yaw in degrees
            marker_centers_2d = {}  # Store 2D image coordinates of marker centers

            # Process each detected marker (up to any number, typically 2–4)
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])
                # Reshape corner to (4, 2) for solvePnP
                img_points = corner[0].reshape(-1, 2).astype(np.float32)

                # Calculate center of marker in image coordinates (average of 4 corners)
                center_2d = np.mean(img_points, axis=0).astype(int)
                marker_centers_2d[marker_id] = center_2d

                # Solve PnP to get rotation and translation vectors
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, camera_matrix, dist_coeffs
                )

                if success:
                    # Get position (translation vector)
                    position = tvec.flatten()
                    marker_positions[marker_id] = position

                    # Convert rotation vector to roll, pitch, yaw (in degrees)
                    R, _ = cv2.Rodrigues(rvec)
                    # Assuming camera coordinate system:
                    # x-right, y-down, z-forward (OpenCV convention)
                    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
                    singular = sy < 1e-6
                    if not singular:
                        yaw = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
                        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
                        roll = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
                    else:
                        yaw = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
                        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
                        roll = 0.0
                    marker_angles[marker_id] = (roll, pitch, yaw)

            # If at least 2 markers detected with valid poses, calculate pairwise distances
            if len(marker_positions) >= 2:
                marker_ids_list = sorted(marker_positions.keys())

                # Display individual positions
                y_offset = 30
                line_height = 22
                for idx, marker_id in enumerate(marker_ids_list):
                    pos = marker_positions[marker_id]
                    roll, pitch, yaw = marker_angles.get(marker_id, (0.0, 0.0, 0.0))
                    pos_y = int(y_offset + idx * line_height)
                    cv2.putText(
                        frame,
                        f"Marker {marker_id}: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}",
                        (10, pos_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )
                    angles_y = int(y_offset + (idx + 0.6) * line_height)
                    cv2.putText(
                        frame,
                        f"  roll={roll:5.1f}  pitch={pitch:5.1f}  yaw={yaw:5.1f} deg",
                        (10, angles_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 200, 255),
                        1,
                    )

                # Pairwise distances and lines between markers (robust for up to 4 markers)
                pair_y_start = y_offset + len(marker_ids_list) * line_height + 10
                pair_idx = 0
                for i in range(len(marker_ids_list)):
                    for j in range(i + 1, len(marker_ids_list)):
                        id1 = marker_ids_list[i]
                        id2 = marker_ids_list[j]
                        pos1 = marker_positions[id1]
                        pos2 = marker_positions[id2]

                        # 3D Euclidean distance
                        distance = np.linalg.norm(pos1 - pos2)

                        # Draw line between marker centers in image
                        c1 = tuple(marker_centers_2d[id1])
                        c2 = tuple(marker_centers_2d[id2])
                        cv2.line(frame, c1, c2, (255, 0, 255), 2)

                        # Midpoint for distance text
                        midpoint = ((c1[0] + c2[0]) // 2, (c1[1] + c2[1]) // 2)
                        distance_text = f"{distance * 100:.1f} cm"
                        text_size = cv2.getTextSize(distance_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        text_x = midpoint[0] - text_size[0] // 2
                        text_y = midpoint[1] - 5

                        # Background rectangle and text on the line
                        cv2.rectangle(
                            frame,
                            (text_x - 3, text_y - text_size[1] - 3),
                            (text_x + text_size[0] + 3, text_y + 3),
                            (0, 0, 0),
                            -1,
                        )
                        cv2.putText(
                            frame,
                            distance_text,
                            (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 0, 255),
                            1,
                        )

                        # Text summary in top-left
                        pair_label = f"{id1}-{id2}: {distance:.3f} m ({distance * 100:.1f} cm)"
                        cv2.putText(
                            frame,
                            pair_label,
                            (10, pair_y_start + pair_idx * line_height),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (255, 0, 255),
                            2,
                        )
                        pair_idx += 1

                        # Print to terminal as well
                        print(
                            f"Markers {id1} & {id2}: "
                            f"pos1=({pos1[0]:.3f}, {pos1[1]:.3f}, {pos1[2]:.3f}) m, "
                            f"pos2=({pos2[0]:.3f}, {pos2[1]:.3f}, {pos2[2]:.3f}) m, "
                            f"distance={distance:.3f} m ({distance * 100:.1f} cm)"
                        )
                print("-" * 50)
            elif len(marker_positions) == 1:
                # Only one marker with valid pose – show its position
                marker_id, pos = next(iter(marker_positions.items()))
                roll, pitch, yaw = marker_angles.get(marker_id, (0.0, 0.0, 0.0))
                cv2.putText(
                    frame,
                    f"Marker {marker_id}: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    f"roll={roll:5.1f}  pitch={pitch:5.1f}  yaw={yaw:5.1f} deg",
                    (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 200, 255),
                    2,
                )

        cv2.imshow("ArUco Pose (iPhone 15)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
