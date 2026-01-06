import cv2
import numpy as np
import socket
import time
import logging
import os
from datetime import datetime

# Robot control settings
ESP_IP = "172.27.106.106"
PORT = 3333
TICK = 1 / 40.0  # Command send rate (Hz)

# Marker IDs
ROBOT_ID = 1
DESTINATION_ID = 11
TARGET_DISTANCE_M = 0.40  # 40 cm in meters - tolerance for completion

# Thresholds for alignment in robot coordinate frame (meters)
LATERAL_THRESHOLD = 0.05  # 5 cm - consider laterally aligned if within this
DISTANCE_THRESHOLD = 0.05  # 5 cm - consider at target distance if within this

# PID Controller parameters (for robot frame)
# Reduced gains for 500 RPM motor to prevent overshooting
PID_LATERAL_KP = 0.15  # Reduced from 0.5 - less aggressive for fast motor
PID_LATERAL_KI = 0.005  # Reduced from 0.01 - slower integral accumulation
PID_LATERAL_KD = 0.05  # Reduced from 0.1 - less derivative action

PID_DISTANCE_KP = 0.2  # Reduced from 0.3
PID_DISTANCE_KI = 0.005  # Reduced from 0.01
PID_DISTANCE_KD = 0.05  # Reduced from 0.1

# PID output limits
PID_OUTPUT_MAX = 1.0  # Maximum PID output (normalized)
PID_OUTPUT_MIN = -1.0  # Minimum PID output (normalized)

# Deadband and thresholds for fast motor
MIN_LATERAL_ERROR = 0.02  # 2 cm - don't correct if error is smaller (deadband)
MIN_DISTANCE_ERROR = 0.03  # 3 cm - don't correct if error is smaller (deadband)
PID_OUTPUT_THRESHOLD = 0.15  # Minimum PID output to trigger command (reduces jitter)

# Command rate limiting (prevent rapid command changes)
MIN_COMMAND_INTERVAL = 0.1  # Minimum time between command changes (seconds)

# Coordinate frame fix flag (if z_robot sign is inverted, set to True)
FLIP_Z_AXIS = False  # Set to True if robot moves backward when it should move forward

# Robot control socket
sock = None

# Setup logging
LOG_FORMAT = '%(asctime)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
LOG_FILE = f'robot_navigation_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'

logging.basicConfig(
    level=logging.INFO,  # Changed from DEBUG to INFO to reduce verbosity
    format=LOG_FORMAT,
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()  # Also print to console
    ]
)

logger = logging.getLogger(__name__)
logger.info("=" * 60)
logger.info("Robot Navigation System - Logging Started")
logger.info(f"Log file: {LOG_FILE}")
logger.info("=" * 60)


def connect():
    logger.debug(f"Attempting to connect to {ESP_IP}:{PORT}")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((ESP_IP, PORT))
    s.settimeout(None)
    logger.info(f"✅ Connected to {ESP_IP}:{PORT}")
    return s


def safe_connect_forever():
    global sock
    logger.info("Starting connection attempt...")
    while True:
        try:
            sock = connect()
            try:
                sock.sendall(b"S")
                logger.debug("Sent initial stop command")
            except Exception as e:
                logger.warning(f"Failed to send initial stop command: {e}")
            return
        except Exception as e:
            logger.warning(f"⚠️ Connect failed, retrying... {e}")
            time.sleep(1)


def send_command(cmd: str):
    """Send a command to the robot."""
    global sock
    if not sock:
        logger.warning("Socket not connected, reconnecting...")
        safe_connect_forever()
    try:
        sock.sendall(cmd.encode("utf-8"))
        # logger.debug(f"Sent command: '{cmd}'")  # Commented to reduce logging
    except Exception as e:
        logger.error(f"⚠️ Disconnected, reconnecting... {e}")
        try:
            sock.close()
        except Exception:
            pass
        sock = None
        safe_connect_forever()
        sock.sendall(cmd.encode("utf-8"))
        # logger.debug(f"Sent command after reconnect: '{cmd}'")  # Commented to reduce logging


class PIDController:
    """PID Controller for smooth control."""
    
    def __init__(self, kp, ki, kd, output_min=-1.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def update(self, error, dt=None):
        """
        Update PID controller with new error.
        
        Args:
            error: Current error (setpoint - current_value)
            dt: Time delta since last update (if None, uses internal timing)
        
        Returns:
            output: PID output value
        """
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.01  # Default small dt for first call
        else:
            if dt is None:
                dt = current_time - self.last_time
            if dt <= 0:
                dt = 0.01  # Prevent division by zero
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        # Clamp integral to prevent windup
        max_integral = (self.output_max - self.output_min) / (self.ki + 1e-6)
        self.integral = np.clip(self.integral, -max_integral, max_integral)
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = np.clip(output, self.output_min, self.output_max)
        
        # Update for next iteration
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID controller state."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None


def load_calibration(path="camera_calibration.yml"):
    """Load camera calibration from YAML file."""
    import yaml
    
    if not os.path.exists(path):
        print(f"Warning: Calibration file '{path}' not found. Using default values.")
        h, w = 480, 640
        fx = fy = w * 0.7
        cx, cy = w / 2, h / 2
        camera_matrix = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        return camera_matrix, dist_coeffs, True

    # Try loading as regular YAML first (new format from chessboard_calibration.py)
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        # Check if it's the new format (has 'camera_matrix' as list)
        if isinstance(data.get('camera_matrix'), list):
            camera_matrix = np.array(data['camera_matrix'], dtype=np.float32)
            dist_coeffs = np.array(data['distortion_coefficients'], dtype=np.float32).reshape(-1, 1)
            logger.info(f"Loaded calibration from {path} (new YAML format)")
            logger.info(f"  Image size: {data.get('image_width')}x{data.get('image_height')}")
            logger.info(f"  Reprojection error: {data.get('reprojection_error', 'N/A')}")
            return camera_matrix, dist_coeffs, False
    except Exception as e:
        logger.debug(f"Failed to load as new YAML format: {e}, trying OpenCV format...")
    
    # Fall back to OpenCV FileStorage format (old calib_iPhone15.yml format)
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise RuntimeError(f"Could not open calibration file: {path}")
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        fs.release()
        logger.info(f"Loaded calibration from {path} (OpenCV FileStorage format)")
        return camera_matrix, dist_coeffs, False
    except Exception as e:
        logger.error(f"Failed to load calibration from {path}: {e}")
        raise


def transform_to_robot_frame(robot_pos_3d, robot_rvec, dest_pos_3d):
    """
    Transform destination position from camera frame to robot's coordinate frame.
    
    For marker placed flat on top of robot:
    - Marker is on top of robot (horizontal plane)
    - Bottom of marker (ID-1) faces forward (robot's front)
    - Need to adjust coordinate frame to match robot's expected frame
    
    Robot coordinate convention:
    - X-axis: Right (positive) / Left (negative)
    - Y-axis: Up (positive) / Down (negative) - vertical
    - Z-axis: Forward (positive) / Backward (negative) - horizontal along ground
    
    Args:
        robot_pos_3d: Robot position in camera frame (3D vector)
        robot_rvec: Robot rotation vector in camera frame
        dest_pos_3d: Destination position in camera frame (3D vector)
    
    Returns:
        dest_in_robot_frame: Destination position in robot's coordinate frame [x, y, z]
        robot_R: Robot rotation matrix (for visualization)
    """
    # Convert rotation vector to rotation matrix
    marker_R, _ = cv2.Rodrigues(robot_rvec)
    
    # Calculate relative position in camera frame
    dest_relative_camera = dest_pos_3d - robot_pos_3d
    
    # Transform to marker's coordinate frame first
    marker_R_inv = marker_R.T
    dest_in_marker_frame = marker_R_inv @ dest_relative_camera
    
    # For ArUco marker placed flat on top of robot:
    # ArUco marker coordinate system (from solvePnP):
    # - X-axis: points right (along marker width)
    # - Y-axis: points down (along marker height, into marker plane)
    # - Z-axis: points out of marker plane (up, toward camera when looking from above)
    #
    # Robot coordinate convention (for ground robot):
    # - X-axis: Right (positive) / Left (negative)
    # - Y-axis: Up (positive) / Down (negative) - vertical
    # - Z-axis: Forward (positive) / Backward (negative) - horizontal along ground
    #
    # Since marker is on top and bottom of marker faces forward:
    # - Robot X = Marker X (both point right)
    # - Robot Y = Marker Z (marker Z points up, robot Y points up)
    # - Robot Z = -Marker Y (marker Y points down/forward, robot Z points forward)
    
    # Transform from marker frame to robot frame
    # CRITICAL FIX: User logs show z_robot=0.472 (positive) when destination is visually in front,
    # but robot moves backward. This means the Z-axis sign is inverted.
    # We need: z_robot < 0 when destination is in front (so robot moves forward)
    #         z_robot > 0 when destination is behind (so robot doesn't move forward)
    # For marker on top: marker Y points down -> robot Z should point forward (negative)
    # So we need to NEGATE marker Y to get correct robot Z sign
    dest_in_robot_frame = np.array([
        dest_in_marker_frame[0],   # X: right (same)
        dest_in_marker_frame[2],   # Y: up (marker Z points up)
        -dest_in_marker_frame[1]   # Z: forward (NEGATE marker Y so positive marker Y -> negative robot Z)
        # FIXED: Negate to ensure z_robot < 0 when destination is in front
    ])
    
    # logger.debug(f"Coordinate Transform: "  # Commented to reduce logging
    #              f"Camera frame: robot={robot_pos_3d}, dest={dest_pos_3d}, "
    #              f"relative={dest_relative_camera}, "
    #              f"Marker frame: {dest_in_marker_frame}, "
    #              f"Robot frame: {dest_in_robot_frame}")
    
    # Diagnostic: Check if z_robot sign matches expected behavior (commented to reduce logging)
    # z_robot_check = dest_in_robot_frame[2]
    # if z_robot_check > 0.1:
    #     logger.warning(f"⚠️ Z_AXIS_CHECK: z_robot={z_robot_check:.3f} is POSITIVE")
    # elif z_robot_check < -0.1:
    #     logger.debug(f"✓ Z_AXIS_CHECK: z_robot={z_robot_check:.3f} is NEGATIVE")
    
    # Create robot frame rotation matrix for reference
    # Extract marker axes in camera frame
    marker_x = marker_R[:, 0]  # Marker X (right)
    marker_y = marker_R[:, 1]  # Marker Y (down, into marker plane)
    marker_z = marker_R[:, 2]  # Marker Z (up, out of marker plane)
    
    # Robot axes in camera frame (for marker on top of robot)
    # Updated to match the coordinate transformation fix
    robot_x = marker_x          # Right (same as marker X)
    robot_y = marker_z         # Up (marker Z points up)
    robot_z = -marker_y        # Forward (NEGATE marker Y so positive marker Y -> negative robot Z)
    
    # Normalize and ensure orthonormal
    robot_y = robot_y / np.linalg.norm(robot_y)
    robot_z = robot_z / np.linalg.norm(robot_z)
    robot_x = np.cross(robot_y, robot_z)
    robot_x = robot_x / np.linalg.norm(robot_x)
    
    # Re-orthonormalize
    robot_z = np.cross(robot_x, robot_y)
    robot_z = robot_z / np.linalg.norm(robot_z)
    
    # Create robot frame rotation matrix
    robot_R = np.column_stack([robot_x, robot_y, robot_z])
    
    return dest_in_robot_frame, robot_R


def compute_robot_frame_errors(dest_in_robot_frame, target_distance):
    """
    Calculate errors in robot's coordinate frame.
    
    Args:
        dest_in_robot_frame: [x, y, z] position in robot's coordinate frame
        target_distance: Target distance in meters
    
    Returns:
        lateral_error: Error in x direction (positive = destination to robot's right)
        distance_error: Error in distance (positive = too far)
        distance: Current distance
        angle_deg: Angle to destination in horizontal plane
    """
    x_robot = dest_in_robot_frame[0]  # Lateral (right/left relative to robot)
    y_robot = dest_in_robot_frame[1]  # Vertical (up/down) - usually ignored for ground robots
    z_robot = dest_in_robot_frame[2]  # Forward/backward relative to robot
    
    # Current distance
    distance = np.linalg.norm(dest_in_robot_frame)
    
    # Lateral error (x_robot should be near 0 for alignment)
    lateral_error = x_robot
    
    # Distance error (distance should equal target_distance)
    distance_error = distance - target_distance
    
    # Angle to destination in horizontal plane (x-z plane)
    angle_rad = np.arctan2(x_robot, z_robot)  # Angle from forward direction
    angle_deg = np.degrees(angle_rad)
    
    # logger.debug(f"Error Calculation: "  # Commented to reduce logging
    #              f"Robot frame (x={x_robot:.3f}, y={y_robot:.3f}, z={z_robot:.3f}), "
    #              f"distance={distance:.3f}m, target={target_distance:.3f}m, "
    #              f"lateral_error={lateral_error:.3f}m, distance_error={distance_error:.3f}m, "
    #              f"angle={angle_deg:.1f}°")
    
    return lateral_error, distance_error, distance, angle_deg


def compute_navigation_command_robot_frame(lateral_error, distance_error, distance, z_robot,
                                          pid_lateral, pid_distance):
    """
    Simple, proven navigation approach: Proportional control toward goal.
    
    Strategy:
    1. Calculate angle to goal in horizontal plane
    2. Use proportional control: steer toward goal, move forward if goal is ahead
    3. Stop when close enough
    
    This is based on the "pure pursuit" / "go-to-goal" navigation method.
    
    Args:
        lateral_error: Error in x direction (positive = destination to right)
        distance_error: Distance error (positive = too far, negative = too close)
        distance: Current distance to destination
        z_robot: Z component in robot frame (positive = forward, negative = behind)
        pid_lateral: PID controller (not used in this simple approach)
        pid_distance: PID controller (not used in this simple approach)
    
    Returns:
        cmd: Command string ('L', 'R', 'S', 'F', 'B', 'G', 'I', 'H', 'J')
        phase: Current phase description
        pid_outputs: Dictionary for debugging
    """
    lateral_error_abs = abs(lateral_error)
    distance_error_abs = abs(distance_error)
    
    # logger.debug(f"Navigation Decision Input: "  # Commented to reduce logging
    #              f"lateral_error={lateral_error:.3f}m ({lateral_error*100:.1f}cm), "
    #              f"distance_error={distance_error:.3f}m ({distance_error*100:.1f}cm), "
    #              f"distance={distance:.3f}m, z_robot={z_robot:.3f}m")
    
    # Check if goal reached
    if distance_error_abs <= DISTANCE_THRESHOLD and lateral_error_abs <= LATERAL_THRESHOLD:
        logger.info(f"✅ GOAL REACHED: Distance={distance:.3f}m, Lateral={lateral_error:.3f}m")
        return "S", f"✅ AT GOAL: Distance={distance:.3f}m, Lateral={lateral_error:.3f}m", {
            'pid_lateral': 0.0, 'pid_distance': 0.0
        }
    
    # Simple proportional control thresholds
    ANGLE_THRESHOLD = 0.15  # 15cm lateral error threshold for steering
    DISTANCE_MOVE_THRESHOLD = 0.10  # 10cm - only move if error > this
    
    # Determine if we need to steer (based on lateral error)
    need_steer_left = lateral_error < -ANGLE_THRESHOLD
    need_steer_right = lateral_error > ANGLE_THRESHOLD
    
    logger.debug(f"Steering Decision: "
                 f"need_steer_left={need_steer_left}, need_steer_right={need_steer_right}, "
                 f"ANGLE_THRESHOLD={ANGLE_THRESHOLD*100:.1f}cm")
    
    # Simple logic: Move forward if too far, stop if too close
    # Use z_robot sign: positive = destination in front, negative = behind
    # But if z_robot sign seems inverted (based on debug), we'll handle it
    
    need_forward = False
    need_backward = False
    
    # Only move if distance error is significant
    # CRITICAL FIX: Based on user logs, positive z_robot (e.g., 0.472) causes robot to move BACKWARD
    # This means: positive z_robot = destination BEHIND robot's forward direction
    #            negative z_robot = destination IN FRONT of robot's forward direction
    # So we need to use: z_robot < 0 for "destination in front" (move forward)
    #                    z_robot > 0 for "destination behind" (steer first, don't move forward)
    if distance_error > DISTANCE_MOVE_THRESHOLD:  # Too far - need to get closer
        logger.debug(f"Distance check: Too far (error={distance_error*100:.1f}cm > threshold={DISTANCE_MOVE_THRESHOLD*100:.1f}cm)")
        # Check if destination is in front using z_robot
        # FIXED: z_robot < 0 means destination in front (robot should move forward)
        #        z_robot > 0 means destination behind (robot should steer first, not move forward)
        if z_robot < -0.05:  # Destination in front - move forward
            need_forward = True
            logger.debug(f"z_robot={z_robot:.3f} < -0.05: Destination in front, will move forward")
        elif z_robot > 0.05:  # Destination behind - steer first, don't move forward
            # Don't move backward - just steer toward it first
            need_forward = False
            logger.debug(f"z_robot={z_robot:.3f} > 0.05: Destination behind, will steer first (no forward movement)")
        else:  # z_robot near zero - destination is to the side
            # Move forward anyway if we're far (we'll steer at the same time)
            if distance > TARGET_DISTANCE_M * 1.5:
                need_forward = True
                logger.debug(f"z_robot={z_robot:.3f} near zero, but far away: Will move forward with steering")
            else:
                logger.debug(f"z_robot={z_robot:.3f} near zero and close: Will not move forward")
    else:
        logger.debug(f"Distance check: Close enough (error={distance_error*100:.1f}cm <= threshold={DISTANCE_MOVE_THRESHOLD*100:.1f}cm)")
    
    # When close to target, be more conservative
    if distance < TARGET_DISTANCE_M * 1.2:  # Within 36cm (when target is 30cm)
        logger.debug(f"Close to target (distance={distance:.3f}m < {TARGET_DISTANCE_M*1.2:.3f}m): Being conservative")
        # Only move forward if still significantly too far AND destination is in front
        if distance_error > 0.08:  # More than 8cm too far
            need_forward = z_robot < -0.03  # And destination is in front (z_robot < 0)
            logger.debug(f"Close but still too far (error={distance_error*100:.1f}cm > 8cm): need_forward={need_forward} (z_robot={z_robot:.3f})")
        else:
            need_forward = False  # Stop forward movement when close
            logger.debug(f"Very close to target (error={distance_error*100:.1f}cm <= 8cm): Stopping forward movement")
    
    # Generate command: prioritize steering when far off, combine when needed
    if need_forward and need_steer_left:
        cmd = "G"  # Forward-left
        phase = f"Approaching: Forward-Left (dist={distance:.3f}m, lat={lateral_error*100:.1f}cm)"
        logger.info(f"Command: {cmd} - {phase}")
    elif need_forward and need_steer_right:
        cmd = "I"  # Forward-right
        phase = f"Approaching: Forward-Right (dist={distance:.3f}m, lat={lateral_error*100:.1f}cm)"
        logger.info(f"Command: {cmd} - {phase}")
    elif need_forward:
        cmd = "F"  # Forward only
        phase = f"Approaching: Forward (dist={distance:.3f}m, z={z_robot:.3f}m)"
        logger.info(f"Command: {cmd} - {phase}")
    elif need_steer_left:
        cmd = "L"  # Left only
        phase = f"Aligning: Steer Left (lat={lateral_error*100:.1f}cm)"
        logger.info(f"Command: {cmd} - {phase}")
    elif need_steer_right:
        cmd = "R"  # Right only
        phase = f"Aligning: Steer Right (lat={lateral_error*100:.1f}cm)"
        logger.info(f"Command: {cmd} - {phase}")
    else:
        cmd = "S"  # Stop
        phase = f"Stopped: Close to goal (dist={distance:.3f}m, err={distance_error*100:.1f}cm)"
        logger.debug(f"Command: {cmd} - {phase}")
    
    # Update PID for monitoring
    if lateral_error_abs > MIN_LATERAL_ERROR:
        pid_output_lateral = pid_lateral.update(-lateral_error)
    else:
        pid_output_lateral = 0.0
        pid_lateral.reset()
    
    if distance_error_abs > MIN_DISTANCE_ERROR:
        pid_output_distance = pid_distance.update(distance_error)
    else:
        pid_output_distance = 0.0
        pid_distance.reset()
    
    return cmd, phase, {
        'pid_lateral': pid_output_lateral, 'pid_distance': pid_output_distance
    }


def main():
    logger.info("=" * 60)
    logger.info("Starting Robot Navigation System")
    logger.info(f"Robot ID: {ROBOT_ID}, Destination ID: {DESTINATION_ID}")
    logger.info(f"Target distance: {TARGET_DISTANCE_M * 100:.1f} cm")
    logger.info(f"Lateral threshold: {LATERAL_THRESHOLD * 100:.1f} cm")
    logger.info(f"Distance threshold: {DISTANCE_THRESHOLD * 100:.1f} cm")
    logger.info("=" * 60)
    
    # Connect to robot
    safe_connect_forever()
    
    # Load camera calibration
    logger.info("Loading camera calibration...")
    camera_matrix, dist_coeffs, using_defaults = load_calibration("camera_calibration.yml")
    if using_defaults:
        logger.warning("Using default calibration values - camera may not be calibrated!")
    else:
        logger.info("Camera calibration loaded successfully")
    
    # Marker size in meters (4 inches = 0.1016 meters)
    marker_length_m = 0.1016
    logger.info(f"Marker size: {marker_length_m * 100:.2f} cm")
    
    # Initialize ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    logger.info("ArUco detector initialized (DICT_4X4_50)")
    
    # Open camera
    logger.info("Opening camera...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logger.error("Could not open camera!")
        raise RuntimeError("Could not open camera.")
    logger.info("Camera opened successfully")
    
    print("=" * 60)
    print("Robot Navigation System (Robot Frame PID-controlled)")
    print(f"Robot ID: {ROBOT_ID}, Destination ID: {DESTINATION_ID}")
    print(f"Strategy: Transform to robot frame, then PID control")
    print(f"Target distance: {TARGET_DISTANCE_M * 100:.1f} cm")
    print(f"Lateral threshold: {LATERAL_THRESHOLD * 100:.1f} cm")
    print(f"Distance threshold: {DISTANCE_THRESHOLD * 100:.1f} cm")
    print(f"Robot coordinate frame: X=right, Y=up, Z=forward")
    print(f"Log file: {LOG_FILE}")
    print("Press 'q' to quit, 's' to stop robot")
    print("=" * 60)
    
    # Initialize PID controllers for robot frame
    pid_lateral = PIDController(PID_LATERAL_KP, PID_LATERAL_KI, PID_LATERAL_KD,
                                PID_OUTPUT_MIN, PID_OUTPUT_MAX)
    pid_distance = PIDController(PID_DISTANCE_KP, PID_DISTANCE_KI, PID_DISTANCE_KD,
                                 PID_OUTPUT_MIN, PID_OUTPUT_MAX)
    
    last_cmd = None
    last_cmd_time = 0
    last_cmd_change_time = 0  # Track when command last changed (for rate limiting)
    
    # Object points for marker corners (needed for 3D distance calculation)
    obj_points = np.array([
        [-marker_length_m / 2, marker_length_m / 2, 0],
        [marker_length_m / 2, marker_length_m / 2, 0],
        [marker_length_m / 2, -marker_length_m / 2, 0],
        [-marker_length_m / 2, -marker_length_m / 2, 0]
    ], dtype=np.float32)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect markers
        corners, ids, _ = detector.detectMarkers(frame)
        
        robot_center_2d = None
        dest_center_2d = None
        robot_pos_3d = None
        dest_pos_3d = None
        robot_rvec = None
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            detected_ids = [int(id[0]) for id in ids]
            # logger.debug(f"Detected markers: {detected_ids}")  # Commented to reduce logging
            
            # Process each detected marker - get 2D centers and 3D positions
            for i, corner in enumerate(corners):
                marker_id = int(ids[i][0])
                img_points = corner[0].reshape(-1, 2).astype(np.float32)
                
                # Calculate center in image coordinates
                center_2d = tuple(np.mean(img_points, axis=0).astype(int))
                
                # Calculate 3D position for distance measurement
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, camera_matrix, dist_coeffs
                )
                
                if success:
                    position_3d = tvec.flatten()
                    
                    # Store robot and destination centers and 3D positions
                    if marker_id == ROBOT_ID:
                        robot_center_2d = center_2d
                        robot_pos_3d = position_3d
                        robot_rvec = rvec  # Store rotation vector for robot frame
                        logger.debug(f"Robot marker (ID {ROBOT_ID}) detected: "
                                   f"2D center={center_2d}, 3D pos={position_3d}")
                    elif marker_id == DESTINATION_ID:
                        dest_center_2d = center_2d
                        dest_pos_3d = position_3d
                        logger.debug(f"Destination marker (ID {DESTINATION_ID}) detected: "
                                   f"2D center={center_2d}, 3D pos={position_3d}")
                else:
                    logger.warning(f"Failed to solvePnP for marker ID {marker_id}")
        else:
            pass  # logger.debug("No markers detected in frame")  # Commented to reduce logging
        
        # Navigation logic - Robot frame approach
        status_text = ""
        cmd = "S"
        phase = None
        
        if robot_center_2d is not None and dest_center_2d is not None:
            # Check if we have both 3D positions and robot rotation
            if robot_pos_3d is not None and dest_pos_3d is not None and robot_rvec is not None:
                logger.debug("Both markers detected with 3D pose - computing navigation")
                
                # Transform destination to robot's coordinate frame
                dest_in_robot_frame, robot_R = transform_to_robot_frame(
                    robot_pos_3d, robot_rvec, dest_pos_3d
                )
                
                # Calculate errors in robot frame
                lateral_error, distance_error, distance, angle_deg = compute_robot_frame_errors(
                    dest_in_robot_frame, TARGET_DISTANCE_M
                )
                
                # Extract z_robot for forward/backward determination
                z_robot = dest_in_robot_frame[2]
                
                distance_cm = distance * 100
                
                # Check if within tolerance
                lateral_ok = abs(lateral_error) <= LATERAL_THRESHOLD
                distance_ok = abs(distance_error) <= DISTANCE_THRESHOLD
                
                if lateral_ok and distance_ok:
                    cmd = "S"
                    phase = f"✅ TARGET REACHED! Distance: {distance_cm:.1f} cm"
                    status_text = phase
                    logger.info(f"✅ TARGET REACHED! Distance: {distance_cm:.1f} cm, Lateral: {lateral_error*100:.1f} cm")
                    # Reset PID controllers when target reached
                    pid_lateral.reset()
                    pid_distance.reset()
                else:
                    # Draw line between robot and destination
                    cv2.line(frame, robot_center_2d, dest_center_2d, (0, 255, 255), 3)  # Yellow line
                    
                    # Use PID controllers to compute navigation command (direct path strategy)
                    cmd, phase, pid_outputs = compute_navigation_command_robot_frame(
                        lateral_error, distance_error, distance, z_robot,
                        pid_lateral, pid_distance
                    )
                    
                    # Display information with PID outputs
                    x_robot, y_robot, z_robot = dest_in_robot_frame
                    # FIXED: z_robot < 0 means destination in front, z_robot > 0 means behind
                    z_direction = "FRONT" if z_robot < 0 else "BEHIND"
                    status_text = (f"{phase} | "
                                 f"Robot Frame: x={x_robot:.3f}m y={y_robot:.3f}m z={z_robot:.3f}m ({z_direction}) | "
                                 f"Distance: {distance_cm:.1f} cm (err: {distance_error*100:.1f} cm) | "
                                 f"Lateral err: {lateral_error*100:.1f} cm | "
                                 f"Angle: {angle_deg:.1f}° | "
                                 f"PID: lat={pid_outputs['pid_lateral']:.2f} dist={pid_outputs['pid_distance']:.2f} | "
                                 f"Cmd: {cmd}")
                    
                    # Debug output with more details
                    x_robot, y_robot, z_robot_debug = dest_in_robot_frame
                    print(f"DEBUG: RobotFrame(x={x_robot:.3f}, y={y_robot:.3f}, z={z_robot:.3f}) "
                          f"dist={distance:.3f}m, dist_err={distance_error*100:.1f}cm, "
                          f"lat_err={lateral_error*100:.1f}cm, cmd={cmd}, phase={phase[:30]}")
                    
                    # Draw destination position in robot frame (project back to image)
                    # This helps visualize the relative position
                    # We can draw vectors showing x and z components
                    
            elif robot_pos_3d is None or dest_pos_3d is None or robot_rvec is None:
                # Missing 3D data, fall back to 2D visualization
                cv2.line(frame, robot_center_2d, dest_center_2d, (0, 255, 255), 3)
                status_text = "⚠️ Waiting for 3D pose data..."
                cmd = "S"
                logger.warning(f"Missing 3D pose data: robot_pos_3d={robot_pos_3d is not None}, "
                            f"dest_pos_3d={dest_pos_3d is not None}, robot_rvec={robot_rvec is not None}")
                pid_lateral.reset()
                pid_distance.reset()
            
            # Send command with rate limiting (prevent rapid changes for fast motor)
            current_time = time.time()
            time_since_last_change = current_time - last_cmd_change_time
            
            # Only change command if enough time has passed (rate limiting)
            if cmd != last_cmd:
                if time_since_last_change >= MIN_COMMAND_INTERVAL:
                    logger.info(f"Command changed: '{last_cmd}' -> '{cmd}' (after {time_since_last_change:.3f}s)")
                    send_command(cmd)
                    last_cmd = cmd
                    last_cmd_change_time = current_time
                    last_cmd_time = current_time
                else:
                    # Keep previous command if rate limit not met
                    logger.debug(f"Command change rate limited: '{last_cmd}' -> '{cmd}' "
                               f"(only {time_since_last_change:.3f}s since last change, need {MIN_COMMAND_INTERVAL:.3f}s)")
                    # Still update last_cmd_time for keepalive
                    if (current_time - last_cmd_time) >= TICK:
                        send_command(last_cmd)  # Resend previous command
                        last_cmd_time = current_time
            else:
                # Same command, resend at fixed rate for keepalive
                if (current_time - last_cmd_time) >= TICK:
                    send_command(cmd)
                    last_cmd_time = current_time
            
        elif robot_center_2d is None:
            status_text = f"⚠️ Robot marker (ID {ROBOT_ID}) not detected"
            cmd = "S"
            logger.warning(f"Robot marker (ID {ROBOT_ID}) not detected")
            # Reset PID controllers when marker lost
            pid_lateral.reset()
            pid_distance.reset()
            if cmd != last_cmd:
                send_command(cmd)
                last_cmd = cmd
        
        elif dest_center_2d is None:
            status_text = f"⚠️ Destination marker (ID {DESTINATION_ID}) not detected"
            cmd = "S"
            logger.warning(f"Destination marker (ID {DESTINATION_ID}) not detected")
            # Reset PID controllers when marker lost
            pid_lateral.reset()
            pid_distance.reset()
            if cmd != last_cmd:
                send_command(cmd)
                last_cmd = cmd
        
        else:
            status_text = "⚠️ Waiting for markers..."
            cmd = "S"
            # Reset PID controllers when waiting
            pid_lateral.reset()
            pid_distance.reset()
            if cmd != last_cmd:
                send_command(cmd)
                last_cmd = cmd
        
        # Display status on frame
        cv2.putText(frame, status_text,
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Display robot frame information if both markers detected
        if robot_center_2d is not None and dest_center_2d is not None:
            y_offset = 60
            cv2.putText(frame, f"Robot (ID {ROBOT_ID}): "
                       f"({robot_center_2d[0]}, {robot_center_2d[1]}) px",
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Dest (ID {DESTINATION_ID}): "
                       f"({dest_center_2d[0]}, {dest_center_2d[1]}) px",
                       (10, y_offset + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            
            # Display robot frame information if available
            if robot_pos_3d is not None and dest_pos_3d is not None and robot_rvec is not None:
                dest_in_robot_frame, _ = transform_to_robot_frame(
                    robot_pos_3d, robot_rvec, dest_pos_3d
                )
                lateral_error, distance_error, distance, angle_deg = compute_robot_frame_errors(
                    dest_in_robot_frame, TARGET_DISTANCE_M
                )
                
                x_robot, y_robot, z_robot = dest_in_robot_frame
                distance_cm = distance * 100
                
                phase_color = (255, 255, 0) if "PHASE 1" in phase else (0, 255, 255) if "PHASE 2" in phase else (0, 255, 0)
                cv2.putText(frame, f"Phase: {phase if phase else 'N/A'}",
                           (10, y_offset + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, phase_color, 2)
                cv2.putText(frame, f"Robot Frame: x={x_robot:.3f}m y={y_robot:.3f}m z={z_robot:.3f}m",
                           (10, y_offset + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                cv2.putText(frame, f"Distance: {distance_cm:.1f} cm | Lateral err: {lateral_error*100:.1f} cm | Angle: {angle_deg:.1f}°",
                           (10, y_offset + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.imshow("Robot Navigation", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            logger.info("Quit key pressed - stopping navigation")
            send_command("S")
            break
        elif key == ord('s'):
            logger.info("Stop key pressed - sending stop command")
            send_command("S")
            print("Stopped robot")
    
    # Cleanup
    logger.info("Cleaning up resources...")
    cap.release()
    cv2.destroyAllWindows()
    if sock:
        try:
            sock.close()
            logger.info("Socket closed")
        except Exception as e:
            logger.warning(f"Error closing socket: {e}")
    logger.info("✅ Navigation stopped")
    print("✅ Navigation stopped")


if __name__ == "__main__":
    main()

