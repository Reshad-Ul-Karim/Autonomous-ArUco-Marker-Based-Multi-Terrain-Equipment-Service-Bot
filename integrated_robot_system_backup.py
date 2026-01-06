import cv2
import numpy as np
import socket
import time
import logging
import os
from datetime import datetime
from pynput import keyboard
import threading

# ============================================================================
# CONFIGURATION
# ============================================================================

# Navigation ESP32 settings
NAV_ESP_IP = "172.27.106.106"
NAV_PORT = 3333
TICK = 1 / 40.0  # Command send rate (Hz)

# Arm control ESP32 settings (WiFi TCP)
ARM_ESP_IP = "172.27.106.192"
ARM_PORT = 3334
ARM_SEND_HZ = 30
STEP_DEG = 2
STEP_GRIP = 5
BASE_LIM = (0, 180)
JOINT_LIM = (20, 160)
GRIP_LIM = (30, 100)
HOME_BASE, HOME_JOINT, HOME_GRIP = 110, 160, 40

# Marker IDs
ROBOT_ID = 1
# DESTINATION_ID will be set dynamically

# Navigation thresholds
TARGET_DISTANCE_M = 0.30  # 30 cm in meters
LATERAL_THRESHOLD = 0.05  # 5 cm
DISTANCE_THRESHOLD = 0.05  # 5 cm

# PID Controller parameters
PID_LATERAL_KP = 0.15
PID_LATERAL_KI = 0.005
PID_LATERAL_KD = 0.05
PID_DISTANCE_KP = 0.2
PID_DISTANCE_KI = 0.005
PID_DISTANCE_KD = 0.05
PID_OUTPUT_MAX = 1.0
PID_OUTPUT_MIN = -1.0
MIN_LATERAL_ERROR = 0.02
MIN_DISTANCE_ERROR = 0.03
PID_OUTPUT_THRESHOLD = 0.15
MIN_COMMAND_INTERVAL = 0.1

FLIP_Z_AXIS = False

# ============================================================================
# LOGGING SETUP
# ============================================================================

LOG_FORMAT = '%(asctime)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
LOG_FILE = f'integrated_robot_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'

logging.basicConfig(
    level=logging.INFO,
    format=LOG_FORMAT,
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)
logger.info("=" * 60)
logger.info("Integrated Robot System - Logging Started")
logger.info(f"Log file: {LOG_FILE}")
logger.info("=" * 60)

# ============================================================================
# GLOBAL STATE
# ============================================================================

class SystemState:
    WAITING_TO_START = "WAITING_TO_START"
    MANUAL_NAVIGATION = "MANUAL_NAVIGATION"
    NAVIGATING = "NAVIGATING"
    NAVIGATION_COMPLETE = "NAVIGATION_COMPLETE"
    ARM_CONTROL = "ARM_CONTROL"
    WAITING_FOR_NEXT = "WAITING_FOR_NEXT"

current_state = SystemState.WAITING_TO_START
destination_id = None
nav_sock = None
arm_sock = None  # Changed from arm_serial to arm_sock

# Arm control state
base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
held = set()
held_special = set()
arm_control_running = False
manual_nav_running = False

# ============================================================================
# NAVIGATION ESP32 FUNCTIONS
# ============================================================================

def connect_nav():
    logger.debug(f"Attempting to connect to navigation ESP at {NAV_ESP_IP}:{NAV_PORT}")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((NAV_ESP_IP, NAV_PORT))
    s.settimeout(None)
    logger.info(f"✅ Connected to navigation ESP at {NAV_ESP_IP}:{NAV_PORT}")
    return s


def safe_connect_nav_forever():
    global nav_sock
    logger.info("Starting navigation ESP connection attempt...")
    while True:
        try:
            nav_sock = connect_nav()
            try:
                nav_sock.sendall(b"S")
                logger.debug("Sent initial stop command to navigation ESP")
            except Exception as e:
                logger.warning(f"Failed to send initial stop command: {e}")
            return
        except Exception as e:
            logger.warning(f"⚠️ Navigation ESP connect failed, retrying... {e}")
            time.sleep(1)


def send_nav_command(cmd: str):
    """Send a command to the navigation robot."""
    global nav_sock
    if not nav_sock:
        logger.warning("Navigation socket not connected, reconnecting...")
        safe_connect_nav_forever()
    try:
        nav_sock.sendall(cmd.encode("utf-8"))
    except Exception as e:
        logger.error(f"⚠️ Navigation ESP disconnected, reconnecting... {e}")
        try:
            nav_sock.close()
        except Exception:
            pass
        nav_sock = None
        safe_connect_nav_forever()
        nav_sock.sendall(cmd.encode("utf-8"))


# ============================================================================
# ARM CONTROL ESP32 FUNCTIONS (WiFi TCP)
# ============================================================================

def connect_arm():
    """Connect to arm ESP32 via WiFi TCP."""
    global arm_sock
    logger.debug(f"Attempting to connect to arm ESP at {ARM_ESP_IP}:{ARM_PORT}")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((ARM_ESP_IP, ARM_PORT))
    s.settimeout(None)
    
    # Read the "READY" message from ESP32
    try:
        ready_msg = s.recv(128).decode('utf-8').strip()
        logger.debug(f"Arm ESP32 response: {ready_msg}")
    except Exception as e:
        logger.warning(f"Could not read READY message: {e}")
    
    logger.info(f"✅ Connected to arm ESP32 at {ARM_ESP_IP}:{ARM_PORT}")
    arm_sock = s
    return f"{ARM_ESP_IP}:{ARM_PORT}"


def disconnect_arm():
    """Disconnect from arm ESP32."""
    global arm_sock
    if arm_sock:
        try:
            arm_sock.close()
            logger.info("Arm socket closed")
        except Exception as e:
            logger.warning(f"Error closing arm socket: {e}")
        arm_sock = None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def send_arm_pose():
    """Send arm pose to ESP32 via WiFi TCP."""
    global base, joint, grip, arm_sock
    base = clamp(base, *BASE_LIM)
    joint = clamp(joint, *JOINT_LIM)
    grip = clamp(grip, *GRIP_LIM)
    
    if arm_sock:
        try:
            # Send in format: "base,joint,grip\n"
            cmd = f"{base},{joint},{grip}\n"
            arm_sock.sendall(cmd.encode('utf-8'))
            
            # Optionally read response ("OK" or "ERR")
            # response = arm_sock.recv(32).decode('utf-8').strip()
            # logger.debug(f"Arm response: {response}")
        except Exception as e:
            logger.error(f"Failed to send arm pose: {e}")


def send_arm_home():
    """Send HOME command to arm ESP32."""
    global arm_sock
    if arm_sock:
        try:
            arm_sock.sendall(b"HOME\n")
            logger.info("Sent HOME command to arm")
        except Exception as e:
            logger.error(f"Failed to send HOME command: {e}")


# ============================================================================
# KEYBOARD HANDLERS FOR ARM AND MANUAL NAV CONTROL
# ============================================================================

def on_press(key):
    global arm_control_running, manual_nav_running, base, joint, grip, current_state

    if key == keyboard.Key.esc:
        if current_state == SystemState.ARM_CONTROL:
            arm_control_running = False
            logger.info("ESC pressed - exiting arm control mode")
            return False
        elif current_state == SystemState.MANUAL_NAVIGATION:
            manual_nav_running = False
            logger.info("ESC pressed - exiting manual navigation mode")
            return False

    # Arrow keys
    if key in (keyboard.Key.left, keyboard.Key.right, keyboard.Key.up, keyboard.Key.down):
        held_special.add(key)
        return

    try:
        k = key.char.lower()
        held.add(k)

        if k == 'r':
            # Send HOME command to ESP32
            send_arm_home()
            base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
            print(f"[HOME] base={base} joint={joint} grip={grip}")

        if k == 'p':
            print(f"[POSE] base={base} joint={joint} grip={grip}")

    except Exception:
        pass


def on_release(key):
    if key in held_special:
        held_special.discard(key)
        return
    try:
        held.discard(key.char.lower())
    except Exception:
        pass


# ============================================================================
# NAVIGATION PID CONTROLLER
# ============================================================================

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
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.01
        else:
            if dt is None:
                dt = current_time - self.last_time
            if dt <= 0:
                dt = 0.01
        
        p_term = self.kp * error
        
        self.integral += error * dt
        max_integral = (self.output_max - self.output_min) / (self.ki + 1e-6)
        self.integral = np.clip(self.integral, -max_integral, max_integral)
        i_term = self.ki * self.integral
        
        d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_min, self.output_max)
        
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None


# ============================================================================
# CAMERA CALIBRATION
# ============================================================================

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

    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        
        if isinstance(data.get('camera_matrix'), list):
            camera_matrix = np.array(data['camera_matrix'], dtype=np.float32)
            dist_coeffs = np.array(data['distortion_coefficients'], dtype=np.float32).reshape(-1, 1)
            logger.info(f"Loaded calibration from {path} (new YAML format)")
            return camera_matrix, dist_coeffs, False
    except Exception as e:
        logger.debug(f"Failed to load as new YAML format: {e}, trying OpenCV format...")
    
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


# ============================================================================
# NAVIGATION FUNCTIONS
# ============================================================================

def transform_to_robot_frame(robot_pos_3d, robot_rvec, dest_pos_3d):
    """Transform destination position from camera frame to robot's coordinate frame."""
    marker_R, _ = cv2.Rodrigues(robot_rvec)
    dest_relative_camera = dest_pos_3d - robot_pos_3d
    marker_R_inv = marker_R.T
    dest_in_marker_frame = marker_R_inv @ dest_relative_camera
    
    dest_in_robot_frame = np.array([
        dest_in_marker_frame[0],
        dest_in_marker_frame[2],
        -dest_in_marker_frame[1]
    ])
    
    marker_x = marker_R[:, 0]
    marker_y = marker_R[:, 1]
    marker_z = marker_R[:, 2]
    
    robot_x = marker_x
    robot_y = marker_z
    robot_z = -marker_y
    
    robot_y = robot_y / np.linalg.norm(robot_y)
    robot_z = robot_z / np.linalg.norm(robot_z)
    robot_x = np.cross(robot_y, robot_z)
    robot_x = robot_x / np.linalg.norm(robot_x)
    robot_z = np.cross(robot_x, robot_y)
    robot_z = robot_z / np.linalg.norm(robot_z)
    
    robot_R = np.column_stack([robot_x, robot_y, robot_z])
    
    return dest_in_robot_frame, robot_R


def compute_robot_frame_errors(dest_in_robot_frame, target_distance):
    """Calculate errors in robot's coordinate frame."""
    x_robot = dest_in_robot_frame[0]
    y_robot = dest_in_robot_frame[1]
    z_robot = dest_in_robot_frame[2]
    
    distance = np.linalg.norm(dest_in_robot_frame)
    lateral_error = x_robot
    distance_error = distance - target_distance
    
    angle_rad = np.arctan2(x_robot, z_robot)
    angle_deg = np.degrees(angle_rad)
    
    return lateral_error, distance_error, distance, angle_deg


def compute_navigation_command_robot_frame(lateral_error, distance_error, distance, z_robot,
                                          pid_lateral, pid_distance):
    """Simple proportional control toward goal."""
    lateral_error_abs = abs(lateral_error)
    distance_error_abs = abs(distance_error)
    
    if distance_error_abs <= DISTANCE_THRESHOLD and lateral_error_abs <= LATERAL_THRESHOLD:
        logger.info(f"✅ GOAL REACHED: Distance={distance:.3f}m, Lateral={lateral_error:.3f}m")
        return "S", f"✅ AT GOAL: Distance={distance:.3f}m, Lateral={lateral_error:.3f}m", {
            'pid_lateral': 0.0, 'pid_distance': 0.0
        }
    
    ANGLE_THRESHOLD = 0.15
    DISTANCE_MOVE_THRESHOLD = 0.10
    
    need_steer_left = lateral_error < -ANGLE_THRESHOLD
    need_steer_right = lateral_error > ANGLE_THRESHOLD
    
    need_forward = False
    
    if distance_error > DISTANCE_MOVE_THRESHOLD:
        if z_robot < -0.05:
            need_forward = True
        elif z_robot > 0.05:
            need_forward = False
        else:
            if distance > TARGET_DISTANCE_M * 1.5:
                need_forward = True
    
    if distance < TARGET_DISTANCE_M * 1.2:
        if distance_error > 0.08:
            need_forward = z_robot < -0.03
        else:
            need_forward = False
    
    if need_forward and need_steer_left:
        cmd = "G"
        phase = f"Approaching: Forward-Left (dist={distance:.3f}m, lat={lateral_error*100:.1f}cm)"
    elif need_forward and need_steer_right:
        cmd = "I"
        phase = f"Approaching: Forward-Right (dist={distance:.3f}m, lat={lateral_error*100:.1f}cm)"
    elif need_forward:
        cmd = "F"
        phase = f"Approaching: Forward (dist={distance:.3f}m, z={z_robot:.3f}m)"
    elif need_steer_left:
        cmd = "L"
        phase = f"Aligning: Steer Left (lat={lateral_error*100:.1f}cm)"
    elif need_steer_right:
        cmd = "R"
        phase = f"Aligning: Steer Right (lat={lateral_error*100:.1f}cm)"
    else:
        cmd = "S"
        phase = f"Stopped: Close to goal (dist={distance:.3f}m, err={distance_error*100:.1f}cm)"
    
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


# ============================================================================
# MAIN SYSTEM LOOP
# ============================================================================

def main():
    global current_state, destination_id, arm_control_running, base, joint, grip
    global held, held_special
    
    print("=" * 70)
    print("INTEGRATED ROBOT NAVIGATION & ARM CONTROL SYSTEM")
    print("=" * 70)
    print("\nSYSTEM OVERVIEW:")
    print("  - Press 'm' for manual robot drive")
    print("  - Press 'n' for autonomous navigation")
    print("  - Press 'a' for manual arm control")
    print("  - All modes can be accessed independently")
    print("\nROBOT NAVIGATION CONTROLS:")
    print("  'm' - Manual drive mode")
    print("  'n' - Autonomous navigation (you'll be prompted for destination ID)")
    print("  'a' - Manual arm control")
    print("  'q' - Quit entire system")
    print("\nMANUAL DRIVE KEYS (when enabled):")
    print("  W - Forward    |  S - Backward")
    print("  A - Left       |  D - Right")
    print("  Q - Fwd-Left   |  E - Fwd-Right")
    print("  Z - Back-Left  |  C - Back-Right")
    print("  X - Stop       |  N - Start autonomous nav")
    print("  ESC - Exit manual drive mode")
    print("\nARM CONTROL KEYS (when enabled):")
    print("  Arrow Left/Right - base rotation")
    print("  Arrow Up/Down    - joint angle")
    print("  Q/E              - gripper open/close")
    print("  R                - return to home position")
    print("  P                - print current pose")
    print("  ESC              - exit arm control mode")
    print("=" * 70)
    
    # Connect to navigation ESP
    logger.info("Connecting to navigation ESP32...")
    safe_connect_nav_forever()
    
    # Load camera calibration
    logger.info("Loading camera calibration...")
    camera_matrix, dist_coeffs, using_defaults = load_calibration("camera_calibration.yml")
    if using_defaults:
        logger.warning("Using default calibration values!")
    
    marker_length_m = 0.1016
    
    # Initialize ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    
    # Open camera
    logger.info("Opening camera...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logger.error("Could not open camera!")
        raise RuntimeError("Could not open camera.")
    
    # Initialize PID controllers
    pid_lateral = PIDController(PID_LATERAL_KP, PID_LATERAL_KI, PID_LATERAL_KD,
                                PID_OUTPUT_MIN, PID_OUTPUT_MAX)
    pid_distance = PIDController(PID_DISTANCE_KP, PID_DISTANCE_KI, PID_DISTANCE_KD,
                                 PID_OUTPUT_MIN, PID_OUTPUT_MAX)
    
    obj_points = np.array([
        [-marker_length_m / 2, marker_length_m / 2, 0],
        [marker_length_m / 2, marker_length_m / 2, 0],
        [marker_length_m / 2, -marker_length_m / 2, 0],
        [-marker_length_m / 2, -marker_length_m / 2, 0]
    ], dtype=np.float32)
    
    last_cmd = None
    last_cmd_time = 0
    last_cmd_change_time = 0
    
    print(f"\n✅ System ready! Current state: {current_state}")
    print("Press 'm' (manual drive) | 'n' (autonomous nav) | 'a' (arm control)...\n")
    
    while True:
        # ====================================================================
        # STATE: WAITING_TO_START
        # ====================================================================
        if current_state == SystemState.WAITING_TO_START:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Detect markers for visualization
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            cv2.putText(frame, f"STATE: {current_state}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, "Press 'm' (drive) | 'n' (auto nav) | 'a' (arm control)",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Integrated Robot System", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("Quit key pressed")
                send_nav_command("S")
                break
            elif key == ord('a'):
                # Start arm control mode
                current_state = SystemState.ARM_CONTROL
                logger.info("Starting arm control mode")
                print("\n" + "=" * 70)
                print("🦾 ARM CONTROL MODE ACTIVATED")
                print("=" * 70)
                
                # Connect to arm ESP32
                try:
                    endpoint = connect_arm()
                    print(f"✅ Connected to arm ESP32 at {endpoint}")
                except Exception as e:
                    logger.error(f"Failed to connect to arm ESP32: {e}")
                    print(f"❌ Failed to connect to arm ESP32 at {ARM_ESP_IP}:{ARM_PORT}")
                    print(f"   Error: {e}")
                    print(f"   Make sure arm ESP32 is on the network!")
                    current_state = SystemState.WAITING_TO_START
                    continue
                
                # Reset arm to home position
                base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
                send_arm_pose()
                
                print("\nARM CONTROLS:")
                print("  Arrow Left/Right - base rotation")
                print("  Arrow Up/Down    - joint angle")
                print("  Q/E              - gripper open/close")
                print("  R                - home position")
                print("  P                - print pose")
                print("  ESC              - exit arm control")
                print("=" * 70 + "\n")
                
                # Start keyboard listener
                arm_control_running = True
                held.clear()
                held_special.clear()
                listener = keyboard.Listener(on_press=on_press, on_release=on_release)
                listener.start()
            elif key == ord('m'):
                # Start manual navigation mode
                current_state = SystemState.MANUAL_NAVIGATION
                logger.info("Starting manual navigation mode")
                print("\n" + "=" * 70)
                print("🎮 MANUAL DRIVE MODE ACTIVATED")
                print("=" * 70)
                print("\nMANUAL DRIVE CONTROLS:")
                print("  W - Forward    |  S - Backward")
                print("  A - Left       |  D - Right")
                print("  Q - Fwd-Left   |  E - Fwd-Right")
                print("  Z - Back-Left  |  C - Back-Right")
                print("  X - Stop")
                print("  N - Start autonomous navigation")
                print("  ESC - Exit manual drive")
                print("=" * 70 + "\n")
                
                manual_nav_running = True
                held.clear()
                held_special.clear()
                listener = keyboard.Listener(on_press=on_press, on_release=on_release)
                listener.start()
            elif key == ord('n'):
                # Get destination ID from user
                print("\n" + "=" * 70)
                dest_input = input("Enter destination marker ID: ")
                try:
                    destination_id = int(dest_input)
                    current_state = SystemState.NAVIGATING
                    logger.info(f"Starting navigation to marker ID {destination_id}")
                    print(f"✅ Navigation started to marker ID {destination_id}")
                    print("=" * 70 + "\n")
                except ValueError:
                    print("❌ Invalid ID. Please try again.")
        
        # ====================================================================
        # STATE: MANUAL_NAVIGATION
        # ====================================================================
        elif current_state == SystemState.MANUAL_NAVIGATION:
            ret, frame = cap.read()
            if not ret:
                break
            
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            cv2.putText(frame, f"STATE: MANUAL NAVIGATION",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)
            cv2.putText(frame, "WASD: Drive | QE: Diagonals | X: Stop | N: Auto Nav | ESC: Exit",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            cv2.imshow("Integrated Robot System", frame)
            
            # Process manual navigation commands
            cmd = None
            
            if 'w' in held:
                cmd = "F"  # Forward
            elif 's' in held:
                cmd = "B"  # Backward
            elif 'a' in held:
                cmd = "L"  # Left
            elif 'd' in held:
                cmd = "R"  # Right
            elif 'q' in held:
                cmd = "G"  # Forward-Left
            elif 'e' in held:
                cmd = "I"  # Forward-Right
            elif 'z' in held:
                cmd = "H"  # Backward-Left
            elif 'c' in held:
                cmd = "J"  # Backward-Right
            elif 'x' in held:
                cmd = "S"  # Stop
            
            if cmd:
                send_nav_command(cmd)
            
            # Check if user wants to exit manual nav or start autonomous nav
            if not manual_nav_running:
                logger.info("Exiting manual navigation mode")
                send_nav_command("S")
                listener.stop()
                current_state = SystemState.WAITING_FOR_NEXT
                print("\n" + "=" * 70)
                print("✅ MANUAL DRIVE COMPLETE")
                print("=" * 70)
                print("Press 'm' (drive) | 'n' (auto nav) | 'a' (arm control) | 'q' (quit)...")
                print("=" * 70 + "\n")
            
            key = cv2.waitKey(50) & 0xFF
            if key == ord('q'):
                logger.info("Quit key pressed")
                send_nav_command("S")
                manual_nav_running = False
                listener.stop()
                break
            elif key == ord('n'):
                # Start autonomous navigation
                print("\n" + "=" * 70)
                dest_input = input("Enter destination marker ID: ")
                try:
                    destination_id = int(dest_input)
                    manual_nav_running = False
                    listener.stop()
                    current_state = SystemState.NAVIGATING
                    logger.info(f"Switching to autonomous navigation to marker ID {destination_id}")
                    print(f"✅ Autonomous navigation started to marker ID {destination_id}")
                    print("=" * 70 + "\n")
                except ValueError:
                    print("❌ Invalid ID. Please try again.")
        
        # ====================================================================
        # STATE: NAVIGATING
        # ====================================================================
        elif current_state == SystemState.NAVIGATING:
            ret, frame = cap.read()
            if not ret:
                break
            
            corners, ids, _ = detector.detectMarkers(frame)
            
            robot_center_2d = None
            dest_center_2d = None
            robot_pos_3d = None
            dest_pos_3d = None
            robot_rvec = None
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    img_points = corner[0].reshape(-1, 2).astype(np.float32)
                    center_2d = tuple(np.mean(img_points, axis=0).astype(int))
                    
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, img_points, camera_matrix, dist_coeffs
                    )
                    
                    if success:
                        position_3d = tvec.flatten()
                        
                        if marker_id == ROBOT_ID:
                            robot_center_2d = center_2d
                            robot_pos_3d = position_3d
                            robot_rvec = rvec
                        elif marker_id == destination_id:
                            dest_center_2d = center_2d
                            dest_pos_3d = position_3d
            
            status_text = ""
            cmd = "S"
            phase = None
            
            if robot_center_2d is not None and dest_center_2d is not None:
                if robot_pos_3d is not None and dest_pos_3d is not None and robot_rvec is not None:
                    dest_in_robot_frame, robot_R = transform_to_robot_frame(
                        robot_pos_3d, robot_rvec, dest_pos_3d
                    )
                    
                    lateral_error, distance_error, distance, angle_deg = compute_robot_frame_errors(
                        dest_in_robot_frame, TARGET_DISTANCE_M
                    )
                    
                    z_robot = dest_in_robot_frame[2]
                    distance_cm = distance * 100
                    
                    lateral_ok = abs(lateral_error) <= LATERAL_THRESHOLD
                    distance_ok = abs(distance_error) <= DISTANCE_THRESHOLD
                    
                    if lateral_ok and distance_ok:
                        cmd = "S"
                        phase = f"✅ TARGET REACHED!"
                        status_text = f"{phase} Distance: {distance_cm:.1f} cm"
                        logger.info(f"✅ Navigation complete! Distance: {distance_cm:.1f} cm")
                        
                        # Transition to next state
                        current_state = SystemState.NAVIGATION_COMPLETE
                        send_nav_command("S")
                        pid_lateral.reset()
                        pid_distance.reset()
                        
                        print("\n" + "=" * 70)
                        print("🎯 NAVIGATION COMPLETE!")
                        print("=" * 70)
                        print("Ready for manual arm control.")
                        print("Press 'a' to start arm control...")
                        print("=" * 70 + "\n")
                    else:
                        cv2.line(frame, robot_center_2d, dest_center_2d, (0, 255, 255), 3)
                        
                        cmd, phase, pid_outputs = compute_navigation_command_robot_frame(
                            lateral_error, distance_error, distance, z_robot,
                            pid_lateral, pid_distance
                        )
                        
                        x_robot, y_robot, z_robot_val = dest_in_robot_frame
                        z_direction = "FRONT" if z_robot_val < 0 else "BEHIND"
                        status_text = (f"{phase} | Distance: {distance_cm:.1f} cm | "
                                     f"Lateral: {lateral_error*100:.1f} cm | Cmd: {cmd}")
                        
                        current_time = time.time()
                        time_since_last_change = current_time - last_cmd_change_time
                        
                        if cmd != last_cmd:
                            if time_since_last_change >= MIN_COMMAND_INTERVAL:
                                send_nav_command(cmd)
                                last_cmd = cmd
                                last_cmd_change_time = current_time
                                last_cmd_time = current_time
                            else:
                                if (current_time - last_cmd_time) >= TICK:
                                    send_nav_command(last_cmd)
                                    last_cmd_time = current_time
                        else:
                            if (current_time - last_cmd_time) >= TICK:
                                send_nav_command(cmd)
                                last_cmd_time = current_time
                
                else:
                    cv2.line(frame, robot_center_2d, dest_center_2d, (0, 255, 255), 3)
                    status_text = "⚠️ Waiting for 3D pose data..."
                    cmd = "S"
                    if cmd != last_cmd:
                        send_nav_command(cmd)
                        last_cmd = cmd
            
            elif robot_center_2d is None:
                status_text = f"⚠️ Robot marker (ID {ROBOT_ID}) not detected"
                cmd = "S"
                if cmd != last_cmd:
                    send_nav_command(cmd)
                    last_cmd = cmd
            
            elif dest_center_2d is None:
                status_text = f"⚠️ Destination marker (ID {destination_id}) not detected"
                cmd = "S"
                if cmd != last_cmd:
                    send_nav_command(cmd)
                    last_cmd = cmd
            
            cv2.putText(frame, f"STATE: NAVIGATING to ID {destination_id}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, status_text,
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            cv2.imshow("Integrated Robot System", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("Quit key pressed")
                send_nav_command("S")
                break
            elif key == ord('s'):
                send_nav_command("S")
                print("⏸️ Robot stopped")
        
        # ====================================================================
        # STATE: NAVIGATION_COMPLETE
        # ====================================================================
        elif current_state == SystemState.NAVIGATION_COMPLETE:
            ret, frame = cap.read()
            if not ret:
                break
            
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            cv2.putText(frame, f"STATE: NAVIGATION COMPLETE",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, "Press 'a' to start arm control",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow("Integrated Robot System", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("Quit key pressed")
                break
            elif key == ord('a'):
                current_state = SystemState.ARM_CONTROL
                logger.info("Starting arm control mode")
                print("\n" + "=" * 70)
                print("🦾 ARM CONTROL MODE ACTIVATED")
                print("=" * 70)
                
                # Connect to arm ESP32
                try:
                    endpoint = connect_arm()
                    print(f"✅ Connected to arm ESP32 at {endpoint}")
                except Exception as e:
                    logger.error(f"Failed to connect to arm ESP32: {e}")
                    print(f"❌ Failed to connect to arm ESP32 at {ARM_ESP_IP}:{ARM_PORT}")
                    print(f"   Error: {e}")
                    current_state = SystemState.NAVIGATION_COMPLETE
                    continue
                
                # Reset arm to home position
                base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
                send_arm_pose()
                
                print("\nARM CONTROLS:")
                print("  Arrow Left/Right - base rotation")
                print("  Arrow Up/Down    - joint angle")
                print("  Q/E              - gripper open/close")
                print("  R                - home position")
                print("  P                - print pose")
                print("  ESC              - exit arm control")
                print("=" * 70 + "\n")
                
                # Start keyboard listener
                arm_control_running = True
                held.clear()
                held_special.clear()
                listener = keyboard.Listener(on_press=on_press, on_release=on_release)
                listener.start()
        
        # ====================================================================
        # STATE: ARM_CONTROL
        # ====================================================================
        elif current_state == SystemState.ARM_CONTROL:
            ret, frame = cap.read()
            if not ret:
                break
            
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            cv2.putText(frame, f"STATE: ARM CONTROL",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            cv2.putText(frame, f"Pose: B={base} J={joint} G={grip}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, "Press ESC to exit arm control",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            cv2.imshow("Integrated Robot System", frame)
            
            # Process arm movements
            moved = False
            
            if keyboard.Key.left in held_special:  base -= STEP_DEG; moved = True
            if keyboard.Key.right in held_special: base += STEP_DEG; moved = True
            if keyboard.Key.up in held_special:    joint += STEP_DEG; moved = True
            if keyboard.Key.down in held_special:  joint -= STEP_DEG; moved = True
            
            if 'a' in held: base -= STEP_DEG; moved = True
            if 'd' in held: base += STEP_DEG; moved = True
            if 'w' in held: joint += STEP_DEG; moved = True
            if 's' in held: joint -= STEP_DEG; moved = True
            
            if 'q' in held: grip -= STEP_GRIP; moved = True
            if 'e' in held: grip += STEP_GRIP; moved = True
            
            if moved:
                send_arm_pose()
            
            # Check if user wants to exit arm control
            if not arm_control_running:
                logger.info("Exiting arm control mode")
                listener.stop()
                disconnect_arm()
                
                current_state = SystemState.WAITING_FOR_NEXT
                print("\n" + "=" * 70)
                print("✅ ARM CONTROL COMPLETE")
                print("=" * 70)
                print("Ready for next task.")
                print("Press 'm' (drive) | 'n' (auto nav) | 'a' (arm control) | 'q' (quit)...")
                print("=" * 70 + "\n")
            
            key = cv2.waitKey(int(1000 / ARM_SEND_HZ)) & 0xFF
            if key == ord('q'):
                logger.info("Quit key pressed")
                arm_control_running = False
                listener.stop()
                if arm_serial:
                    arm_serial.close()
                break
        
        # ====================================================================
        # STATE: WAITING_FOR_NEXT
        # ====================================================================
        elif current_state == SystemState.WAITING_FOR_NEXT:
            ret, frame = cap.read()
            if not ret:
                break
            
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            cv2.putText(frame, f"STATE: READY FOR NEXT TASK",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, "Press 'm' (drive) | 'n' (auto nav) | 'a' (arm control) | 'q' (quit)",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow("Integrated Robot System", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("Quit key pressed")
                break
            elif key == ord('a'):
                # Start arm control mode
                current_state = SystemState.ARM_CONTROL
                logger.info("Starting arm control mode")
                print("\n" + "=" * 70)
                print("🦾 ARM CONTROL MODE ACTIVATED")
                print("=" * 70)
                
                # Connect to arm ESP32
                try:
                    endpoint = connect_arm()
                    print(f"✅ Connected to arm ESP32 at {endpoint}")
                except Exception as e:
                    logger.error(f"Failed to connect to arm ESP32: {e}")
                    print(f"❌ Failed to connect to arm ESP32 at {ARM_ESP_IP}:{ARM_PORT}")
                    print(f"   Error: {e}")
                    current_state = SystemState.WAITING_FOR_NEXT
                    continue
                
                # Reset arm to home position
                base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
                send_arm_pose()
                
                print("\nARM CONTROLS:")
                print("  Arrow Left/Right - base rotation")
                print("  Arrow Up/Down    - joint angle")
                print("  Q/E              - gripper open/close")
                print("  R                - home position")
                print("  P                - print pose")
                print("  ESC              - exit arm control")
                print("=" * 70 + "\n")
                
                # Start keyboard listener
                arm_control_running = True
                held.clear()
                held_special.clear()
                listener = keyboard.Listener(on_press=on_press, on_release=on_release)
                listener.start()
            elif key == ord('m'):
                # Start manual navigation mode
                current_state = SystemState.MANUAL_NAVIGATION
                logger.info("Starting manual navigation mode")
                print("\n" + "=" * 70)
                print("🎮 MANUAL DRIVE MODE ACTIVATED")
                print("=" * 70)
                print("\nMANUAL DRIVE CONTROLS:")
                print("  W - Forward    |  S - Backward")
                print("  A - Left       |  D - Right")
                print("  Q - Fwd-Left   |  E - Fwd-Right")
                print("  Z - Back-Left  |  C - Back-Right")
                print("  X - Stop")
                print("  N - Start autonomous navigation")
                print("  ESC - Exit manual drive")
                print("=" * 70 + "\n")
                
                manual_nav_running = True
                held.clear()
                held_special.clear()
                listener = keyboard.Listener(on_press=on_press, on_release=on_release)
                listener.start()
            elif key == ord('n'):
                print("\n" + "=" * 70)
                dest_input = input("Enter destination marker ID: ")
                try:
                    destination_id = int(dest_input)
                    current_state = SystemState.NAVIGATING
                    pid_lateral.reset()
                    pid_distance.reset()
                    last_cmd = None
                    logger.info(f"Starting new navigation to marker ID {destination_id}")
                    print(f"✅ Navigation started to marker ID {destination_id}")
                    print("=" * 70 + "\n")
                except ValueError:
                    print("❌ Invalid ID. Please try again.")
    
    # Cleanup
    logger.info("Cleaning up resources...")
    cap.release()
    cv2.destroyAllWindows()
    if nav_sock:
        try:
            nav_sock.close()
            logger.info("Navigation socket closed")
        except Exception as e:
            logger.warning(f"Error closing navigation socket: {e}")
    disconnect_arm()
    logger.info("✅ System shutdown complete")
    print("\n✅ System shutdown complete")


if __name__ == "__main__":
    main()
