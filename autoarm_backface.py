"""
Position-Based Visual Servoing using calibrated pixel-to-robot mapping
Much more reliable than image-based approach
"""

import cv2
import numpy as np
import serial
from serial.tools import list_ports
import json
import time
from scipy.interpolate import RBFInterpolator

BAUD = 115200
STREAM_URL = "http://172.27.106.74:81/stream"
CAMERA_POSITION = "BACK"  # "FRONT" or "BACK" - set to BACK for camera behind arm
BASE_LIM = (0, 180)
JOINT_LIM = (20, 160)
GRIP_LIM = (0, 100)  # Full range for stronger grip

HOME_BASE, HOME_JOINT, HOME_GRIP = 110, 160, 30  # Start more open
LIFT_JOINT = 160
GRIP_CLOSE_STEPS = [50, 65, 75, 85, 95, 100]  # Close tighter - goes to maximum grip
AREA_GRAB = 4000
AREA_MIN = 300

def find_port():
    candidates = []
    for p in list_ports.comports():
        dev = (p.device or "").lower()
        desc = (p.description or "").lower()
        if ("usbserial" in dev) or ("slab" in dev) or ("wch" in dev) or ("cp210" in desc) or ("ch340" in desc):
            candidates.append(p.device)
    for c in candidates:
        if str(c).startswith("/dev/cu."):
            return c
    return candidates[0] if candidates else None

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_pose(ser, base, joint, grip):
    base = clamp(int(base), *BASE_LIM)
    joint = clamp(int(joint), *JOINT_LIM)
    grip = clamp(int(grip), *GRIP_LIM)
    ser.write(f"{base},{joint},{grip}\n".encode())

def load_calibration():
    """Load calibration and create interpolators"""
    with open('robot_calibration.json', 'r') as f:
        data = json.load(f)

    points = data['calibration_points']
    img_size = data['image_size']
    calib_camera_pos = data.get('camera_position', 'UNKNOWN')

    # Validate camera position matches
    if calib_camera_pos != CAMERA_POSITION:
        print(f"\n⚠ WARNING: Camera position mismatch!")
        print(f"   Calibration was done with: {calib_camera_pos}")
        print(f"   Current setting: {CAMERA_POSITION}")
        print(f"   Please recalibrate with CAMERA_POSITION='{CAMERA_POSITION}' for accurate results\n")

    # Extract training data
    pixels = np.array([[p['pixel_x'], p['pixel_y']] for p in points])
    bases = np.array([p['base'] for p in points])
    joints = np.array([p['joint'] for p in points])

    # Create RBF interpolators (smooth mapping from pixels to joint angles)
    base_interp = RBFInterpolator(pixels, bases, kernel='thin_plate_spline', smoothing=1.0)
    joint_interp = RBFInterpolator(pixels, joints, kernel='thin_plate_spline', smoothing=1.0)

    print(f"✓ Loaded calibration with {len(points)} points")
    print(f"✓ Camera position: {CAMERA_POSITION} (calibrated with: {calib_camera_pos})")

    # Print calibration point summary
    print("\nCalibration points:")
    for p in points:
        print(f"  {p['description']:15s}: pixel ({p['pixel_x']:3d}, {p['pixel_y']:3d}) → base={p['base']:3d}° joint={p['joint']:3d}°")

    return base_interp, joint_interp, img_size

def pixel_to_joints(pixel_x, pixel_y, base_interp, joint_interp, img_width):
    """Convert pixel coordinates to joint angles using calibrated mapping"""
    # For back camera: invert X coordinate (left/right)
    # When object is on RIGHT of camera view, arm should move RIGHT (lower base angle)
    # When object is on LEFT of camera view, arm should move LEFT (higher base angle)
    if CAMERA_POSITION == "BACK":
        pixel_x = img_width - pixel_x
    
    pixel = np.array([[pixel_x, pixel_y]])
    base = float(base_interp(pixel)[0])
    joint = float(joint_interp(pixel)[0])

    # Clamp to limits
    base = clamp(base, *BASE_LIM)
    joint = clamp(joint, *JOINT_LIM)

    return base, joint

def detect_red_object(frame):
    """Detect red object and return (cx, cy, area, bbox) or None"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red mask - more flexible ranges for better detection
    # Lower red range (expanded hue 0-15, lower sat/val thresholds)
    mask1 = cv2.inRange(hsv, np.array([0, 80, 50]), np.array([15, 255, 255]))
    # Upper red range (expanded hue 165-180, lower sat/val thresholds)
    mask2 = cv2.inRange(hsv, np.array([165, 80, 50]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)

    # Clean mask
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find largest blob
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < AREA_MIN:
        return None

    M = cv2.moments(c)
    if M['m00'] == 0:
        return None

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    x, y, w, h = cv2.boundingRect(c)

    return cx, cy, area, (x, y, w, h)

def main():
    print("="*70)
    print("POSITION-BASED VISUAL SERVOING")
    print("="*70)

    # Load calibration
    try:
        base_interp, joint_interp, img_size = load_calibration()
        img_width, img_height = img_size
    except FileNotFoundError:
        print("\n❌ ERROR: robot_calibration.json not found!")
        print("Please run 'python calibrate_system.py' first")
        return
    except Exception as e:
        print(f"\n❌ ERROR loading calibration: {e}")
        print("Please run calibration again")
        return

    # Setup hardware
    port = find_port()
    if not port:
        raise RuntimeError("ESP32 port not found")
    print(f"Using port: {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(2)

    print(f"Connecting to ESP32-CAM stream: {STREAM_URL}")
    cap = cv2.VideoCapture(STREAM_URL)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to connect to ESP32-CAM stream at {STREAM_URL}")
    
    # Reduce buffer size to minimize latency (get latest frames, not old buffered ones)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # Test read to verify stream
    ret, test_frame = cap.read()
    if not ret:
        raise RuntimeError(f"Connected but cannot read frames from {STREAM_URL}")
    print(f"✓ ESP32-CAM connected ({test_frame.shape[1]}x{test_frame.shape[0]})")

    # State machine
    SEARCH, APPROACH, GRAB, LIFT, DONE = range(5)
    state = SEARCH

    # Start at home
    base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
    send_pose(ser, base, joint, grip)
    time.sleep(1)

    print("\n✓ System ready. Starting autonomous pick...")
    print("Press 'q' to quit\n")

    target_base, target_joint = None, None
    locked_target_base, locked_target_joint = None, None  # Locked position for grab
    initial_target_base, initial_target_joint = None, None  # Initial locked target from first detection
    approach_count = 0
    lost_frames_count = 0  # Track how long object has been lost
    MAX_APPROACH_STEPS = 50
    MAX_LOST_FRAMES = 30  # Allow 30 frames of lost detection before giving up

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Increase saturation for easier detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 1] = hsv[:, :, 1] * 1.5  # Increase saturation by 50%
        hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)  # Clip to valid range
        frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        h, w = frame.shape[:2]
        detection = detect_red_object(frame)

        # Draw frame center
        cv2.circle(frame, (w//2, h//2), 5, (0, 255, 255), -1)
        
        # Draw camera mode indicator
        mode_text = f"Camera: {CAMERA_POSITION}"
        mode_color = (0, 255, 0) if CAMERA_POSITION == "BACK" else (255, 128, 0)
        cv2.putText(frame, mode_text, (w - 180, h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)

        if state == SEARCH:
            cv2.putText(frame, "SEARCHING...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            if detection is not None:
                cx, cy, area, bbox = detection
                # LOCK initial target position immediately upon detection
                initial_target_base, initial_target_joint = pixel_to_joints(cx, cy, base_interp, joint_interp, img_width)
                print(f"✓ Object detected at pixel ({cx}, {cy}), area={int(area)}")
                print(f"✓ INITIAL TARGET LOCKED: base={initial_target_base:.0f}° joint={initial_target_joint:.0f}°")
                state = APPROACH
                approach_count = 0
                lost_frames_count = 0

        elif state == APPROACH:
            if detection is None:
                lost_frames_count += 1
                print(f"⚠ Object lost ({lost_frames_count}/{MAX_LOST_FRAMES} frames) - continuing to INITIAL target")
                
                if lost_frames_count > MAX_LOST_FRAMES:
                    print("✗ Object lost for too long, returning to search")
                    state = SEARCH
                    initial_target_base, initial_target_joint = None, None
                    continue
                
                # Continue using INITIAL locked target even when object is lost
                target_base = initial_target_base
                target_joint = initial_target_joint
                
                # Draw "LOST" indicator
                cv2.putText(frame, f"LOST {lost_frames_count}/{MAX_LOST_FRAMES} - Using locked target", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                # Object is visible - reset lost counter but keep using initial target
                lost_frames_count = 0
                cx, cy, area, (x, y, bw, bh) = detection

                # Draw detection
                cv2.rectangle(frame, (x, y), (x+bw, y+bh), (255, 0, 255), 2)
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)

                # Use INITIAL locked target (not current position)
                target_base = initial_target_base
                target_joint = initial_target_joint

            # Calculate errors
            base_error = target_base - base
            joint_error = target_joint - joint
            
            # Check if aligned
            aligned = (abs(base_error) < 4) and (abs(joint_error) < 4)
            close_enough = (detection is not None and area >= AREA_GRAB) or approach_count > MAX_APPROACH_STEPS

            # Move toward target
            base += np.clip(base_error * 0.3, -5, 5)
            joint += np.clip(joint_error * 0.3, -5, 5)
            send_pose(ser, base, joint, grip)

            # Display status
            status_text = f"APPROACH: err=({base_error:+.1f}°, {joint_error:+.1f}°)"
            if detection is not None:
                status_text += f" area={int(area)}"
            color = (0, 255, 0) if aligned else (0, 165, 255)
            cv2.putText(frame, status_text, (10, 30 if detection is not None else 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, f"Initial Target: B={initial_target_base:.0f}° J={initial_target_joint:.0f}°", (10, 60 if detection is not None else 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(frame, f"Current: B={base:.0f}° J={joint:.0f}°", (10, 85 if detection is not None else 115),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Step: {approach_count}/{MAX_APPROACH_STEPS}", (10, 110 if detection is not None else 140),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            area_str = f"area={int(area)}" if detection is not None else "area=LOST"
            print(f"Approach #{approach_count}: base_err={base_error:+.1f}° joint_err={joint_error:+.1f}° {area_str} aligned={aligned}")

            approach_count += 1

            if aligned and close_enough:
                # LOCK the target position - stop tracking from here
                locked_target_base = base
                locked_target_joint = joint
                area_msg = f"area={int(area)}" if detection is not None else "max steps"
                print(f"✓ TARGET LOCKED at base={locked_target_base:.0f}° joint={locked_target_joint:.0f}° ({area_msg}). GRABBING!")
                state = GRAB
            elif approach_count > MAX_APPROACH_STEPS:
                # Lock current position even if not perfectly aligned
                locked_target_base = base
                locked_target_joint = joint
                print(f"⚠ Max approach steps reached, locking position and attempting grab")
                state = GRAB

        elif state == GRAB:
            cv2.putText(frame, "GRABBING...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Locked pos: B={locked_target_base:.0f}° J={locked_target_joint:.0f}°", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # Use LOCKED position - no more tracking
            for gv in GRIP_CLOSE_STEPS:
                grip = gv
                send_pose(ser, locked_target_base, locked_target_joint, grip)
                time.sleep(0.3)

            print("✓ Gripper closed at locked position")
            # Update base/joint to locked position for lift
            base = locked_target_base
            joint = locked_target_joint
            state = LIFT

        elif state == LIFT:
            cv2.putText(frame, "LIFTING...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            joint = LIFT_JOINT
            send_pose(ser, base, joint, grip)
            time.sleep(1)

            print("✓ Object lifted")
            state = DONE

        elif state == DONE:
            cv2.putText(frame, "DONE ✓", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(frame, "Press 'r' to reset and pick again", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("PBVS Auto Pick", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('r') and state == DONE:
            # Reset for another pick
            print("\n--- Resetting for another pick ---\n")
            base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
            send_pose(ser, base, joint, grip)
            time.sleep(1)
            state = SEARCH
            approach_count = 0

        time.sleep(1/30.0)

    # Cleanup
    send_pose(ser, HOME_BASE, HOME_JOINT, HOME_GRIP)
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("\n✓ System stopped")

if __name__ == "__main__":
    main()
