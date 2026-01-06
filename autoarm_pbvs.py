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
CAM_INDEX = 0
BASE_LIM = (0, 180)
JOINT_LIM = (20, 160)
GRIP_LIM = (0, 100)  # Full range for stronger grip

HOME_BASE, HOME_JOINT, HOME_GRIP = 110, 160, 30  # Start more open
LIFT_JOINT = 160
GRIP_CLOSE_STEPS = [40, 50, 60, 70, 80, 90]  # Close harder in more steps
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
    
    # Extract training data
    pixels = np.array([[p['pixel_x'], p['pixel_y']] for p in points])
    bases = np.array([p['base'] for p in points])
    joints = np.array([p['joint'] for p in points])
    
    # Create RBF interpolators (smooth mapping from pixels to joint angles)
    base_interp = RBFInterpolator(pixels, bases, kernel='thin_plate_spline', smoothing=1.0)
    joint_interp = RBFInterpolator(pixels, joints, kernel='thin_plate_spline', smoothing=1.0)
    
    print(f"✓ Loaded calibration with {len(points)} points")
    
    # Print calibration point summary
    print("\nCalibration points:")
    for p in points:
        print(f"  {p['description']:15s}: pixel ({p['pixel_x']:3d}, {p['pixel_y']:3d}) → base={p['base']:3d}° joint={p['joint']:3d}°")
    
    return base_interp, joint_interp, data['image_size']

def pixel_to_joints(pixel_x, pixel_y, base_interp, joint_interp):
    """Convert pixel coordinates to joint angles using calibrated mapping"""
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
    
    # Red mask (two ranges because red wraps around hue)
    mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
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
    
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Camera not opened")
    
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
    approach_count = 0
    MAX_APPROACH_STEPS = 50
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        h, w = frame.shape[:2]
        detection = detect_red_object(frame)
        
        # Draw frame center
        cv2.circle(frame, (w//2, h//2), 5, (0, 255, 255), -1)
        
        if state == SEARCH:
            cv2.putText(frame, "SEARCHING...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            if detection is not None:
                cx, cy, area, bbox = detection
                print(f"✓ Object detected at pixel ({cx}, {cy}), area={int(area)}")
                state = APPROACH
                approach_count = 0
        
        elif state == APPROACH:
            if detection is None:
                print("✗ Lost object, returning to search")
                state = SEARCH
                continue
            
            cx, cy, area, (x, y, bw, bh) = detection
            
            # Draw detection
            cv2.rectangle(frame, (x, y), (x+bw, y+bh), (255, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
            
            # Calculate target joint angles from pixel position using calibration
            target_base, target_joint = pixel_to_joints(cx, cy, base_interp, joint_interp)
            
            # Smooth approach (move incrementally with proportional control)
            base_error = target_base - base
            joint_error = target_joint - joint
            
            # Proportional control with step limits
            step_size = 4.0
            base += clamp(base_error * 0.4, -step_size, step_size)
            joint += clamp(joint_error * 0.4, -step_size, step_size)
            
            send_pose(ser, base, joint, grip)
            
            # Check if aligned
            aligned = (abs(base_error) < 4) and (abs(joint_error) < 4)
            close_enough = area >= AREA_GRAB
            
            status_text = f"APPROACH: err=({base_error:+.1f}°, {joint_error:+.1f}°) area={int(area)}"
            color = (0, 255, 0) if aligned else (0, 165, 255)
            cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, f"Target: B={target_base:.0f}° J={target_joint:.0f}°", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Current: B={base:.0f}° J={joint:.0f}°", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Step: {approach_count}/{MAX_APPROACH_STEPS}", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            print(f"Approach #{approach_count}: base_err={base_error:+.1f}° joint_err={joint_error:+.1f}° area={int(area)} aligned={aligned}")
            
            approach_count += 1
            
            if aligned and close_enough:
                print(f"✓ Aligned and close enough (area={int(area)}). GRABBING!")
                state = GRAB
            elif approach_count > MAX_APPROACH_STEPS:
                print(f"⚠ Max approach steps reached, attempting grab anyway")
                state = GRAB
        
        elif state == GRAB:
            cv2.putText(frame, "GRABBING...", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            for gv in GRIP_CLOSE_STEPS:
                grip = gv
                send_pose(ser, base, joint, grip)
                time.sleep(0.3)
            
            print("✓ Gripper closed")
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
