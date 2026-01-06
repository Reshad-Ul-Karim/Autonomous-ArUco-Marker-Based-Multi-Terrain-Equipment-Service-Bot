"""
Complete system calibration - run this ONCE to generate calibration data
Creates a mapping from camera pixels to robot coordinates
"""

import cv2
import numpy as np
import serial
from serial.tools import list_ports
import json
import time

BAUD = 115200
CAM_INDEX = 0

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

def send_pose(ser, base, joint, grip):
    ser.write(f"{base},{joint},{grip}\n".encode())
    time.sleep(0.5)

def main():
    print("="*70)
    print("SYSTEM CALIBRATION - This creates pixel-to-robot mapping")
    print("="*70)
    
    port = find_port()
    if not port:
        raise RuntimeError("ESP32 port not found")
    print(f"Using port: {port}")
    
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(2)
    
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Camera not opened")
    
    # Calibration data storage
    calibration_points = []
    
    # Define test positions in joint space - EXTENSIVE vertical coverage
    # Format: (base_angle, joint_angle, description)
    # Joint angle: LOWER values = LOWER gripper position (extended down toward table)
    # HIGHER values = HIGHER gripper position (retracted up away from table)
    # JOINT_LIM = (20, 160)
    test_positions = [
        # Extremely Low height (fully extended down toward table) - joint 20-35
        (70, 22, "Left-ExtremelyLow"),
        (110, 22, "Center-ExtremelyLow"),
        (150, 22, "Right-ExtremelyLow"),
        
        # Very Low height - joint 40-55
        (70, 45, "Left-VeryLow"),
        (110, 45, "Center-VeryLow"),
        (150, 45, "Right-VeryLow"),
        
        # Low height - joint 60-75
        (70, 70, "Left-Low"),
        (110, 70, "Center-Low"),
        (150, 70, "Right-Low"),
        
        # Mid-low height - joint 80-95
        (70, 90, "Left-MidLow"),
        (110, 90, "Center-MidLow"),
        (150, 90, "Right-MidLow"),
        
        # Mid-high height - joint 100-120
        (70, 115, "Left-MidHigh"),
        (110, 115, "Center-MidHigh"),
        (150, 115, "Right-MidHigh"),
        
        # High height (retracted up away from table) - joint 135-155
        (70, 145, "Left-High"),
        (110, 145, "Center-High"),
        (150, 145, "Right-High"),
        
        # Additional vertical test points at center for precise Z-axis mapping
        (110, 28, "Center-Extra1_ExtremelyLow"),
        (110, 55, "Center-Extra2_VeryLow"),
        (110, 80, "Center-Extra3_Low"),
        (110, 105, "Center-Extra4_MidLow"),
        (110, 130, "Center-Extra5_MidHigh"),
        (110, 155, "Center-Extra6_High"),
        
        # Additional mid-points for left/right to ensure good horizontal coverage
        (90, 70, "MidLeft-Low"),
        (130, 70, "MidRight-Low"),
        (90, 90, "MidLeft-MidLow"),
        (130, 90, "MidRight-MidLow"),
    ]
    
    print("\nInstructions:")
    print("1. Place a SMALL RED MARKER at gripper tip (tape it securely)")
    print("2. Arm will move to 30 different positions")
    print("3. Coverage: Joint angles from 22° (VERY LOW/down) to 155° (HIGH/up)")
    print("4. Lower joint angle = gripper closer to table")
    print("5. Higher joint angle = gripper retracted away from table")
    print("6. Special focus on vertical (Z-axis) with 6 extra center points")
    print("\nPress ENTER to start...")
    input()
    
    grip = 40  # Open gripper
    
    for idx, (base, joint, desc) in enumerate(test_positions):
        print(f"\n[{idx+1}/{len(test_positions)}] Moving to: {desc} (base={base}°, joint={joint}°)")
        
        # Move arm
        send_pose(ser, base, joint, grip)
        time.sleep(1.5)  # Wait for arm to settle
        
        # Capture multiple frames and average
        print("   Detecting red marker...")
        pixel_positions = []
        
        for _ in range(10):  # Average over 10 frames
            ret, frame = cap.read()
            if not ret:
                continue
            
            # Detect red marker
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Clean mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    pixel_positions.append((cx, cy))
            
            time.sleep(0.05)
        
        if len(pixel_positions) < 5:
            print(f"   ❌ Failed to detect marker! Only {len(pixel_positions)} detections")
            print("   Make sure red marker is visible and try again")
            retry = input("   Press ENTER to retry this position, or 's' to skip: ")
            if retry.lower() != 's':
                # Retry this position
                idx -= 1
                continue
            else:
                print("   ⚠ Skipping this position")
                continue
        
        # Average pixel position
        avg_px = int(np.mean([p[0] for p in pixel_positions]))
        avg_py = int(np.mean([p[1] for p in pixel_positions]))
        
        print(f"   ✓ Detected at pixel ({avg_px}, {avg_py})")
        
        # Store calibration point
        calibration_points.append({
            'base': base,
            'joint': joint,
            'pixel_x': avg_px,
            'pixel_y': avg_py,
            'description': desc
        })
        
        # Show confirmation
        ret, frame = cap.read()
        if ret:
            cv2.circle(frame, (avg_px, avg_py), 10, (0, 255, 0), 2)
            cv2.putText(frame, desc, (avg_px+15, avg_py), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Calibration", frame)
            cv2.waitKey(500)
    
    if len(calibration_points) < 20:
        print("\n❌ ERROR: Not enough calibration points!")
        print(f"   Only collected {len(calibration_points)} points, need at least 20")
        print("   Please run calibration again")
        ser.close()
        cap.release()
        cv2.destroyAllWindows()
        return
    
    # Validate vertical coverage by checking pixel Y variation at center position
    center_points = [p for p in calibration_points if p['base'] == 110]
    if len(center_points) >= 3:
        center_y_pixels = [p['pixel_y'] for p in center_points]
        center_joints = [p['joint'] for p in center_points]
        
        y_range = max(center_y_pixels) - min(center_y_pixels)
        joint_range = max(center_joints) - min(center_joints)
        
        print(f"\n📊 Vertical coverage validation:")
        print(f"   Joint angle range at center: {joint_range}° ({min(center_joints)}° to {max(center_joints)}°)")
        print(f"   Pixel Y range at center: {y_range}px ({min(center_y_pixels)} to {max(center_y_pixels)})")
        
        if y_range < 50:
            print(f"   ⚠ WARNING: Pixel Y range is small ({y_range}px)")
            print(f"   Vertical resolution may be limited. Consider adjusting camera angle.")
        elif y_range > 200:
            print(f"   ✓ EXCELLENT: Large pixel Y range ({y_range}px) - good vertical sensitivity")
        else:
            print(f"   ✓ GOOD: Adequate pixel Y range for vertical tracking")
    
    # Get frame size
    ret, frame = cap.read()
    
    # Save calibration data
    calib_data = {
        'calibration_points': calibration_points,
        'image_size': (frame.shape[1], frame.shape[0]),
        'num_points': len(calibration_points),
        'vertical_coverage': {
            'joint_range': joint_range if len(center_points) >= 3 else 0,
            'pixel_y_range': y_range if len(center_points) >= 3 else 0
        }
    }
    
    with open('robot_calibration.json', 'w') as f:
        json.dump(calib_data, f, indent=2)
    
    print("\n" + "="*70)
    print("✓ CALIBRATION COMPLETE!")
    print(f"✓ Saved {len(calibration_points)} calibration points to 'robot_calibration.json'")
    print("="*70)
    print("\nCalibration quality:")
    print(f"  - Total points: {len(calibration_points)}")
    print(f"  - Vertical sensitivity: {y_range if len(center_points) >= 3 else 'N/A'}px range")
    print("\nNext step: Run 'python autoarm_pbvs.py' to use the calibration")
    
    # Return to home
    send_pose(ser, 110, 160, 40)
    
    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
