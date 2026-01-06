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
STREAM_URL = "http://172.27.106.74:81/stream"
CAMERA_POSITION = "BACK"  # Camera is mounted behind the arm

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
    print(f"Camera Position: {CAMERA_POSITION} (behind arm)")
    print("="*70)
    
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
    
    # Test read to verify stream is working
    ret, test_frame = cap.read()
    if not ret:
        raise RuntimeError(f"Connected to stream but cannot read frames from {STREAM_URL}")
    print(f"✓ ESP32-CAM stream connected successfully ({test_frame.shape[1]}x{test_frame.shape[0]})")
    
    # Calibration data storage
    calibration_points = []
    
    # Define test positions in joint space - EXTENSIVE vertical coverage
    # Format: (base_angle, joint_angle, description)
    # Joint angle: LOWER values = LOWER gripper position (extended down toward table)
    # HIGHER values = HIGHER gripper position (retracted up away from table)
    # JOINT_LIM = (20, 160)
    # NOTE: Camera is BEHIND arm, so base=70° appears RIGHT in camera view, base=150° appears LEFT
    test_positions = [
        # Extremely Low height (fully extended down toward table) - joint 20-35
        (70, 22, "Right-ExtremelyLow"),      # base 70° → RIGHT in back camera
        (110, 22, "Center-ExtremelyLow"),
        (150, 22, "Left-ExtremelyLow"),      # base 150° → LEFT in back camera
        
        # Very Low height - joint 40-55
        (70, 45, "Right-VeryLow"),
        (110, 45, "Center-VeryLow"),
        (150, 45, "Left-VeryLow"),
        
        # Low height - joint 60-75
        (70, 70, "Right-Low"),
        (110, 70, "Center-Low"),
        (150, 70, "Left-Low"),
        
        # Mid-low height - joint 80-95
        (70, 90, "Right-MidLow"),
        (110, 90, "Center-MidLow"),
        (150, 90, "Left-MidLow"),
        
        # Mid-high height - joint 100-120
        (70, 115, "Right-MidHigh"),
        (110, 115, "Center-MidHigh"),
        (150, 115, "Left-MidHigh"),
        
        # High height (retracted up away from table) - joint 135-155
        (70, 145, "Right-High"),
        (110, 145, "Center-High"),
        (150, 145, "Left-High"),
        
        # Additional vertical test points at center for precise Z-axis mapping
        (110, 28, "Center-Extra1_ExtremelyLow"),
        (110, 55, "Center-Extra2_VeryLow"),
        (110, 80, "Center-Extra3_Low"),
        (110, 105, "Center-Extra4_MidLow"),
        (110, 130, "Center-Extra5_MidHigh"),
        (110, 155, "Center-Extra6_High"),
        
        # Additional mid-points for left/right to ensure good horizontal coverage
        (90, 70, "MidRight-Low"),            # base 90° → MidRight in back camera
        (130, 70, "MidLeft-Low"),            # base 130° → MidLeft in back camera
        (90, 90, "MidRight-MidLow"),
        (130, 90, "MidLeft-MidLow"),
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
        time.sleep(3.0)  # Wait longer for arm to fully settle and stop vibrating
        
        # Capture multiple frames and average
        print("   Detecting red marker...")
        pixel_positions = []
        last_valid_pos = None  # Track last position to avoid jumping
        MAX_JUMP = 80  # Maximum pixel distance jump allowed between frames
        last_frame = None  # Store last valid frame
        last_mask = None  # Store last valid mask
        
        # Create debug window
        cv2.namedWindow("Detection Debug", cv2.WINDOW_NORMAL)
        
        for frame_idx in range(20):  # Capture more frames (20 instead of 10)
            ret, frame = cap.read()
            if not ret:
                print(f"      Frame {frame_idx+1}: Failed to read")
                continue
            
            # Detect red marker - VERY flexible ranges for better detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Lower red range (expanded hue 0-20, very low sat/val thresholds)
            mask1 = cv2.inRange(hsv, np.array([0, 50, 30]), np.array([20, 255, 255]))
            # Upper red range (expanded hue 160-180, very low sat/val thresholds)
            mask2 = cv2.inRange(hsv, np.array([160, 50, 30]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Clean mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Calculate center directly from mask pixels (center of mass of ALL red pixels)
            # This is more accurate than using contour centroid
            
            # Create debug visualization
            debug_frame = frame.copy()
            
            # Calculate moments of the entire mask
            M = cv2.moments(mask)
            
            if M['m00'] > 0:  # If there are any white pixels in mask
                # Calculate center of mass of ALL detected red pixels
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                total_area = int(M['m00'])
                
                # If we have a previous position, check distance
                if last_valid_pos is not None:
                    dist = np.sqrt((cx - last_valid_pos[0])**2 + (cy - last_valid_pos[1])**2)
                    if dist > MAX_JUMP:
                        # Position jumped too far - probably different object appeared
                        print(f"      Frame {frame_idx+1}: Position jump detected (dist={int(dist)}px), REJECTED")
                        cv2.putText(debug_frame, f"POSITION JUMP {int(dist)}px - REJECTED", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        cv2.circle(debug_frame, (cx, cy), 8, (0, 0, 255), 2)  # Red circle = rejected
                    else:
                        # Valid detection
                        pixel_positions.append((cx, cy))
                        last_valid_pos = (cx, cy)
                        last_frame = frame.copy()  # Store this frame
                        last_mask = mask.copy()  # Store this mask
                        
                        # Draw detection on debug frame
                        cv2.circle(debug_frame, (cx, cy), 8, (0, 255, 0), -1)  # Green dot = mask center
                        cv2.circle(debug_frame, (cx, cy), 3, (0, 0, 255), -1)  # Red dot in center
                        cv2.putText(debug_frame, f"MASK CENTER ({cx},{cy})", (cx+15, cy-10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(debug_frame, f"Area={total_area}px", (cx+15, cy+10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                        print(f"      Frame {frame_idx+1}: Mask center at ({cx}, {cy}), total_area={total_area}px")
                else:
                    # First detection - accept it
                    pixel_positions.append((cx, cy))
                    last_valid_pos = (cx, cy)
                    last_frame = frame.copy()  # Store this frame
                    last_mask = mask.copy()  # Store this mask
                    
                    cv2.circle(debug_frame, (cx, cy), 8, (0, 255, 0), -1)
                    cv2.circle(debug_frame, (cx, cy), 3, (0, 0, 255), -1)
                    cv2.putText(debug_frame, f"MASK CENTER ({cx},{cy})", (cx+15, cy-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    print(f"      Frame {frame_idx+1}: Initial mask center at ({cx}, {cy}), total_area={total_area}px")
            else:
                print(f"      Frame {frame_idx+1}: No red pixels detected in mask")
                cv2.putText(debug_frame, "NO DETECTION", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Draw expected region if we have last position
            if last_valid_pos is not None:
                cv2.circle(debug_frame, last_valid_pos, MAX_JUMP, (255, 255, 0), 1)  # Cyan circle = valid region
            
            # Show mask and detection side by side
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            combined = np.hstack([debug_frame, mask_color])
            cv2.imshow("Detection Debug", combined)
            cv2.waitKey(100)  # Show each frame for 100ms
            
            time.sleep(0.1)  # Slower sampling (100ms instead of 50ms)
        
        if len(pixel_positions) < 5:
            print(f"   ❌ Failed to detect marker! Only {len(pixel_positions)}/20 detections")
            print("   Possible issues:")
            print("     - Red marker not visible to camera")
            print("     - Lighting too bright/dark")
            print("     - Marker color not red enough")
            print("     - Camera focus issue")
            print("   Check the 'Detection Debug' window to see what camera sees")
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
        std_px = int(np.std([p[0] for p in pixel_positions]))
        std_py = int(np.std([p[1] for p in pixel_positions]))
        
        print(f"   ✓ Detected at pixel ({avg_px}, {avg_py})")
        print(f"     Stability: ±{std_px}px (X), ±{std_py}px (Y) | {len(pixel_positions)}/20 frames")
        
        # Store calibration point
        calibration_points.append({
            'base': base,
            'joint': joint,
            'pixel_x': avg_px,
            'pixel_y': avg_py,
            'description': desc
        })
        
        # Show confirmation using the LAST VALID frame and mask (so it matches the data)
        if last_frame is not None and last_mask is not None:
            # Use the stored frame and mask
            confirm_frame = last_frame.copy()
            
            # Overlay mask in semi-transparent red
            mask_overlay = confirm_frame.copy()
            mask_overlay[last_mask > 0] = [0, 0, 255]  # Red where mask is white
            confirm_frame = cv2.addWeighted(confirm_frame, 0.7, mask_overlay, 0.3, 0)
            
            # Recalculate mask center from stored mask to verify
            M_verify = cv2.moments(last_mask)
            if M_verify['m00'] > 0:
                verify_cx = int(M_verify['m10'] / M_verify['m00'])
                verify_cy = int(M_verify['m01'] / M_verify['m00'])
                
                # Draw actual mask center (should match avg_px, avg_py)
                cv2.circle(confirm_frame, (verify_cx, verify_cy), 12, (0, 255, 0), 3)  # Large green circle
                cv2.circle(confirm_frame, (verify_cx, verify_cy), 3, (255, 255, 255), -1)  # White center dot
                cv2.putText(confirm_frame, f"MASK CENTER: {desc}", (verify_cx+20, verify_cy-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(confirm_frame, f"({verify_cx}, {verify_cy})", (verify_cx+20, verify_cy+10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(confirm_frame, f"Saved avg: ({avg_px},{avg_py})", (verify_cx+20, verify_cy+30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.putText(confirm_frame, f"Stability: +/-{std_px},{std_py}px", (verify_cx+20, verify_cy+50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            cv2.imshow("Calibration", confirm_frame)
            cv2.waitKey(1500)  # Display longer (1.5 seconds)
    
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
        'camera_position': CAMERA_POSITION,
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
