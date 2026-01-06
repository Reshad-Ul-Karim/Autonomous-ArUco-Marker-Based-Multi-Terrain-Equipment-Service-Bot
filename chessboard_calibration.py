import numpy as np
import cv2
import yaml
import os
from datetime import datetime

class ChessboardCalibration:
    def __init__(self, chessboard_size=(9, 6), square_size=1.0):
        """
        Initialize chessboard calibration
        
        Args:
            chessboard_size: Tuple (width, height) - number of inner corners
            square_size: Size of chessboard square in your preferred unit (mm, cm, etc)
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        
        # Termination criteria for corner refinement
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ...
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # Arrays to store object points and image points
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.img_size = None
        
    def capture_images_live(self, num_images=20, camera_id=0):
        """
        Capture calibration images from live camera feed
        
        Args:
            num_images: Number of images to capture
            camera_id: Camera device ID
        """
        cap = cv2.VideoCapture(camera_id)
        
        if not cap.isOpened():
            print("Error: Cannot open camera")
            return False
        
        print(f"\n=== Chessboard Camera Calibration ===")
        print(f"Chessboard size: {self.chessboard_size[0]}x{self.chessboard_size[1]} inner corners")
        print(f"Target images: {num_images}")
        print("\nInstructions:")
        print("- Show the chessboard pattern to the camera")
        print("- Press SPACE when corners are detected (shown in color)")
        print("- Move the board to different positions and angles")
        print("- Press 'q' to quit early")
        print("- Press 'c' to calibrate with current images\n")
        
        captured_count = 0
        
        while captured_count < num_images:
            ret, frame = cap.read()
            if not ret:
                print("Error: Cannot read frame")
                break
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            display_frame = frame.copy()
            
            # Find chessboard corners
            ret_corners, corners = cv2.findChessboardCorners(
                gray, 
                self.chessboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            
            # If corners found, refine and draw them
            if ret_corners:
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                cv2.drawChessboardCorners(display_frame, self.chessboard_size, corners_refined, ret_corners)
                
                # Add status text
                status = f"Corners detected! Press SPACE to capture ({captured_count}/{num_images})"
                cv2.putText(display_frame, status, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                status = f"Move board into view ({captured_count}/{num_images})"
                cv2.putText(display_frame, status, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('Chessboard Calibration', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and ret_corners:
                # Save the corner points
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners_refined)
                self.img_size = gray.shape[::-1]
                captured_count += 1
                print(f"✓ Image {captured_count}/{num_images} captured")
                
            elif key == ord('q'):
                print("\nCalibration cancelled by user")
                break
                
            elif key == ord('c') and captured_count > 0:
                print(f"\nStarting calibration with {captured_count} images...")
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        return captured_count > 0
    
    def calibrate_from_folder(self, folder_path):
        """
        Calibrate from images in a folder
        
        Args:
            folder_path: Path to folder containing calibration images
        """
        images = []
        for fname in os.listdir(folder_path):
            if fname.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
                images.append(os.path.join(folder_path, fname))
        
        if not images:
            print(f"No images found in {folder_path}")
            return False
        
        print(f"\nProcessing {len(images)} images from {folder_path}...")
        
        for idx, fname in enumerate(images):
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
            
            if ret:
                self.objpoints.append(self.objp)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                self.imgpoints.append(corners_refined)
                self.img_size = gray.shape[::-1]
                print(f"✓ [{idx+1}/{len(images)}] {os.path.basename(fname)}")
            else:
                print(f"✗ [{idx+1}/{len(images)}] {os.path.basename(fname)} - No corners found")
        
        return len(self.objpoints) > 0
    
    def calibrate(self):
        """
        Perform camera calibration using collected points
        """
        if len(self.objpoints) == 0:
            print("Error: No calibration data available")
            return False
        
        print(f"\nCalibrating with {len(self.objpoints)} images...")
        
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.objpoints, 
            self.imgpoints, 
            self.img_size, 
            None, 
            None
        )
        
        if ret:
            # Calculate reprojection error
            mean_error = 0
            for i in range(len(self.objpoints)):
                imgpoints2, _ = cv2.projectPoints(
                    self.objpoints[i], 
                    self.rvecs[i], 
                    self.tvecs[i], 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error
            
            mean_error /= len(self.objpoints)
            
            print("\n=== Calibration Successful ===")
            print(f"Reprojection Error: {mean_error:.4f} pixels")
            print(f"\nCamera Matrix:\n{self.camera_matrix}")
            print(f"\nDistortion Coefficients:\n{self.dist_coeffs.ravel()}")
            
            return True
        else:
            print("Calibration failed")
            return False
    
    def save_calibration(self, filename="camera_calibration.yml"):
        """
        Save calibration parameters to YAML file
        
        Args:
            filename: Output filename
        """
        if self.camera_matrix is None:
            print("Error: No calibration data to save")
            return False
        
        calibration_data = {
            'image_width': int(self.img_size[0]),
            'image_height': int(self.img_size[1]),
            'camera_matrix': self.camera_matrix.tolist(),
            'distortion_coefficients': self.dist_coeffs.tolist(),
            'calibration_date': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'num_images': len(self.objpoints),
            'chessboard_size': list(self.chessboard_size),
            'square_size': float(self.square_size)
        }
        
        with open(filename, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)
        
        print(f"\n✓ Calibration saved to: {filename}")
        return True
    
    def load_calibration(self, filename):
        """
        Load calibration parameters from YAML file
        
        Args:
            filename: Input filename
        """
        if not os.path.exists(filename):
            print(f"Error: File {filename} not found")
            return False
        
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)
        
        self.camera_matrix = np.array(data['camera_matrix'])
        self.dist_coeffs = np.array(data['distortion_coefficients'])
        self.img_size = (data['image_width'], data['image_height'])
        
        print(f"\n✓ Calibration loaded from: {filename}")
        return True
    
    def test_undistortion(self, camera_id=0):
        """
        Test the calibration by showing undistorted video feed
        
        Args:
            camera_id: Camera device ID
        """
        if self.camera_matrix is None:
            print("Error: No calibration data loaded")
            return
        
        cap = cv2.VideoCapture(camera_id)
        
        print("\n=== Testing Undistortion ===")
        print("Press 'q' to quit")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Undistort the image
            h, w = frame.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
            )
            undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, 
                                       None, new_camera_matrix)
            
            # Crop the image based on ROI
            x, y, w, h = roi
            undistorted = undistorted[y:y+h, x:x+w]
            
            # Resize for comparison
            frame_resized = cv2.resize(frame, (undistorted.shape[1], undistorted.shape[0]))
            
            # Show side by side
            comparison = np.hstack((frame_resized, undistorted))
            cv2.putText(comparison, "Original", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(comparison, "Undistorted", (frame_resized.shape[1] + 10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('Undistortion Test', comparison)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()


def main():
    """
    Main calibration workflow
    """
    print("\n" + "="*50)
    print("  CHESSBOARD CAMERA CALIBRATION")
    print("="*50)
    
    # Configuration
    chessboard_size = (9, 6)  # Change to your chessboard inner corners (width, height)
    square_size = 25.0  # Size of square in mm (change to your actual size)
    
    print(f"\nConfiguration:")
    print(f"- Chessboard size: {chessboard_size[0]}x{chessboard_size[1]} inner corners")
    print(f"- Square size: {square_size} mm")
    
    # Create calibration object
    calib = ChessboardCalibration(chessboard_size=chessboard_size, square_size=square_size)
    
    # Choose calibration mode
    print("\nSelect calibration mode:")
    print("1. Live camera capture")
    print("2. Use images from folder")
    choice = input("Enter choice (1 or 2): ").strip()
    
    if choice == "1":
        # Live capture mode
        num_images = int(input("Number of images to capture (recommended: 20-30): ") or "20")
        camera_id = int(input("Camera ID (default: 0): ") or "0")
        
        if calib.capture_images_live(num_images=num_images, camera_id=camera_id):
            if calib.calibrate():
                output_file = input("\nSave calibration as (default: camera_calibration.yml): ").strip()
                if not output_file:
                    output_file = "camera_calibration.yml"
                calib.save_calibration(output_file)
                
                # Test undistortion
                test = input("\nTest undistortion? (y/n): ").strip().lower()
                if test == 'y':
                    calib.test_undistortion(camera_id=camera_id)
    
    elif choice == "2":
        # Folder mode
        folder_path = input("Enter folder path containing calibration images: ").strip()
        
        if calib.calibrate_from_folder(folder_path):
            if calib.calibrate():
                output_file = input("\nSave calibration as (default: camera_calibration.yml): ").strip()
                if not output_file:
                    output_file = "camera_calibration.yml"
                calib.save_calibration(output_file)
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()
