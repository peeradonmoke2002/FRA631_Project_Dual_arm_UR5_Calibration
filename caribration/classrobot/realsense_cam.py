import pyrealsense2 as rs
import cv2
import numpy as np
import math
from .point3d import Point3D
import json
import os

config_path = os.path.join(os.path.dirname(__file__), "..", "config", "cam.json")
jsonObj = json.load(open(config_path))
json_string = str(jsonObj).replace("'", '\"')



class RealsenseCam:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.profile = None
        self.align = None
        self.align_depth = None 
        self.init_cam()
        print("RealsenseCam initialized with width: {}, height: {}, fps: {}".format(width, height, fps))
  

    def init_cam(self):
        """
        Initialize the RealSense camera pipeline with both color and depth streams.
        This method is called in the constructor.
        """
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            if self.pipeline is None or self.config is None:
                raise RuntimeError("Failed to create RealSense pipeline or config.")

            # Enable color and depth streams before starting the pipeline
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

            # Start the pipeline and get the profile
            self.profile = self.pipeline.start(self.config)

            # Align depth to color
            self.align = rs.align(rs.stream.color)
            self.align_depth = rs.align(rs.stream.depth)

            # Get device and enable advanced mode
            dev = self.profile.get_device()

            if not dev:
                raise RuntimeError("Failed to get RealSense device.")

            advnc_mode = rs.rs400_advanced_mode(dev)
            if not advnc_mode.is_enabled():
                print("Enabling advanced mode...")
                advnc_mode.toggle_advanced_mode(True)

            # Load settings from config JSON
            advnc_mode.load_json(json_string)

            print(f"RealSense camera started with aligned color and depth streams "
                f"({self.width}x{self.height}@{self.fps}fps)")

        except Exception as e:
            print(f"[ERROR] Failed to initialize RealSense camera: {e}")
            raise  # Or use sys.exit(1) if running as a standalone script

    def get_color_frame(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            return None
        image = np.asanyarray(color_frame.get_data())
        return image

    def get_depth_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            return None
        # Return the depth frame (do not convert to numpy array so get_distance() remains available)
        return depth_frame

    def get_color_and_depth_frames(self) -> tuple:
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None, None
        
        # Get intrinsics from the depth stream.
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        color_image = np.asanyarray(color_frame.get_data())
        # Return the color image, the raw depth frame, and the intrinsics.
        return color_image, depth_frame, depth_intrinsics


    def get_color_intrinsics(self, depth_intrinsics) -> tuple:
        """
        Retrieve the color camera's intrinsic matrix and distortion coefficients.

        Returns:
            camera_matrix (np.ndarray): 3x3 intrinsic matrix.
            dist_coeffs (np.ndarray): Distortion coefficients array.
        """
        # Reuse our get_color_and_depth_frames to get the intrinsics
        
        if depth_intrinsics is None:
            return None, None
        # Build the 3x3 camera matrix from the intrinsics.
        camera_matrix = np.array([[depth_intrinsics.fx, 0, depth_intrinsics.ppx],
                                  [0, depth_intrinsics.fy, depth_intrinsics.ppy],
                                  [0, 0, 1]], dtype=np.float32)
        # Get the distortion coefficients.
        dist_coeffs = np.array(depth_intrinsics.coeffs, dtype=np.float32)
        return camera_matrix, dist_coeffs

    def get_depth_intrinsics(self, depth_frame) -> tuple:
        """
        Retrieve the depth camera's intrinsic matrix and distortion coefficients.

        Returns:
            camera_matrix (np.ndarray): 3x3 intrinsic matrix.
            dist_coeffs (np.ndarray): Distortion coefficients array.
        """
        if not depth_frame:
            return None, None
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        camera_matrix = np.array([[depth_intrinsics.fx, 0, depth_intrinsics.ppx],
                                  [0, depth_intrinsics.fy, depth_intrinsics.ppy],
                                  [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array(depth_intrinsics.coeffs, dtype=np.float32)
        return camera_matrix, dist_coeffs




    def get_board_pose(self, aruco_dict):
        color_image, depth_frame, depth_intrinsics = self.get_color_and_depth_frames()
        if color_image is None or depth_frame is None:
            print("Failed to capture color/depth.")
            return None, Point3D(0, 0, 0)  # Use Point3D, not Points3D

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        output_image = color_image.copy()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

            all_cx = []
            all_cy = []
            for i in range(len(ids)):
                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                all_cx.append(cx)
                all_cy.append(cy)
                cv2.circle(output_image, (cx, cy), 4, (0, 0, 255), -1)

            cx = int(np.mean(all_cx))
            cy = int(np.mean(all_cy))
            point2d = [cx, cy]
            print(cx,cy)
            depth = depth_frame.get_distance(cx, cy)
            if depth > 0:
                point_coords = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], depth)
                # Convert to Point3D 
                # x = result[2]
                # y = -point_coords[1]
                # z = -point_coords[2]
                x = point_coords[0]
                y = -point_coords[1]
                z = -point_coords[2]
                point3d = Point3D(x, y, z)
                print(f"Detected board center at: {point3d}")
                return output_image, point3d
            else:
                print("Invalid depth.")
                return output_image, Point3D(0, 0, 0)
        else:
            print("No markers detected.")
            return output_image, Point3D(0, 0, 0)




    def get_board_pose_estimate(self, aruco_dict, camera_matrix, dist_coeffs, marker_length=0.10):
        """
        Estimates the 6DoF pose of the detected Aruco marker board using solvePnP.
        
        Parameters:
        - aruco_dict: The dictionary used for marker detection (e.g., cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50))
        - camera_matrix: The camera intrinsic matrix (3x3 numpy array)
        - dist_coeffs: Distortion coefficients (numpy array, e.g., np.zeros((5,1), dtype=np.float32))
        - marker_length: The actual side length of the marker in meters (default: 0.05)
        
        Returns:
        - output_image: The color image with markers drawn.
        - rvec: Rotation vector (if successful) or None.
        - tvec: Translation vector (if successful) or a Point3D at (0,0,0).
        """
        # Capture color and depth frames.
        color_image, depth_frame, depth_intrinsics = self.get_color_and_depth_frames()
        if color_image is None or depth_frame is None:
            print("Failed to capture color/depth.")
            return None, None, Point3D(0, 0, 0)
        
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # Create Aruco detector and detect markers.
        parameters = cv2.aruco.DetectorParameters_create()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
        output_image = color_image.copy()
        
        if ids is not None and len(ids) > 0:
            # Draw detected markers for visualization.
            cv2.aruco.drawDetectedMarkers(output_image, corners, ids)
            
            # Prepare lists for image points and object points.
            image_points_list = []  # Will hold 2D points from all markers.
            object_points_list = []  # Corresponding known 3D points in marker coordinate system.
            
            # Define the 3D coordinates of the marker corners in the marker's object coordinate system.
            # This assumes that the marker is a square centered at the origin.
            half_len = marker_length / 2.0
            # Assuming the corners order is: top-left, top-right, bottom-right, bottom-left.
            obj_corners = np.array([
                [-half_len,  half_len, 0],
                [ half_len,  half_len, 0],
                [ half_len, -half_len, 0],
                [-half_len, -half_len, 0]
            ], dtype=np.float32)
            
            # Process each detected marker.
            for marker_corners in corners:
                # marker_corners is shape (1,4,2); extract the 2D points.
                pts = marker_corners[0]  # Now pts is (4,2)
                image_points_list.append(pts)
                object_points_list.append(obj_corners)
            
            # Concatenate all points into single arrays.
            image_points = np.vstack(image_points_list)  # shape (N*4, 2)
            object_points = np.vstack(object_points_list)  # shape (N*4, 3)
            
            # Now use solvePnP to estimate the pose of the marker board.
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            if success:
                # Optional: draw the coordinate axes on the detected marker.
                cv2.aruco.drawAxis(output_image, camera_matrix, dist_coeffs, rvec, tvec, marker_length * 0.5)
                
                # Convert tvec to a Point3D for consistency.
                point3d = Point3D(tvec[0][0], tvec[1][0], tvec[2][0])
                print(f"Detected board pose: Translation: {point3d}")
                print(f"Rotation vector: {rvec.ravel()}")
                return output_image, rvec, point3d
            else:
                print("solvePnP failed.")
                return output_image, None, Point3D(0, 0, 0)
        else:
            print("No markers detected.")
            return output_image, None, Point3D(0, 0, 0)

    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
            
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
            
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
            
        return roll_x, pitch_y, yaw_z # in radians

    def stop(self):
        """
        Stop the RealSense pipeline.
        """
        if self.pipeline is None:
            print("Pipeline is not initialized.")
            return
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
            self.config = None
            self.profile = None
            self.align = None
            self.align_depth = None 
            print("RealSense camera stopped.")
        else:
            print("Pipeline is already stopped or not initialized.")
 

    def restart(self):
        """
        Restart the RealSense pipeline.
        """
        if self.pipeline is None:
            print("Pipeline is not initialized. Cannot restart.")
            return
        print("Restarting RealSense camera...")
        # Stop the pipeline before reinitializing
        self.stop()
        self.init_cam()
        print("RealSense camera restarted with aligned color and depth streams.")

    def __del__(self):
        self.stop()



