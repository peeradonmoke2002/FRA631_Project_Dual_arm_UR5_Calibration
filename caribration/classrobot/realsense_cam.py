import pyrealsense2 as rs
import cv2
import cv2.aruco as aruco
import numpy as np
import sys
from .point3d import Point3D
import json
import os

config_path = os.path.join(os.path.dirname(__file__), "..", "config", "cam.json")
jsonObj = json.load(open(config_path))
json_string = str(jsonObj).replace("'", '\"')



class RealsenseCam:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        """
        Initialize the RealSense camera pipeline with both color and depth streams.
        
        Parameters:
            width (int): Width of the streams in pixels.
            height (int): Height of the streams in pixels.
            fps (int): Frame rate.
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable both color and depth streams.
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        # Start the pipeline and get the profile.
        self.profile = self.pipeline.start(self.config)
        
        # Create an align object to align depth frames to the color frame.
        self.align = rs.align(rs.stream.color)
        
        # Get the device and load advanced mode settings.
        dev = self.profile.get_device()
        advnc_mode = rs.rs400_advanced_mode(dev)
        if not advnc_mode.is_enabled():
            advnc_mode.toggle_advanced_mode(True)
        advnc_mode.load_json(json_string)
        print("RealSense camera started with aligned color and depth streams.")

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
        
        # Get intrinsics from the color stream.
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        color_image = np.asanyarray(color_frame.get_data())
        # Return the color image, the raw depth frame, and the intrinsics.
        return color_image, depth_frame, intrinsics

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
            print(cx,cy)
            depth = depth_frame.get_distance(cx, cy)
            if depth > 0:
                point_coords = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], depth)
                point3d = Point3D(point_coords[0], point_coords[1], point_coords[2])
                print(f"Detected board center at: {point3d}")
                return output_image, point3d
            else:
                print("Invalid depth.")
                return output_image, Point3D(0, 0, 0)
        else:
            print("No markers detected.")
            return output_image, Point3D(0, 0, 0)


    def stop(self):
        """
        Stop the RealSense pipeline.
        """
        self.pipeline.stop()
        print("RealSense camera stopped.")

    def restart(self):
        """
        Restart the RealSense pipeline.
        """
        self.stop()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        print("RealSense camera restarted with aligned color and depth streams.")
