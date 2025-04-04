import cv2
import numpy as np
import pyrealsense2 as rs
import time  # Import time module for sleep

# Configure and start the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# Get depth scale (conversion factor to meters)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is:", depth_scale)

# Align depth frames to color frames
align_to = rs.stream.color
align = rs.align(align_to)

# Wait for frames and get camera intrinsics
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()
if not depth_frame or not color_frame:
    print("Could not acquire frames")
    exit(0)

color_image = np.asanyarray(color_frame.get_data())
intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
print("Camera intrinsics:", intrinsics)

# Load the ArUco dictionary and create detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None and len(ids) > 0:
        print("Detected marker IDs:", ids.flatten())
        for i in range(len(ids)):
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
            c = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
            
            # Get depth at marker center and compute 3D position
            depth = depth_frame.get_distance(cx, cy)
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
            print(f"Marker ID: {ids[i][0]} at 3D coordinates: {point_3d}")
            
            # Overlay marker ID text
            cv2.putText(color_image, f"ID:{ids[i][0]}", (cx, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Create a text string with the 3D coordinates (formatted to 2 decimal places)
            pos_text = f"({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})"
            # Overlay the position text slightly below the marker center
            cv2.putText(color_image, pos_text, (cx, cy + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    else:
        # Debug message if no markers are detected
        print("No markers detected in this frame.")

    cv2.imshow("Aruco Marker Detection", color_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Add a 0.1 second delay to control the loop rate
    time.sleep(0.5)

pipeline.stop()
cv2.destroyAllWindows()
