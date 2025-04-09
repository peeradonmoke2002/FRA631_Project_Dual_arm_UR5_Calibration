import cv2
import numpy as np
import pyrealsense2 as rs
import pathlib


# Configure and start the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
try:
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
except Exception as e:
    print(f"Error starting RealSense pipeline: {e}")
    exit(1)

# Get depth scale (conversion factor to meters)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is:", depth_scale)

# Warm up: capture and discard several frames to allow the sensor to stabilize
for _ in range(30):
    pipeline.wait_for_frames()

# Align depth frames to color frames
align_to = rs.stream.color
align = rs.align(align_to)

# Wait for a single set of frames and get camera intrinsics
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()
if not depth_frame or not color_frame:
    print("Could not acquire frames")
    pipeline.stop()
    exit(0)

color_image = np.asanyarray(color_frame.get_data())
intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
print("Camera intrinsics:", intrinsics)

# Get image dimensions for boundary checks
img_height, img_width = color_image.shape[:2]

# Load the ArUco dictionary and create detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
parameters = cv2.aruco.DetectorParameters()

# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Convert captured image to grayscale for detection
gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

# Detect the markers in the image
corners, ids, rejected = detector.detectMarkers(gray)

if ids is not None and len(ids) > 0:
    print("Detected marker IDs:", ids.flatten())
    for i in range(len(ids)):
        # Draw detected markers on the image
        cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
        c = corners[i][0]
        # Compute the marker center using the average of its corner coordinates
        cx = int(np.mean(c[:, 0]))
        cy = int(np.mean(c[:, 1]))
        cv2.circle(color_image, (cx, cy), 2, (0, 0, 255), -1)
        
        # Use only the center pixel's depth value
        depth = depth_frame.get_distance(cx, cy)
        print(f"Depth at ({cx}, {cy}): {depth}")
        point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
        print(f"Marker ID: {ids[i][0]} at 3D coordinates: {point_3d}")
        
        # Adjust text positions so they remain within image bounds
        # For marker ID text: try to place above the marker; if too close to the top, place below.
        # id_text_y = cy - 10 if cy - 10 > 10 else cy + 20
        # cv2.putText(color_image, f"ID:{ids[i][0]}", (cx, id_text_y),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # # For 3D coordinates: try to place below; if too close to the bottom, move above.
        # coord_text_y = cy + 20 if cy + 20 < img_height - 10 else cy - 20
        # pos_text = f"({point_3d[0]:.2f}, {point_3d[1]:.2f}, {point_3d[2]:.2f})"
        # cv2.putText(color_image, pos_text, (cx, coord_text_y),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
else:
    print("No markers detected.")


# Path to your CSV file
filename = pathlib.Path(__file__).parent.parent / "images" / "captured_image.png"
cv2.imwrite(filename, color_image)
print(f"Image saved as {filename}")

# Display the captured image until a key is pressed
cv2.imshow("Captured Aruco Marker Image", color_image)
cv2.waitKey(0)

# Cleanup
pipeline.stop()
cv2.destroyAllWindows()
