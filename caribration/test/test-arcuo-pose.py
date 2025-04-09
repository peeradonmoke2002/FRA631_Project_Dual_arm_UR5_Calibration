import cv2
import numpy as np
import pyrealsense2 as rs
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from classrobot.realsense_cam import RealsenseCam

def main():
    # Create a RealSenseCam instance.
    realsense_cam = RealsenseCam()
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)



    
    # Get a frame to retrieve the intrinsics.
    color_image, depth_frame, depth_intrinsics = realsense_cam.get_color_and_depth_frames()
    if depth_intrinsics is None:
        print("Failed to obtain depth intrinsics.")
        return
    
    accel_data, gyro_data = realsense_cam.get_imu_frames()
    print("Accelerometer data:", accel_data)
    print("Gyroscope data:", gyro_data)
    
    # Use the provided method to obtain the camera matrix and distortion coefficients.
    camera_matrix, dist_coeffs = realsense_cam.get_color_intrinsics(depth_intrinsics)
    if camera_matrix is None or dist_coeffs is None:
        print("Failed to obtain camera intrinsic parameters.")
        return

    # Call the get_board_pose_estimate function.
    # We set marker_length to 0.10 (i.e., 10 cm, assuming that from the board’s center to edge is 5 cm).
    output_image, rvec, board_pose = realsense_cam.get_board_pose_estimate(
        aruco_dict, camera_matrix, dist_coeffs, marker_length=0.077
    )

    if output_image is not None:
        # Display the output image with detected markers and coordinate axes.
        cv2.imshow("Board Pose Estimate", output_image)
        print("Rotation vector (rvec):", rvec)
        print("Estimated board pose (translation) as Point3D:", board_pose)
        
        # Wait for a key press and close the window.
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No output image available from pose estimation.")

    # Stop the RealSense pipeline when finished.
    realsense_cam.stop()

if __name__ == "__main__":
    main()
