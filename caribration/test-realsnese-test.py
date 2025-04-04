from classrobot import realsense_cam
import cv2
import numpy as np
from classrobot.point3d import Point3D
from classrobot.robot_movement import RobotControl
import time

# Init cam and robot
cam = realsense_cam.RealsenseCam()
# robot = RobotControl()
# robot.robot_init("192.168.200.10")

# Get robot position
# pos_robot = robot.robot_get_position()
# pos_robot_3d = [pos_robot[0], pos_robot[1], pos_robot[2]]

# ArUco detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
image_marked, point3d = cam.get_board_pose(aruco_dict)

# if point3d is not None:
#     point3d = point3d.to_array()
#     point3d = np.array([*point3d, 1])  # make it 1x4

    # Transformation matrix (example values)
    # best_matrix = np.array([
    #     [ 0.01264135, -1.01779307,  0.229349,    0.49121051],
    #     [ 0.08439283,  0.0429427,   1.09825977, -0.54507379],
    #     [-0.97115769,  0.03103773, -0.02858476, -0.01855436],
    #     [ 0.0,         0.0,         0.0,         1.0]
    # ])

#     best_matrix = np.array([[-0.68457803, -1.65106014, -0.51678327,  0.75048569],
#                             [ 0.03388109,  0.03492339,  0.02919003,  0.17697453],
#                             [-1.12146604, -0.15064356, -0.45164393,  0.21332539],
#                             [ 0.0,         0.0,         0.0,         1.0]])

#     # Apply transformation
#     pos_3d_final = best_matrix @ point3d

#     print("Robot Position:", pos_robot_3d)
#     print("Detected Point3D:", point3d)
#     print("Transformed Point3D:", pos_3d_final)
# else:
#     print("No board detected.")

# Show image if available
if image_marked is not None:
    cv2.imshow("Detected Board", image_marked)
    cv2.waitKey(10000)
    cv2.destroyAllWindows()

cam.stop()
