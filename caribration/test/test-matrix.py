import cv2
import sys
import pathlib
import numpy as np
from typing import List
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from classrobot import realsense_cam
from classrobot import robot_movement
from classrobot.point3d import Point3D
import json


def cal_error_xyz(target: Point3D, transformed: Point3D) -> List[float]:
    """
    Calculate the absolute error in X, Y, Z (and average error) between two 3D points.
    
    Parameters:
        target (Point3D): The target (expected) 3D point.
        transformed (Point3D): The measured 3D point.
    
    Returns:
        List[float]: A list in the format [error_x, error_y, error_z, overall_error],
                     where overall_error is the average of the three errors.
    """
    error_x = abs(target.x - transformed.x)
    error_y = abs(target.y - transformed.y)
    error_z = abs(target.z - transformed.z)
    
    overall_error = (error_x + error_y + error_z) / 3.0
    
    return [error_x, error_y, error_z, overall_error]

robot_ip = "192.168.200.10"
robot = robot_movement.RobotControl()
robot.robot_init(robot_ip)


# Init cam 
cam = realsense_cam.RealsenseCam()


# ArUco detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
image_marked, ccs = cam.get_board_pose(aruco_dict)
point3d = ccs.to_list()
# point3d = robot.my_transform_position_to_world_ref(point3d)

# best_matrix = np.array([[ 5.18471136e-02, -9.98244336e-01,  6.88817547e-01, -1.16411659e-01],
#                         [ 3.35885879e-03,  1.63514282e-04,  3.81926098e-02,  3.32313474e-02],
#                         [-1.09828480e+00,  4.10440377e-03, -1.22030291e+00, -1.96968064e-02],
#                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
# best_matrix = np.array([[ 0.00577452, -0.99848631,  0.01241998, -0.08242943],
#                         [ 0.1508613,   0.02166607,  1.01799482, -0.076001  ],
#                         [-0.98476372,  0.00648918,  0.07938951, -0.101862  ],
#                         [ 0.0,         0.0,         0.0,         1.0       ]])
# best_matrix = np.array([[ 0.00465481, -0.99626089,  0.01972409, -0.08235386],
#                         [ 0.1348973,   0.009654,    1.02464258, -0.06076352],
#                         [-0.98427042,  0.00608958,  0.08377368, -0.10207071],
#                         [ 0.0,         0.0,         0.0,         1.0       ]])
# best_matrix = np.array([[ 0.0091902,   0.98683012, -0.08573875,  0.69742562],
#                         [ 0.01638979, -0.11234373, -1.02432666, -1.19305022],
#                         [-0.99304297,  0.00872967, -0.01697439, -0.00232298],
#                         [ 0.0,         0.0,         0.0,         1.0       ]])


config_path = pathlib.Path(__file__).parent.parent / "config" / "best_matrix.json"
with open(config_path, 'r') as f:
    loaded_data = json.load(f)
    name = loaded_data["name"]
    best_matrix = np.array(loaded_data["matrix"])

print("Name:", name)
print("Matrix:\n", best_matrix)
point3d_array = np.array([point3d[0], point3d[1], point3d[2]])
point3d_array = np.append(point3d_array, 1.0)  # Convert to homogeneous coordinates
final_point = best_matrix @ point3d_array
final_point = [final_point[0], final_point[1], final_point[2]]
print("Transformed point:", final_point[:3])

pos_left = robot.robot_get_position()
pos_left = [pos_left[0]+0.18, pos_left[1]+0.18, pos_left[2]]
# pos_left = robot.my_convert_position_from_left_to_avatar(pos_left)

print("Robot Position:", pos_left[:3])

final_point = Point3D(final_point[0], final_point[1], final_point[2])
robot_position = Point3D(pos_left[0], pos_left[1], pos_left[2])
rms_error = cal_error_xyz(final_point, robot_position)
print("RMS Error X:", rms_error[0])
print("RMS Error Y:", rms_error[1]) 
print("RMS Error Z:", rms_error[2])
print("Overall RMS Error:", rms_error[3])

# final_point_robot_real = robot.my_convert_position_from_avatar_to_left([final_point.x, final_point.y, final_point.z])
# print("Final Point in Robot Left Ref:", final_point_robot_real[:3])
if image_marked is not None:
    cv2.imshow("Detected Board", image_marked)
    cv2.waitKey(10000)
    cv2.destroyAllWindows()

cam.stop()
