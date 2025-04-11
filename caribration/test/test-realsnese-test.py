import cv2
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from classrobot import realsense_cam
from classrobot import robot_movement

# Init cam 
cam = realsense_cam.RealsenseCam()
robot = robot_movement.RobotControl()

# ArUco detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
image_marked, point3d = cam.get_board_pose(aruco_dict)
print("3D Points:", point3d)
# point3d_world = robot.convert_cam_to_world(point3d.to_list())
# print("3D Points in World Reference:", point3d_world)


# Show image if available
if image_marked is not None:
    cv2.imshow("Detected Board", image_marked)
    cv2.waitKey(10000)
    cv2.destroyAllWindows()

cam.stop()
