import cv2
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from classrobot import realsense_cam


# Init cam 
cam = realsense_cam.RealsenseCam()

# ArUco detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
image_marked, point3d = cam.get_board_pose(aruco_dict)

# Show image if available
if image_marked is not None:
    cv2.imshow("Detected Board", image_marked)
    cv2.waitKey(10000)
    cv2.destroyAllWindows()

cam.stop()
