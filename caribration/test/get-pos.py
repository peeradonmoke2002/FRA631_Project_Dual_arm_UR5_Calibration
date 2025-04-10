import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from classrobot import robot_movement

def main():

    robot_ip = "192.168.200.10"
    robot = robot_movement.RobotControl()
    robot.robot_init(robot_ip)
    pos_left = robot.robot_get_position()
    print("x,z,y")
    print("Robot Position:", pos_left[:3])
    # Convert to Avatar Reference
    pos_avatar = robot.convert_position_from_left_to_world(pos_left)
    print("Avatar Position:", pos_avatar)


if __name__ == "__main__":
    main()
