from classrobot import robot_movement

def main():

    robot_ip = "192.168.200.10"
    robot = robot_movement.RobotControl()
    robot.robot_init(robot_ip)
    pos_left = robot.robot_get_position()
    print("Robot Position:", pos_left)


if __name__ == "__main__":
    main()
