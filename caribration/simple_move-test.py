import time
from classrobot import robot_movement
import roboticstoolbox as rtb
from roboticstoolbox import RevoluteDH, DHRobot
from spatialmath import SE3
import numpy as np

def main():
        # Link 1
    L1 = RevoluteDH(
        d=0.1625,
        a=0,
        alpha=np.pi/2,
        m=3.761,
        r=[0, -0.02561, 0.00193],
        I=np.zeros((3, 3))
    )

    # Link 2
    L2 = RevoluteDH(
        d=0,
        a=-0.425,
        alpha=0,
        m=8.058,
        r=[0.2125, 0, 0.11336],
        I=np.zeros((3, 3))
    )

    # Link 3
    L3 = RevoluteDH(
        d=0,
        a=-0.3922,
        alpha=0,
        m=2.846,
        r=[0.15, 0, 0.0265],
        I=np.zeros((3, 3))
    )

    # Link 4
    L4 = RevoluteDH(
        d=0.1333,
        a=0,
        alpha=np.pi/2,
        m=1.37,
        r=[0, -0.0018, 0.01634],
        I=np.zeros((3, 3))
    )

    # Link 5
    L5 = RevoluteDH(
        d=0.0997,
        a=0,
        alpha=-np.pi/2,
        m=1.3,
        r=[0, 0.0018, 0.01634],
        I=np.zeros((3, 3))
    )

    # Link 6
    I6 = np.array([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0.0002]
    ])
    L6 = RevoluteDH(
        d=0.0996,
        a=0,
        alpha=0,
        m=0.365,
        r=[0, 0, -0.001159],
        I=I6
    )

    # Create the UR5e robot model
    UR5e = DHRobot([L1, L2, L3, L4, L5, L6], name='UR5e')
    tool_offset = SE3(0, 0, 0.2)
    UR5e.tool = tool_offset  # Incorporate this offset into forward kinematics
    # Replace with your robot's IP address
    robot_ip = "192.168.200.10"
    robot = robot_movement.RobotControl()
    robot.robot_init(robot_ip)
    pos_left = robot.robot_get_position()
    tcp_offset = robot.robot_get_TCP_offset()
    pos_left_avatar	= robot.my_convert_position_from_left_to_avatar(pos_left)
    inv = robot.robot_get_ik(pos_left)
    joint = robot.robot_get_joint_rad()

    T_fk = UR5e.fkine(joint)
    T_ik = UR5e.ik_LM(T_fk)
    # print(T_ik)
    
    # print("Position in left reference frame:", pos_left)
    # print("Inverse Kinematics:", inv)
    # print("Joint Positions:", joint)

    print("Position in left reference frame:", pos_left)
    # print("TCP offset:", tcp_offset)
    # print("Position in avatar reference frame:", pos_left_avatar)


    print(tcp_offset)
    pos_left[1]+=0.01
    print(pos_left)
    robot.robot_moveL(pos_left,0.01)


if __name__ == "__main__":
    main()
