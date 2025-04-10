import rtde_control
import rtde_receive
import rtde_io
import time
import math
from typing import Optional, List
import numpy as np
from math import pi
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
from spatialmath.base import troty, trotz, trotx

class RobotControl:
    def __init__(self):
        self._ROBOT_CON_ = None
        self._ROBOT_RECV_ = None
        self._ROBOT_IO_ = None

    def robot_init(self, host: str) -> None:
        self._ROBOT_CON_ = rtde_control.RTDEControlInterface(host)
        self._ROBOT_RECV_ = rtde_receive.RTDEReceiveInterface(host)
        self._ROBOT_IO_ = rtde_io.RTDEIOInterface(host)

    def robot_release(self) -> None:
        if self._ROBOT_CON_ is not None:
            self._ROBOT_CON_.stopScript()
            self._ROBOT_CON_.disconnect()

        if self._ROBOT_RECV_ is not None:
            self._ROBOT_RECV_.disconnect()

        if self._ROBOT_IO_ is not None:
            self._ROBOT_IO_.disconnect()

    # --------------------------
    # Data Acquisition Methods
    # --------------------------
    def robot_get_joint_deg(self) -> list:
        """Return the actual joint positions in degrees."""
        res = self._ROBOT_RECV_.getActualQ()
        return [math.degrees(rad) for rad in res]

    def robot_get_joint_rad(self) -> list:
        """Return the actual joint positions in degrees."""
        res = self._ROBOT_RECV_.getActualQ()
        return res
    
    def robot_get_position(self):
        """Return the current TCP pose."""
        return self._ROBOT_RECV_.getActualTCPPose()
    
    def robot_get_TCP_offset(self):
        """Return the current TCP offset."""
        return self._ROBOT_CON_.getTCPOffset()
    
    def robot_get_fk(self, q, tcp_offset):
        """
        Return forward kinematics using provided joint positions and TCP offset.
        If either q is empty or tcp_offset is None, call the default FK.
        """
        if q and tcp_offset is not None:
            return self._ROBOT_RECV_.getForwardKinematics(q, tcp_offset)
        else:
            return self._ROBOT_RECV_.getForwardKinematics()

    def robot_get_ik(self, 
                     x: List[float], 
                     qnear: Optional[List[float]] = None, 
                     maxPositionError: Optional[float] = None, 
                     maxOrientationError: Optional[float] = None):
        return self._ROBOT_CON_.getInverseKinematics(x)

        
    # --------------------------
    # Movement Methods
    # --------------------------
    def robot_move_j(self, joint_rad=None, speed=0.01, acceleration=0.05, asynchronous=False) -> None:
        """Move robot joints to specified positions (in degrees)."""
        # if joint_degree is None:
        #     joint_degree = [0] * 6
        # joint_rad = [math.radians(deg) for deg in joint_degree]
        self._ROBOT_CON_.moveJ(q=joint_rad, speed=speed, acceleration=acceleration, asynchronous=asynchronous)

    def robot_move_jik(self, joint_rad=None, speed=0.01, acceleration=0.05, asynchronous=False) -> None:
        """Move robot joints to specified positions (in degrees)."""
        # if joint_degree is None:
        #     joint_degree = [0] * 6
        # joint_rad = [math.radians(deg) for deg in joint_degree]
        self._ROBOT_CON_.moveJ_IK(pose=joint_rad, speed=speed, acceleration=acceleration, asynchronous=asynchronous)

    def robot_move_j_stop(self, a=2.0, asynchronous=False) -> None:
        """Stop joint movement."""
        self._ROBOT_CON_.stopJ(a, asynchronous)

    def robot_move_speed(self, velocity) -> None:
        """Move robot with a linear speed."""
        self._ROBOT_CON_.speedL(xd=velocity, acceleration=0.1, time=0)

    def robot_move_speed_stop(self, acceleration=0.1) -> None:
        """Stop linear speed movement."""
        self._ROBOT_CON_.speedStop(a=acceleration)

    def robot_is_joint_move(self) -> bool:
        """Check if any joint is moving based on joint velocities."""
        res = self._ROBOT_RECV_.getActualQd()
        vel_max = max(res)
        print(f"getActualQd = {res}, vel_max = {vel_max}")
        return abs(vel_max) > 0.0001

    def robot_io_digital_set(self, id: int, signal: bool):
        """Set a digital output signal."""
        return self._ROBOT_IO_.setStandardDigitalOut(id, signal)

    def robot_moveL(self, pose: list, speed: float = 0.25, acceleration: float = 1.2, asynchronous=False) -> None:
        """Move robot linearly to the given pose."""
        self._ROBOT_CON_.moveL(pose, speed, acceleration, asynchronous=asynchronous)

    def robot_moveL_stop(self, a=10.0, asynchronous=False) -> None:
        """Stop linear movement."""
        self._ROBOT_CON_.stopL(a, asynchronous)





    def convert_position_from_left_to_world(self, position: list[float]) -> list[float]:
        """
        Convert TCP Position from Robot (Left) Reference to World Reference.
        
        Assumptions:
        - The input position is provided in the left robot's frame with the ordering [x, z, y]:
                x: forward (+x = forward, -x = backward)
                z: left/right (+z = left, -z = right)
                y: vertical (–y = up, +y = down)
        - The output is in a world coordinate frame with standard [x, y, z] order.
        
        The conversion is done in two steps:
        1. Reorder the input from [x, z, y] to [x, y, z]. (If needed, here we assume that if the
            input has more than 3 values, only the first three are used.)
        2. Apply a transformation to convert the left frame to the world frame.
            In this example:
            - A rotation Rx by –pi/2 (about the x-axis) swaps the y and z axes.
            - A translation T = SE3(0.0, 0.0575, 0.4) is applied.
            The overall transformation is defined as: transformation = T * Rx,
            and we use its inverse to go from left coordinates to world coordinates.
        
        Returns:
        A list of 3 floats representing the position in world coordinates.
        """
        # Ensure that only the first three coordinates are used.
        pos_array = np.array(position)[:3]  # Now pos_array has shape (3,)        
        # If your input is truly in [x, z, y] order, reorder it to [x, y, z]
        # Uncomment the following line if reordering is required:
        # pos_array = np.array([pos_array[0], pos_array[2], pos_array[1]])
        
        # Define the rotation about X by -pi/2 (swaps y and z)
        Rx = SE3(trotx(pi/2))
        
        # Define the translation vector
        T = SE3(0.0, 0.4, 0.0)
        
        # Compose the full transformation: from world frame to left frame
        transformation = T * Rx
        # Invert the transformation to go from left frame to world frame
        inv_transformation = transformation.inv()
        
        # Convert the 3D point to homogeneous coordinates (4x1 vector)
        pos_h = np.append(pos_array, 1)  # Now shape is (4,)
        
        # Apply the inverse transformation: multiply the 4x4 transformation matrix by the 4x1 vector
        pos_transformed = inv_transformation.A @ pos_h  # Result is a 4-element vector
        
        # Extract the [x, y, z] coordinates and return them as a list
        return pos_transformed[:3].tolist()


    

    def my_convert_position_from_left_to_avatar(self,position: list[float]) -> list[float]:
        '''
        Convert TCP Position from Robot (Left) Ref to Avatar Ref
        '''
        
        # swap axis
        res	= [-position[2], -position[1], -position[0]]

        # translation
        res[0]	-=	0.055
        res[1]	+=	0.400
        return res
    
    def convert_position_from_avatar_to_left(self, position: list[float]) -> list[float]:
        '''
        Convert TCP Position from Avatar Reference back to Robot (Left) Reference.
        This is the inverse of my_convert_position_from_left_to_avatar.
        
        Given (from the original conversion):
        Avatar[0] = -Robot[2] - 0.055
        Avatar[1] = -Robot[1] + 0.400
        Avatar[2] = -Robot[0]
        
        Then the inverse conversion is:
        Robot[0] = -Avatar[2]
        Robot[1] = 0.400 - Avatar[1]
        Robot[2] = -(Avatar[0] + 0.055)
        '''
        res = [-position[2], 0.400 - position[1], -(position[0] + 0.055)]
        return res
    
    def convert_avatar_to_world(self, position: list[float]) -> list[float]:
        '''
        Convert TCP Position from Avatar Ref to World Ref
        '''
        res = [position[0], position[1], position[2]]

        # translation
        res[1] -= 0.055
        res[2] -= 0.400

        return res
    
    def convert_world_to_avatar(self, position: list[float]) -> list[float]:
        '''
        Convert world Ref to Avatar Ref
        '''
        # swap axis z    y   x
        res = [position[0], position[1], position[2]]

        # translation
        res[0] += 0.75
        res[1] -= 0.0
        res[2] -= 1.51

        return res
    
    def convert_gripper_to_maker(self, position: list[float]) -> list[float]:
        '''
        Convert TCP Position from Gripper Ref to Marker Ref
        '''
    
        res = [position[0], position[1], position[2]]

        # translation
        res[0] += 0.18
        res[1] += 0.18

        return res

            
    def convert_cam_to_world(self, position: list[float]) -> list[float]:
        """
        Convert a camera coordinate (position vector) to a world coordinate.

        The transformation consists of:
        - A rotation about Z by -pi/2 (Rz)
        - A rotation about Y by pi (Ry)
        - A translation given by [0.75, 0.0, 1.51]

        The overall transformation is T * (Ry * Rz).

        Parameters:
        position : list of 3 floats representing the camera frame coordinates.

        Returns:
        list of 3 floats representing the position in the world coordinate frame.
        """
        # Rotation about Z by -pi/2:
        Rz = SE3(trotz(pi/2))
        Rx = SE3(trotx(pi))
        # Combined rotation: first Rz, then Ry
        R = Rx @ Rz
        # Translation vector:
        T = SE3(0.83, 0.02, 1.52)
        # Compose the complete transformation
        transformation = T * R
        # Compute final world position: the SE3 class overloads * to transform a point.
        pos_final = transformation * np.array(position)
        
        return pos_final

    

