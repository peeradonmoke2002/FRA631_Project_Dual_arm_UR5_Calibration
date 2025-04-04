# main.py
import time
import cv2 as cv
from classrobot import robot_movement, realsense_cam, point3d
import os

class calibrationUR5e():
    def __init__(self):
        # End effector home position (6 DOF) and other test positions
        self.HOME_POS = [0.701172053107018, 0.184272460738082, 0.1721568294843568, -1.7318488600590023, 0.686830145115122, -1.731258978679887]
        self.TEST = [0.801172053107018, 0.084272460738082, 0.1721568294843568, -1.7318488600590023, 0.686830145115122, -1.731258978679887]
        self.TEST2 = [0.8011561071792352, 0.08430097586065964, 0.1721584901349059, -1.73183931957943, 0.6868204659244422, -1.7312337199195151]
        self.HOME_POS2 = [25.35, -124.37, -95, -139.88, -66, 135.7]
        self.robot_ip = "192.168.200.10"
        self.speed = 0.1
        self.acceleration = 1.2
        # Initialize the robot connection once.
        self.robot = robot_movement.RobotControl()
        self.robot.robot_release()
        self.robot.robot_init(self.robot_ip)

        self.cam = realsense_cam.RealsenseCam()

    def stop_all(self):
        self.robot.robot_release()
        self.cam.stop()


    def cam_relasense(self):
        """
        Launches the RealSense camera to obtain the board pose.
        Returns the board pose (camera coordinate system) as a Point3D instance.
        """

        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        image_marked, point3d = self.cam.get_board_pose(aruco_dict)
        print("Camera measurement:", point3d)
        if image_marked is not None:
            cv.imshow("Detected Board", image_marked)
            cv.waitKey(5000)
            cv.destroyAllWindows()
        return point3d
    
    

    def get_robot_TCP(self):
        """
        Connects to the robot and retrieves the current TCP (end-effector) position.
        Returns a 3-element list: [x, y, z].
        """
        pos = self.robot.robot_get_position()
        # Extract x, y, z (assumed to be in indices 0, 1, 2).
        pos_3d = [pos[0]+0.18, pos[1]+0.18, pos[2]]
        print("Robot TCP position:", pos_3d)
        return pos_3d

    def move_home(self):
        print("Moving to home position...")
        self.robot.robot_moveL(self.HOME_POS, self.speed)


    def moveL_square(self):
        """
        Move the robot through a 3x3 grid defined by 4 reference points that form a perfect square.
        

        The grid is computed via bilinear interpolation on the x-z plane (y remains constant).
        The grid layout is as follows (row, col):
            (0,0)   (0,1)   (0,2)     --> Top row
            (1,0)   (1,1)   (1,2)     --> Middle row
            (2,0)   (2,1)   (2,2)     --> Bottom row
            
        In this updated version, the home (starting) position is set to the bottom-left cell, (2,0).
        The move order (clockwise spiral starting at (2,0)) is defined as:
            (2,0), (2,1), (2,2), (1,2), (0,2), (0,1), (0,0), (1,0), (1,1)
            
        At each grid cell, data is collected before proceeding.
        """

        #--------Test positions for the grid corners (TL, TR, BL, BR)--------
        # TL = [0.6379, 0.1841, 0.3519]
        # TR = [0.9326, 0.1841, 0.3519]
        # BL = [0.6379, 0.1841, 0.0573]
        # BR = [0.9326, 0.1841, 0.0573]
        # TL = [0.9322103843777579, 0.18395298037313315, 0.35192282556710097]
        # TR = [0.6379185573572154, 0.18395298037313315, 0.35192282556710097]
        # BL = [0.6379046407898854, 0.18395298037313315, -0.4311363888217942]
        # BR = [0.9329119096579548, 0.18395298037313315, -0.4310893355873706]
        #-------------------------
        #--------- Real positions for the grid corners (TL, TR, BL, BR)---------
        pos_home = [0.7011797304915488, 0.18427154391614353, 0.17217411213036665]
        BL = [0.6391839708261646, 0.18426921633782534, 0.3510680386492011]
        TL = [0.9034970156209872, 0.18431919874933683, 0.3510680386492011]
        TR = [0.9035034184486379, 0.18425659123476879, -0.43708867396716417]
        BR = [0.6158402179629584, 0.18424774957164802, -0.4371210612556637]
        # Fixed orientation (RPY) for the robot's end-effector.
        RPY = [-1.7318443587261685, 0.686842056802218, -1.7312759524010408]
        #-----------------------------------------------------------
        # Build a 3x3 grid using bilinear interpolation.
        # u (horizontal) varies from 0 (left) to 1 (right)
        # v (vertical) varies from 0 (top) to 1 (bottom)
        grid = []  # grid[i][j] will be the position at row i, col j.
        for i in range(3):
            row_positions = []
            v = i / 2.0  # 0.0, 0.5, 1.0 for rows 0, 1, 2
            for j in range(3):
                u = j / 2.0  # 0.0, 0.5, 1.0 for columns 0, 1, 2
                x = (1 - u) * (1 - v) * TL[0] + u * (1 - v) * TR[0] + (1 - u) * v * BL[0] + u * v * BR[0]
                y = (TL[1] + TR[1] + BL[1] + BR[1]) / 4.0  # constant
                z = (1 - u) * (1 - v) * TL[2] + u * (1 - v) * TR[2] + (1 - u) * v * BL[2] + u * v * BR[2]
                row_positions.append([x, y, z])
            grid.append(row_positions)

        # Define move order with home at bottom-left (grid cell (2,0)) and a clockwise spiral.
        move_order = [
            (2, 0),  # Home: bottom-left
            (2, 1),  # bottom-center
            (2, 2),  # bottom-right
            (1, 2),  # middle-right
            (0, 2),  # top-right
            (0, 1),  # top-center
            (0, 0),  # top-left
            (1, 0),  # middle-left
            (1, 1)   # center
        ]

        # Loop over each grid cell in the defined move order.
        for idx, (i, j) in enumerate(move_order):
            pos = grid[i][j]
            # Combine the position and fixed orientation (RPY) into a single target list.
            target = pos + RPY
            print(f"Moving to grid cell ({i},{j}) - Target: {target}")
            self.robot.robot_moveL(target, self.speed)
            time.sleep(3)  # Pause to allow settling

            # Define a unique state number; here simply idx+1.
            state = idx + 1
            if self.collect_data(state=state):
                print(f"Data collection successful for state {state}.")
            else:
                print(f"Data collection failed for state {state}. Halting sequence.")
                return

        


    # def moveL_square(self):
    #         """
    #         Move the robot in a square pattern covering 9 positions arranged in a 3x3 grid,
    #         repeated on 3 planes. The home position (self.HOME_POS) is defined as the bottom center cell on the top plane.
    #         After moving to each position, data is collected. Only if data collection returns True,
    #         the robot proceeds to the next position.
            
    #         Coordinate system:
    #         +x: forward, -x: backward, +z: left, -z: right, and -y: downward.
            
    #         Grid layout (row, col) for each plane:
    #             (0,0)   (0,1)   (0,2)     --> Top row (most forward)
    #             (1,0)   (1,1)   (1,2)     --> Middle row
    #             (2,0)   (2,1)   (2,2)     --> Bottom row (home is at (2,1) on the first plane)
            
    #         Offsets relative to the plane’s center (which is derived from self.HOME_POS with a modified y):
    #         offset_x = (2 - row) * dx   (dx: step size in x)
    #         offset_z = (1 - col) * dz   (dz: step size in z)
    #         Between planes, the y coordinate is offset by -0.5 m per plane.
    #         """
    #         print("Moving in a square pattern over 3 planes (home = bottom center on the top plane)...")
                
    #         dx = 0.05  # step in x (forward/backward)
    #         dz = 0.05  # step in z (left/right)
    #         dy = 0.005   # vertical offset between planes (in -y direction)
            
    #         # Use HOME_POS as the reference for the top plane.
    #         center = self.HOME_POS.copy()  # center is a list of 6 elements; index 0: x, index 1: y, index 2: z
            
    #         # Define grid cell indices (row, col) for the 3x3 square.
    #         grid_positions = [
    #             (2, 1),  # Home: Bottom center (starting position on each plane)
    #             (2, 0),  # Bottom left
    #             (2, 2),  # Bottom right
    #             (1, 0),  # Middle left
    #             (1, 1),  # Middle center
    #             (1, 2),  # Middle right
    #             (0, 0),  # Top left
    #             (0, 1),  # Top center
    #             (0, 2)   # Top right
    #         ]
            
    #         # Outer loop: iterate over 3 planes.
    #         for plane_idx in range(3):
    #             # For each plane, adjust the y coordinate by subtracting plane_idx * dy.
    #             plane_center = center.copy()
    #             plane_center[1] = center[1] - plane_idx * dy
                
    #             print(f"--- Moving in plane {plane_idx+1} (y = {plane_center[1]:.3f}) ---")
                
    #             positions = []
    #             for row, col in grid_positions:
    #                 pos = plane_center.copy()
    #                 pos[0] += (2 - row) * dx  # Adjust x: top row (row 0) gets 2*dx forward relative to home
    #                 pos[2] += (1 - col) * dz   # Adjust z: left (col 0) gets +dz; right (col 2) gets -dz
    #                 positions.append(pos)
                
    #             # Iterate over positions for this plane.
    #             for idx, pos in enumerate(positions):
    #                 print(f"Moving to plane {plane_idx+1} position {idx+1} (Grid cell {grid_positions[idx]}): {pos}")
    #                 self.robot.robot_moveL(pos, self.speed)
    #                 time.sleep(3)  # Pause to allow the robot to settle
                    
    #                 # Collect data at this position.
    #                 state = plane_idx * 10 + idx + 1  # Unique state number per position.
    #                 if self.collect_data(state=state):
    #                     print(f"Data collection successful for plane {plane_idx+1} position {idx+1}.")
    #                 else:
    #                     print(f"Data collection failed for plane {plane_idx+1} position {idx+1}. Halting sequence.")
    #                     return
        





    def collect_data(self, state: int, filename: str = "calibration_data.csv") -> bool:
        """
        Collect calibration data at the current robot state.
        This function collects:
           - the board pose from the camera (ccs: camera coordinate system)
           - the robot TCP (ac: actual coordinate, i.e., end-effector)
        It then appends a row to a CSV file in the format:
           Pos, ccs_x, ccs_y, ccs_z, ac_x, ac_y, ac_z
        Parameters:
            state (int): A state number or position index.
            filename (str): The CSV file to which data is appended.
        Returns:
            bool: True if data collection is successful.
        """
        print(f"Collecting data for state {state} ...")
        # Get camera measurement (ccs)
        ccs = self.cam_relasense()  # This should return a Point3D object
        ccs = ccs.to_list()
        ccs = self.robot.my_transform_position_to_world_ref(ccs)
        # Get robot TCP (ac)
        ac = self.get_robot_TCP()  # This returns a list [x, y, z]
        ac = self.robot.my_convert_position_from_left_to_avatar(ac)
        
        # Create a CSV row.
        data_row = [state, ccs[0], ccs[1], ccs[2], ac[0], ac[1], ac[2]]
        print("Collected data row:", data_row)
        
        # Check if file exists; if not, write header.
        file_exists = os.path.exists(filename)
        with open(filename, "a") as f:
            if not file_exists:
                f.write("Pos,ccs_x,ccs_y,ccs_z,ac_x,ac_y,ac_z\n")
            # Write data row.
            f.write(",".join(map(str, data_row)) + "\n")
        return True

def main():
    calibration = calibrationUR5e()

    calibration.move_home()
    time.sleep(3)
    calibration.moveL_square()
    time.sleep(3)
    calibration.move_home()
    calibration.stop_all()

if __name__ == "__main__":
    main()
