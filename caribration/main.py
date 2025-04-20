# main.py
import time
import cv2 as cv
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))
from classrobot import robot_movement, realsense_cam
import os

class calibrationUR5e():
    def __init__(self):
        # End effector home position (6 DOF) and other test positions
        self.HOME_POS = [0.701172053107018, 0.184272460738082, 0.1721568294843568, -1.7318488600590023, 0.686830145115122, -1.731258978679887]
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
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
        image_marked, point3d = self.cam.get_board_pose(aruco_dict)
        print("Camera measurement:", point3d)
        if image_marked is not None:
            # Display the image with detected board
            cv.imshow("Detected Board", image_marked)
            cv.waitKey(5000)
            cv.destroyAllWindows()

            # Save the image to a file with a unique name
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(os.path.dirname(__file__), "images", f"detected_board_{timestamp}.png")
            cv.imwrite(save_path, image_marked)
            print(f"Marked image saved to {save_path}")
        return point3d
    
    

    def get_robot_TCP(self):
        """
        Connects to the robot and retrieves the current TCP (end-effector) position.
        Returns a 3-element list: [x, y, z].
        """
        pos = self.robot.robot_get_position()
        pos_3d = self.robot.convert_gripper_to_maker(pos)
        print("Robot TCP position:", pos_3d)
        return pos_3d

    def move_home(self):
        print("Moving to home position...")
        self.robot.robot_moveL(self.HOME_POS, self.speed)


    def moveL_square(self):
        """
        Move the robot through a 3x3 grid defined by 4 reference points that form a square,
        and repeat this pattern over 3 vertical planes (each separated by 10 cm).
        
        The grid is computed by bilinear interpolation on the x-z plane (y is set from the reference)
        using the following real reference positions (assumed in robot coordinates):
        
        pos_home (reference home) = [0.7011797304915488, 0.18427154391614353, 0.17217411213036665]
        BL = [0.6391839708261646, 0.18426921633782534, 0.3510680386492011]
        TL = [0.9034970156209872, 0.18431919874933683, 0.3510680386492011]
        TR = [0.9035034184486379, 0.18425659123476879, -0.43708867396716417]
        BR = [0.6158402179629584, 0.18424774957164802, -0.4371210612556637]
        
        Here, the reference points are used only for interpolation on x and z.
        The fixed orientation (RPY) is defined as:
            RPY = [-1.7318443587261685, 0.686842056802218, -1.7312759524010408]
        
        For the grid:
        - u (horizontal parameter) varies from 0 (left) to 1 (right)
        - v (vertical parameter) varies from 0 (top) to 1 (bottom)
        
        The 3x3 grid (rows, cols) is laid out as follows:
            (0,0)   (0,1)   (0,2)     --> Top row
            (1,0)   (1,1)   (1,2)     --> Middle row
            (2,0)   (2,1)   (2,2)     --> Bottom row
            
        In this version, the home (starting) cell is (2,0) – the bottom‑left cell.
        
        Then, an outer loop moves the robot vertically over 3 planes. Each plane’s y coordinate
        is offset by 10 cm (0.10 m) downward relative to the previous plane.
        
        At each grid cell, the robot moves to the target position (which is given by the interpolated
        x, y, z combined with the fixed RPY), and data is collected. Only if data collection returns True
        does the sequence continue.
        """
        
        # ----- Reference data -----
        # Use the provided "home" and reference corner positions.
        pos_home = [0.7011797304915488, 0.18427154391614353, 0.17217411213036665]
        BL_ref = [0.6158402179629584, 0.18426921633782534, 0.3510680386492011]
        TL_ref = [0.9034970156209872, 0.18431919874933683, 0.3510680386492011]
        TR_ref = [0.9035034184486379, 0.18425659123476879, -0.43708867396716417]
        BR_ref = [0.6158402179629584, 0.18424774957164802, -0.43708867396716417]
        RPY = [-1.7318443587261685, 0.686842056802218, -1.7312759524010408]
        
        # For grid interpolation, we use the four corners.
        # Here, interpret the corners for x and z interpolation:
        # Let’s assume:
        #   TL for top-left, TR for top-right, BL for bottom-left, and BR for bottom-right.
        # In our given numbers, note that the y values are nearly identical; so we will fix y to the average.
        avg_y = (TL_ref[1] + TR_ref[1] + BL_ref[1] + BR_ref[1]) / 4.0
        
        # ----- Build a 3x3 grid on the x-z plane via bilinear interpolation -----
        grid = []  # grid[i][j] corresponds to position at row i, col j.
        for i in range(3):
            row_positions = []
            v = i / 2.0  # v = 0.0, 0.5, 1.0 for rows 0,1,2
            for j in range(3):
                u = j / 2.0  # u = 0.0, 0.5, 1.0 for columns 0,1,2
                # Bilinear interpolation: x and z; y is fixed to avg_y.
                x = (1 - u) * (1 - v) * TL_ref[0] + u * (1 - v) * TR_ref[0] + (1 - u) * v * BL_ref[0] + u * v * BR_ref[0]
                z = (1 - u) * (1 - v) * TL_ref[2] + u * (1 - v) * TR_ref[2] + (1 - u) * v * BL_ref[2] + u * v * BR_ref[2]
                row_positions.append([x, avg_y, z])
            grid.append(row_positions)
        
        # ----- Define move order for the grid (for each plane) -----
        # We want home at bottom-left; thus grid cell (2,0) is home.
        # Define move order in a clockwise spiral:
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
        
        # ----- Outer loop: iterate over 3 vertical planes -----
        # For each plane, we adjust the y coordinate relative to the original grid.
        vertical_distance = 0.10  # 10 cm per plane
        num_planes = 3
        
        # Connect to the robot.
        # robot = robot_movement.RobotControl()
        # robot.robot_init(self.robot_ip)
        
        # For state numbering, we can use: state = plane_idx * 10 + grid_index + 1
        for plane_idx in range(num_planes):
            # For each plane, adjust the grid: the new y becomes avg_y - (plane_idx * vertical_distance)
            plane_offset_y = avg_y - (plane_idx * vertical_distance)
            print(f"--- Moving on plane {plane_idx+1} with y = {plane_offset_y:.4f} ---")
            
            # For this plane, update the grid positions (only y changes)
            plane_grid = []
            for i in range(3):
                row_positions = []
                for j in range(3):
                    pos = grid[i][j].copy()
                    pos[1] = plane_offset_y
                    row_positions.append(pos)
                plane_grid.append(row_positions)
            
            # Iterate over the move order for this plane.
            for idx, (i, j) in enumerate(move_order):
                pos = plane_grid[i][j]
                # Combine position and fixed orientation (RPY) into the target.
                target = pos + RPY
                state = plane_idx * 10 + idx + 1
                print(f"Moving to plane {plane_idx+1} grid cell ({i},{j}) - Target: {target}")
                self.robot.robot_moveL(target, self.speed)
                time.sleep(3)  # Pause for the robot to settle
                
                if self.collect_data(state=state):
                    print(f"Data collection successful for state {state}.")
                else:
                    print(f"Data collection failed for state {state}. Halting sequence.")
                    # self.robot.robot_release()
                    return




    def collect_data(self, state: int) -> bool:
        """
        Collect calibration data at the current robot state.
        This function collects:
           - the board pose from the camera (ccs: camera coordinate system)
           - the robot TCP (ac: actual coordinate, i.e., end-effector)
        It then appends a row to a CSV file in the format:
           Pos, ccs_x, ccs_y, ccs_z, ac_x, ac_y, ac_z
        Parameters:
            state (int): A state number or position index.
        Returns:
            bool: True if data collection is successful.
        """
        filename = os.path.join(os.path.dirname(__file__), "data", "calibration_data.csv")
        print(f"Collecting data for state {state} ...")


        # data collection ------
        # ccs = camera coordinate system (camera pose)
        # ac = actual coordinate system (robot TCP pose)
        # Get camera measurement (ccs)
        ccs = self.cam_relasense()  # This should return a Point3D object
        ccs = ccs.to_list()
        # Get robot TCP (ac)
        ac = self.get_robot_TCP()  # This returns a list [x, y, z] maker
        # -- end data collection ------


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
