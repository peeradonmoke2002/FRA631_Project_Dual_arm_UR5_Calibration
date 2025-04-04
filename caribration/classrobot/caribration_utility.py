import cv2
import numpy as np
import random
import math
from typing import List, Tuple, Optional
from .point3d import Point3D
from .calibrationdata import CalibrationData


class Calibrator:
    def __init__(self, calibration_data_list: List[CalibrationData]):
        self.calibration_data_list = calibration_data_list

    def estimate_homogeneous_affine_3d(self,
                                       source_points: List[Point3D],
                                       target_points: List[Point3D],
                                       ransac_threshold: float,
                                       confidence: float) -> Optional[np.ndarray]:
        """
        Estimate a 3D affine transformation from source_points to target_points.
        Returns a 4x4 homogeneous transformation matrix if successful, else None.
        """
        if source_points is None or target_points is None or (len(source_points) != len(target_points)):
            raise ValueError("Invalid input point lists.")

        # Convert list of Point3D to numpy arrays of shape (N, 3)
        src_array = np.array([pt.to_array() for pt in source_points], dtype=np.float32)
        dst_array = np.array([pt.to_array() for pt in target_points], dtype=np.float32)

        retval, affine_matrix, inliers = cv2.estimateAffine3D(src_array, dst_array,
                                                              ransacThreshold=ransac_threshold,
                                                              confidence=confidence)
        if not retval:
            return None

        # Convert the 3x4 affine matrix to a 4x4 homogeneous transformation matrix.
        homo_matrix = np.zeros((4, 4), dtype=np.float64)
        homo_matrix[:3, :4] = affine_matrix.astype(np.float64)
        homo_matrix[3, 3] = 1.0
        return homo_matrix

    def apply_affine_transformation(self,
                                    points: List[Point3D],
                                    transformation_matrix: np.ndarray) -> List[Point3D]:
        """
        Apply a 4x4 transformation to each 3D point.
        Returns a new list of transformed Point3D.
        """
        transformed_points = []
        for pt in points:
            # Convert point to homogeneous coordinates.
            homo_pt = np.array([pt.x, pt.y, pt.z, 1.0], dtype=np.float64)
            transformed_homo = transformation_matrix @ homo_pt
            transformed_points.append(Point3D(transformed_homo[0],
                                               transformed_homo[1],
                                               transformed_homo[2]))
        return transformed_points

    def cal_mean_square_error_xyz(self,
                                  target_points: List[Point3D],
                                  transformed_points: List[Point3D]) -> List[float]:
        """
        Calculate the RMSE in X, Y, Z and overall RMS error between two point sets.
        Returns [rms_error_x, rms_error_y, rms_error_z, overall_rms_error].
        """
        if len(target_points) != len(transformed_points):
            raise ValueError("Input point lists must have the same length.")
        
        num_points = len(target_points)
        error_x = 0.0
        error_y = 0.0
        error_z = 0.0

        for tgt, trans in zip(target_points, transformed_points):
            error_x += (tgt.x - trans.x) ** 2
            error_y += (tgt.y - trans.y) ** 2
            error_z += (tgt.z - trans.z) ** 2

        rms_error_x = math.sqrt(error_x / num_points)
        rms_error_y = math.sqrt(error_y / num_points)
        rms_error_z = math.sqrt(error_z / num_points)
        overall_rms = (rms_error_x + rms_error_y + rms_error_z) / 3.0

        return [rms_error_x, rms_error_y, rms_error_z, overall_rms]

    def random_selection(self, num_selected_positions: int) -> List[int]:
        """
        Randomly select a given number of unique indices from the calibration data.
        """
        if num_selected_positions <= 0 or num_selected_positions > len(self.calibration_data_list):
            raise ValueError("NUmber set point is not same as data input")
        return random.sample(range(len(self.calibration_data_list)), num_selected_positions)

    def extract_points(self, selected_positions: List[int]) -> Tuple[List[Point3D], List[Point3D]]:
        """
        Extract the CCS and AC points from the calibration data for the given indices.
        Returns a tuple: (list_of_CCS_points, list_of_AC_points).
        """
        ccs_points = []
        ac_points = []
        for idx in selected_positions:
            if idx < 0 or idx >= len(self.calibration_data_list):
                raise ValueError("Invalid selected position index.")
            data = self.calibration_data_list[idx]
            ccs_points.append(data.CCS)
            ac_points.append(data.AC)
        return ccs_points, ac_points

    def find_best_matrix(self,
                         num_selected_positions: int,
                         num_iterations: int,
                         target_rms_error: float) -> Tuple[Optional[np.ndarray],
                                                             float,
                                                             List[float],
                                                             List[int],
                                                             List[Point3D]]:
        """
        Iteratively search for the best transformation matrix.
        Returns a tuple:
            (best_transformation_matrix, best_rms_error, rms_errors, selected_positions, best_transformed_points)
        """
        best_rms_error = float('inf')
        best_transformation_matrix = None
        best_transformed_points = []
        rms_errors = []
        selected_positions = []
        iterations = 0

        while iterations < num_iterations and best_rms_error > target_rms_error:
            selected_positions = self.random_selection(num_selected_positions)
            CCS_points, AC_points = self.extract_points(selected_positions)

            # Random values for ransac threshold [1, 3] and confidence [0.8, 0.99]
            ransac_threshold = 1 + (2 * random.random())
            confidence = 0.8 + (0.19 * random.random())

            transformation_matrix = self.estimate_homogeneous_affine_3d(CCS_points, AC_points,
                                                                        ransac_threshold, confidence)
            if transformation_matrix is None:
                iterations += 1
                continue

            transformed_points = self.apply_affine_transformation(CCS_points, transformation_matrix)
            current_rms_errors = self.cal_mean_square_error_xyz(AC_points, transformed_points)
            overall_rms = current_rms_errors[3]

            if overall_rms < best_rms_error:
                best_rms_error = overall_rms
                best_transformation_matrix = transformation_matrix
                best_transformed_points = transformed_points
                rms_errors = current_rms_errors

            iterations += 1
            # Optional: print iteration details for debugging.
            print(f"Iteration {iterations}: Best Overall RMS = {best_rms_error}")

        return best_transformation_matrix, best_rms_error, rms_errors, selected_positions, best_transformed_points

