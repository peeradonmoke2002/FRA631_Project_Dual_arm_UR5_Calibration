import cv2
import numpy as np
import random
import math
from .point3d import Point3D
from typing import Optional, List

class CalibrationData:
    def __init__(self, pos: float, CCS: Point3D, AC: Point3D, GS: Optional[Point3D]=None):
        self.pos = pos
        self.CCS = CCS  # Camera coordinate system point
        self.AC = AC    # Actual (robot) coordinate point
        self.GS = GS

    def __repr__(self):
        return f"CalibrationData(pos={self.pos}, CCS={self.CCS}, AC={self.AC})"
