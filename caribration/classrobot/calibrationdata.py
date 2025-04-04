from dataclasses import dataclass
from typing import Optional
from .point3d import Point3D

@dataclass
class CalibrationData:
    pos: float
    CCS: Point3D  # Camera Coordinate System
    AC: Point3D   # Actual Coordinate (robot)
    GS: Optional[Point3D] = None  # estimated Actual Coordinate (robot)

    def __repr__(self) -> str:
        return (f"CalibrationData(pos={self.pos}, "
                f"CCS={self.CCS}, AC={self.AC}, GS={self.GS})")
