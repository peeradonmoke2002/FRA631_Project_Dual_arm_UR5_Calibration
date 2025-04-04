import numpy as np
from typing import Optional, List, Union
import numpy as np

class Point3D(object):
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z
    

    def to_array(self) -> np.ndarray:
        """Return the point as a numpy array [x, y, z]."""
        return np.array([self.x, self.y, self.z], dtype=np.float32)
    
    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z]

    def __repr__(self):
        return f"Points3D({self.x}, {self.y}, {self.z})"

