from __future__ import annotations
from typing import Tuple
import numpy as np

def generate_circle(t:int = 50, loops: int = 3, radius: float = 0.85, z: float = 0.7, rounding: int = 2) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    # 円軌道の(X, Y, Z)を返す．
    n = t * loops
    theta = np.linspace(0, 2.0 * np.pi * loops, n, endpoint= False)
    
    x = radius*np.cos(theta)
    y = radius*np.sin(theta)
    z_arr = np.full(n, z, dtype=np.float64)
    
    

    return x, y, z_arr