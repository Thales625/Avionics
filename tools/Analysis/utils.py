import numpy as np

def skew(v: np.ndarray) -> np.ndarray:
    """3x3 skew-symmetric matrix for cross-product."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=float)

def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product. q = [x,y,z,w] (vector first, scalar last)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x, y, z, w], dtype=float)

def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)

def quat_from_small_angle(theta: np.ndarray) -> np.ndarray:
    """Convert small-angle vector (3,) to quaternion (x,y,z,w)."""
    angle = np.linalg.norm(theta)
    if angle < 1e-12:
        # small-angle approximation
        return np.array([theta[0]/2.0, theta[1]/2.0, theta[2]/2.0, 1.0], dtype=float)
    axis = theta / angle
    s = np.sin(angle/2.0)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, np.cos(angle/2.0)], dtype=float)

def rot_from_quat(q: np.ndarray) -> np.ndarray:
    """Rotation matrix from quaternion (x,y,z,w) mapping body->nav if we use that convention."""
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y*y+z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x+z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x+y*y)]
    ], dtype=float)
    return R


from scipy.spatial.transform import Rotation as R

def quaternion_to_euler(quat):
    return R.from_quat(quat).as_euler('xyz', degrees=True)