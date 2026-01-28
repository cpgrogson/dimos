import numpy as np


# =============================================================================
# Quaternion operations (vendored from FALCON/sim2real/utils/math.py)
# =============================================================================


def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q_w = q[:, 0]
    q_vec = q[:, 1:]
    a = v * (2.0 * q_w**2 - 1.0)[:, np.newaxis]
    b = np.cross(q_vec, v) * q_w[:, np.newaxis] * 2.0
    dot_product = np.sum(q_vec * v, axis=1, keepdims=True)
    c = q_vec * dot_product * 2.0
    return a - b + c


def quat_rotate_inverse_numpy(q, v):
    """Alias for quat_rotate_inverse for backward compatibility."""

    return quat_rotate_inverse(q, v)


def rpy_to_quat(rpy):
    """Convert roll, pitch, yaw (radians) to quaternion [w,x,y,z], ZYX order."""

    roll, pitch, yaw = rpy
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


