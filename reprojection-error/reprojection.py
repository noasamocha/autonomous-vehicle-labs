"""Student-owned math for fixed-transform corner reprojection."""

import numpy as np


def transform_points(
    points_lidar: np.ndarray, rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    """Map LiDAR-frame points to camera optical coordinates: p_C = R p_L + t."""
    points = np.asarray(points_lidar, dtype=np.float64).reshape(-1, 3)
    R = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    t = np.asarray(translation, dtype=np.float64).reshape(3)
    # Row-vector form of p_C = R p_L + t for every point at once.
    return points @ R.T + t


def project_points(points_camera: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    """Project camera points with K and divide by homogeneous depth w_h."""
    points = np.asarray(points_camera, dtype=np.float64).reshape(-1, 3)
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    homogeneous = points @ K.T  # (u_h, v_h, w_h) per row
    return homogeneous[:, :2] / homogeneous[:, 2:3]


def pixel_errors(
    projected_pixels: np.ndarray, observed_pixels: np.ndarray
) -> np.ndarray:
    """Euclidean distance in pixels between each projected and observed corner."""
    projected = np.asarray(projected_pixels, dtype=np.float64).reshape(-1, 2)
    observed = np.asarray(observed_pixels, dtype=np.float64).reshape(-1, 2)
    return np.linalg.norm(projected - observed, axis=1)


def rms_error(errors: np.ndarray) -> float:
    """Pooled RMS over all matched corners: sqrt(sum(e_i^2) / N)."""
    values = np.asarray(errors, dtype=np.float64).ravel()
    return float(np.sqrt(np.mean(values ** 2)))
