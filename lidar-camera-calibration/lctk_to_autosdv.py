"""
LiDAR-to-camera calibration converter for AutoSDV.

Converts LCTK LiDAR->camera-optical extrinsics (transform.rvec / transform.tvec
in detections_N.json) into camera mounting-link poses relative to
sensor_kit_base_link, for N = 6, 4, 2, 1.

Dependencies:
    Python 3, numpy, scipy, PyYAML
    sudo apt install python3-numpy python3-scipy python3-yaml

Inputs:
    --detections-dir  Directory containing detections_6.json, detections_4.json,
                      detections_2.json and detections_1.json (LCTK archives).
    --yaml            AutoSDV sensor_kit_calibration.yaml; the existing LiDAR
                      pose is read from its sensor_kit_base_link mapping.
    --lidar-link      LiDAR entry in that YAML: robin_w (default) or vlp16.

Output:
    YAML printed to stdout: lab1_t{6,4,2,1}_camera_link entries with x, y, z
    (metres) and roll, pitch, yaw (radians), to paste under sensor_kit_base_link.

Usage:
    python3 lctk_to_autosdv.py \\
        --detections-dir ~/LCTK/lab1/detections \\
        --yaml ~/AutoSDV/src/param/autoware_individual_params/individual_params/config/default/autosdv_sensor_kit/sensor_kit_calibration.yaml \\
        --lidar-link robin_w
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

ZED_LINK_FROM_OPTICAL = (-0.010, 0.025, 0.016, -math.pi / 2, 0.0, -math.pi / 2)
POSE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def pose_to_transform(x, y, z, roll, pitch, yaw):
    """Convert a 6-DOF pose (x, y, z, roll, pitch, yaw) to a 4x4 transform matrix."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def transform_to_pose(T):
    """Convert a 4x4 transform matrix to a 6-DOF pose tuple."""
    x, y, z = T[:3, 3]
    roll, pitch, yaw = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
    return x, y, z, roll, pitch, yaw


def camera_link_pose(T_base_from_lidar, rvec, tvec):
    """
    Compute camera link pose in sensor kit base frame.

    Args:
        T_base_from_lidar: 4x4 transform mapping LiDAR coordinates into the base frame.
        rvec: Rodrigues rotation vector (radians), mapping LiDAR into the camera optical frame.
        tvec: Translation vector (metres), mapping LiDAR into the camera optical frame.

    Returns:
        6-DOF pose (x, y, z, roll, pitch, yaw) of camera link in base frame.
    """
    T_optical_from_lidar = np.eye(4)
    T_optical_from_lidar[:3, :3] = Rotation.from_rotvec(rvec).as_matrix()
    T_optical_from_lidar[:3, 3] = tvec

    T_link_from_optical = pose_to_transform(*ZED_LINK_FROM_OPTICAL)

    T_base_from_link = (
        T_base_from_lidar
        @ np.linalg.inv(T_optical_from_lidar)
        @ np.linalg.inv(T_link_from_optical)
    )
    return transform_to_pose(T_base_from_link)


def load_detection(detections_dir, n):
    """Load calibration transform from a detection JSON file."""
    path = Path(detections_dir) / f"detections_{n}.json"
    with open(path) as f:
        return json.load(f)["transform"]


def main():
    """Parse arguments, compute camera poses, and output YAML calibration."""
    parser = argparse.ArgumentParser(description="Compute camera link poses from LiDAR calibration.")
    parser.add_argument("--detections-dir", required=True, help="Directory with detection JSON files")
    parser.add_argument("--yaml", required=True, help="Sensor calibration YAML file")
    parser.add_argument("--lidar-link", default="robin_w", help="LiDAR link name in YAML")
    args = parser.parse_args()

    with open(args.yaml) as f:
        calibration = yaml.safe_load(f)

    lidar_pose = calibration["sensor_kit_base_link"][args.lidar_link]
    T_base_from_lidar = pose_to_transform(*[float(lidar_pose[k]) for k in POSE_KEYS])

    results = {}
    for n in (6, 4, 2, 1):
        detection = load_detection(args.detections_dir, n)
        pose = camera_link_pose(T_base_from_lidar, detection["rvec"], detection["tvec"])

        if not np.isfinite(pose).all():
            raise ValueError(f"detections_{n}.json: non-finite pose")

        results[f"lab1_t{n}_camera_link"] = dict(zip(POSE_KEYS, (np.round(pose, 6) + 0.0).tolist()))

    print(yaml.safe_dump({"sensor_kit_base_link": results}, sort_keys=False))

if __name__ == "__main__":
    main()