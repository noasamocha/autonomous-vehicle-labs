# Autonomous Vehicle Labs (ROS 2 / Autoware)

> **Work in progress.** The course runs through December 2026, and this repository grows as each lab is completed and graded. See the [roadmap](#roadmap).

Lab work from a university course on autonomous vehicle software. The labs run on small autonomous cars built on AutoSDV, an open-source Autoware-based platform (ROS 2 Humble), with a Seyond Robin-W LiDAR and a ZED X Mini camera. Each lab builds on the previous one, and together they build up a working autonomous driving stack, first in simulation and then on the real car.

| Lab | What I did | Code |
| --- | --- | --- |
| [LiDAR–camera calibration](lidar-camera-calibration) | Calibrated the LiDAR-to-camera extrinsics with LCTK, an open-source LiDAR–camera calibration toolkit, from 6, 4, 2 and 1 board placements. Wrote a converter that turns each result into an AutoSDV sensor-kit pose, and verified the four camera frames in RViz. | `lctk_to_autosdv.py` (Python, NumPy, SciPy) |
| [Reprojection error measurement](reprojection-error) | Implemented the math of an offline reprojection-error tool, then designed and ran three experiments on what makes a calibration generalize. | `reprojection.py` (NumPy) |

## Roadmap

| Status | Lab | Topic |
| --- | --- | --- |
| Done | Sensor calibration | LiDAR–camera extrinsic calibration and AutoSDV integration |
| Done | Reprojection error | Measuring how well calibrations generalize |
| In progress | Speed control | A feed-forward + PID longitudinal speed controller for the car, tuned in simulation and then on the vehicle |
| Planned | Localization | Localizing the vehicle with RTK GNSS and NDT scan matching against a prepared map, evaluated against ground truth |
| Planned | Final project | A team project on the AutoSDV platform: a review of recent research on the chosen topic and an implementation in simulation or on the vehicle |

## What is (and isn't) in this repository

This repository contains **only the code and configuration I wrote**, plus a summary of my results.

- The reprojection tool's loading code, CLI and tests were supplied by the course. Only `reprojection.py` is mine.
- The AutoSDV files I changed (`sensor_kit_calibration.yaml`, `sensor_kit.xacro`) belong to AutoSDV. `lidar-camera-calibration/autosdv-integration/` contains only the parts I added.
- The sensor recordings and Detection Archives are not included. The recording was provided by the course staff.

## Environment

Ubuntu 22.04, ROS 2 Humble, Autoware (AutoSDV), LCTK.
