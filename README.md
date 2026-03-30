<div align="center">

# Kookmin Self Driving
### Autonomous driving project built around ROS and Xycar

<img src="https://img.shields.io/badge/Language-Python-1f2937?style=for-the-badge" alt="Python badge" />
<img src="https://img.shields.io/badge/Middleware-ROS-0f766e?style=for-the-badge" alt="ROS badge" />
<img src="https://img.shields.io/badge/Vision-OpenCV-b45309?style=for-the-badge" alt="OpenCV badge" />
<img src="https://img.shields.io/badge/Platform-Xycar-1d4ed8?style=for-the-badge" alt="Xycar badge" />

</div>

---

## Overview

`Kookmin_Self_Driving` is a Python-based ROS autonomous driving project for an Xycar-style platform. The repository contains lane-following and mission-handling scripts built around camera input, lidar feedback, and topic-based vehicle control.

Rather than presenting a single polished runtime pipeline, the repository preserves several driving implementations and experiments. Across the codebase, the main focus is lane detection, steering control, traffic-light start handling, lidar-based obstacle navigation, and front-vehicle avoidance behavior.

---

## Core Capabilities

### Lane Following

- Bird's-eye-view lane processing using perspective transformation
- HSV-based lane filtering for white and yellow lane extraction
- Sliding-window lane tracking with histogram-based lane sampling
- Hough-transform-based lane detection variants for alternative driving pipelines

### Mission Handling

- Green-light detection before the vehicle starts driving
- Lidar-based navigation logic for rubber-cone or obstacle sections
- Front-obstacle monitoring and lane-change trigger logic
- Return-to-lane behavior after avoidance sequences

### Development Snapshot

- Multiple driving scripts are preserved for different approaches and experiments
- Steering and speed behavior are tuned through script-level parameter changes
- Debugging and visualization are built into several files with OpenCV windows and plotting logic

---

## Project Layout

```text
Kookmin_Self_Driving/
├── .vs/                    # Visual Studio workspace files
├── __pycache__/            # Python cache files
├── Share_page              # Auxiliary project artifact
├── avoid_wd_lidar.py       # Lidar-based avoidance support logic
├── driving.py              # Main integrated lane-following and mission pipeline
├── ep                      # Auxiliary project artifact
├── hough.py                # Hough-based lane detection implementation
├── hough_drive.py          # Class-based Hough lane driving pipeline
├── motortest.py            # Motor-related test script
├── racing.py               # Alternative lane-following implementation
├── rubber.py               # Lidar-based obstacle navigation and lane-change logic
├── sinhodeng.py            # Traffic-light detection helper
└── test.py                 # Additional test or experimental script
```

---

## Pipeline Snapshot

```text
Camera Image
  -> Color filtering / edge extraction
  -> Perspective transform or Hough-based processing
  -> Lane detection
  -> Steering and speed decision
  -> Motor command publish

Traffic Light
  -> Green-light detection
  -> Driving start condition

Lidar Scan
  -> Obstacle clustering / boundary check
  -> Avoidance trigger
  -> Lane-change and return behavior
```

---

## File Highlights

### `driving.py`

`driving.py` is the most integrated script in the repository. It combines image-based lane following, start-condition handling, lidar input, obstacle navigation, and avoidance-related state transitions in one flow.

### `rubber.py`

`rubber.py` contains lidar-centered navigation logic. It processes scan data, groups nearby obstacle points, and manages behavior for obstacle sections and lane-change triggers.

### `avoid_wd_lidar.py`

This file supports avoidance behavior by interpreting forward lidar data and helping determine when the vehicle should begin or finish an overtaking sequence.

### `hough.py`, `hough_drive.py`, and `racing.py`

These files preserve alternative lane-following approaches. Together they show different image-processing and steering strategies explored during development.

---

## Runtime Notes

The repository is organized around a ROS-based runtime and appears to assume a camera, lidar, and a vehicle-control topic interface.

What can be verified from the code:

- ROS publishers and subscribers are used throughout the main scripts
- OpenCV is used heavily for image processing
- Camera input is central to lane detection
- Lidar input is used for obstacle handling and mission logic
- Motor control is published through a ROS message interface

What is not yet documented in the repository:

- Exact installation steps
- ROS workspace setup
- Official launch procedure
- Hardware configuration details
- Recommended execution order between scripts

---

## Technology Snapshot

| Category | Details |
|---|---|
| Language | `Python` |
| Middleware | `ROS` |
| Vision | `OpenCV`, `NumPy` |
| Sensors | Camera, lidar |
| Control Interface | ROS topic-based motor command publishing |
| Repository Scope | Lane following, mission logic, experimental variants |

---

## Repository Status

This repository reads as a working project archive rather than a polished public framework. That makes it valuable as a snapshot of the actual development process, with multiple experiments and mission-specific scripts kept together in one place.

---
