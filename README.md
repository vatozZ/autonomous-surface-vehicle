# Autonomous Surface Vehicle (ASV) – Proof of Concept

This repository contains a proof-of-concept implementation of an **Autonomous Surface Vehicle (ASV)** capable of navigating within a **5x5 meter test pool** using only a **Lidar sensor** for localization. The ASV prototype uses **two BLDC thrusters** controlled via **ESCs** and driven by an **Arduino** microcontroller.

## System Overview

- The ASV autonomously navigates from a start point to a goal point inside a small pool environment.
- **No GPS is used.** Instead, a **2D Lidar sensor (RPLidar A1)** is mounted on the vehicle to perceive its surroundings.
- The Lidar scans feed a **Particle Filter**, which estimates the full state vector:
  - Position (x, y)
  - Velocity
  - Acceleration
  - Heading angle

## Navigation Pipeline

1. **Localization:**
   - Lidar data is processed to extract range and obstacle measurements.
   - A custom Particle Filter estimates the ASV's state based solely on Lidar input.

2. **Path Planning:**
   - A static polygonal path is currently hard-coded.
   - Optionally, you can implement a dynamic planner (e.g., A*, Q-learning) inside `path_planner.py`.

3. **Path Following:**
   - The estimated state is passed to a **Pure Pursuit controller** (`path_follower.py`) to follow the path.
   - The controller sends thruster commands to the motors through the Arduino-ESC interface.

## Hardware Components

- 2x **BLDC motors**
- 2x **ESC (Electronic Speed Controllers)**
- **Arduino** (controls ESCs via PWM)
- **2D Lidar Sensor** (for environment sensing and localization)
- No GPS or IMU used.

## Configuration

System parameters are defined in `constants.py`, including:
- Filter process noise settings
- Lidar maximum range
- Environment layout and obstacle map
- Vehicle dynamics (e.g., motor limits)

You can tune these parameters to match your own prototype or simulation environment.

## Demo

![ASV Demo](assets/autonomous-asv-test-pool.gif)

> A short simulation video showing the ASV navigating autonomously in the test pool.

## File Structure (Selected)

- `main.py`: Runs the full ASV pipeline (localization → control → navigation)
- `particle_filter.py`: State estimation using a custom particle filter
- `path_follower.py`: Pure pursuit controller for path tracking
- `path_planner.py`: Optional path planner (currently static polygonal path)
- `constants.py`: System parameters and configuration values

##  Notes

- This is a **prototype PoC**, intended to demonstrate key aspects of autonomous surface navigation.
- The system can be extended with IMU/GPS fusion, dynamic planners, or real-time Lidar processing for outdoor use cases.

## License

This code is provided for academic, research, and educational purposes.
