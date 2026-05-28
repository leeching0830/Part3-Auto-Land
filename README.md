# Autonomous UAV Precision Landing via AprilTag & ROS

This repository contains an autonomous vision-based precision landing system for UAVs (Unmanned Aerial Vehicles) using ROS Noetic, PX4 Autopilot, and the `apriltag_ros` tracking package. 

The system implements a robust Position-Based Visual Servoing (PBVS) architecture to guide the quadcopter from altitude to a smooth, reliable touchdown on a custom AprilTag platform.

---

## 🚀 Key Technical Features

* **PBVS Velocity Control Architecture:** Translates 2D image coordinates into real-world metric errors, maintaining consistent linear P-gain control performance at any altitude.
* **Camera Optical Axis Dynamic Compensation:** Calibrates and corrects intrinsic principal-point center misalignment offsets derived from simulation camera modules (`ex = raw_x + 1.31`, `ey = raw_y + 1.07`).
* **Anti-Pitch Tilt Debounce Filter:** Features a 10-frame (0.5s) persistent lock mechanism to eliminate landing triggers caused by camera tilt illusions when the vehicle brakes or accelerates rapidly.
* **40x40cm Dynamic Capture Basket:** Implements a realistic tolerance-based bounding zone, allowing smooth terminal sliding before handover to the autopilot's final landing sequence.
* **Automatic OFFBOARD Takeover:** Monitors telemetry states and autonomously requests flight controller override commands without requiring separate manual terminal inputs.

---

## 📂 Repository Structure

```text
.
├── apriltag_ros/                # Modified ROS tracking package
│   ├── apriltag_ros/
│   │   ├── config/
│   │   │   └── tags.yaml        # Confined standalone tag settings (0.16m)
│   │   └── scripts/
│   │       └── landing_control.py # Core English-log PBVS tracking controller
├── gazebo_models/               # Deployed simulation assets
│   └── Apriltag36_11_00000/
│       ├── model.config
│       └── model.sdf            # Rescaled physical dimensions (0.2m x 0.2m)
├── .gitignore                   # Excludes __pycache__ and binary build artifacts
└── README.md                    # Project documentation
