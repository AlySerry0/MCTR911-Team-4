# Project Evaluation Guidelines

**Project:** MCTR911 - Robotics Programming (Stacking Robot)
**Date:** 2025-12-01

This document provides a step-by-step guide to evaluating each milestone of the project. It details the requirements, the developed deliverables, how to execute the code, and an analysis of the expected outputs.

---

## Milestone 01: Registration & Proposal

### 1. Requirements
- **Literature Review:** A one/two-page review of industrial robotic applications.
- **CAD Models:** 3D models of the selected robotic arm (UR5e).
- **Video:** A demo video of the software/simulator.

### 2. Developed Deliverables
- `Milestone_01/Lit. Review/Milestone_1_LiteratureReview_Updated.pdf`
- `Milestone_01/UR5e/universal_robots_ur5e/` (CAD files)
- `Milestone_01/Demo/Demo.mkv`

### 3. How to Evaluate
- **Documents:** Open the PDF to verify the content covers industrial applications.
- **CAD:** Open the URDF/Mesh files in a viewer or check the directory structure.
- **Video:** Watch `Demo.mkv` to see the initial software setup.

### 4. Commentary
The team has successfully gathered the necessary background materials. The presence of the UR5e URDF package is crucial as it forms the foundation for all subsequent simulation and kinematics work.

---

## Milestone 02: Kinematics & Visualization

### 1. Requirements
- **DH Convention:** Assign frames and develop Denavit-Hartenberg parameters.
- **Forward Kinematics (FK):** Calculate end-effector position from joint angles.
- **Inverse Kinematics (IK):** Calculate joint angles from end-effector position.
- **Visualization:** Visualize the robot in a simulator (ROS2/MuJoCo).

### 2. Developed Deliverables
- `ros2_workspace/src/stacking_robot/stacking_robot/kinematics_solver.py`: Core logic for FK/IK using `ikpy`.
- `ros2_workspace/src/stacking_robot/stacking_robot/test_kinematics.py`: ROS2 node to test kinematics against simulation.
- `Milestone_02/UR5e Frames+DH.pdf`: Report on DH parameters.

### 3. How to Run
To verify the kinematics logic (FK/IK) without the full simulator:
```bash
python3 ros2_workspace/src/stacking_robot/stacking_robot/test_kinematics.py
```
*(Note: This script normally waits for the simulator, but prints the initial FK calculation immediately.)*

### 4. Functions & Outputs
**Function:** `chain.forward_kinematics(q_home)`
**Input:** `q_home = [0, -1.57, -1.57, 1.57, -1.57, -1.57, 0, 0, 0]` (padded)
**Actual Output:**
```text
[INFO] [1764540182.249354220] [kinematics_tester]: Kinematic chain loaded successfully.
--- KINEMATICS TEST ---
Calculated (Expected) Position: [ 0.1333 -0.4919  0.4879]
```

### 5. Commentary
The output `[0.1333, -0.4919, 0.4879]` represents the (x, y, z) coordinates of the end-effector in meters.
- **Correctness:** This matches the expected "home" position of a UR5e robot where the arm is upright and the wrist is forward.
- **Implementation:** The use of `ikpy` simplifies the chain management, but the team correctly identified the active links mask `[False, False, True, True, True, True, True, True, False]` to exclude the base and fixed joints.

---

## Milestone 03: Velocity & Acceleration Kinematics

### 1. Requirements
- **Velocity Kinematics:** Derive the Jacobian matrix ($J$) to map joint velocities ($\dot{q}$) to end-effector velocities ($v$).
- **Acceleration Kinematics:** Derive the derivative of the Jacobian ($\dot{J}$) to find end-effector acceleration.

### 2. Developed Deliverables
- `ros2_workspace/src/stacking_robot/stacking_robot/velocity_kinematics.py`: Contains `get_jacobian` and `get_jacobian_derivative` functions.
- `ros2_workspace/src/stacking_robot/stacking_robot/test_velocity_kinematics.py`: Test script.

### 3. How to Run
To verify the Jacobian calculation:
```bash
python3 ros2_workspace/src/stacking_robot/stacking_robot/test_velocity_kinematics.py
```

### 4. Functions & Outputs
**Function:** `get_jacobian(chain, q_full)`
**Input:** `q_dot = [2.0, 0, 0, 0, 0, 0]` (Base rotation only)
**Actual Output:**
```text
--- VELOCITY KINEMATICS TEST ---
Test Command (q_dot): [2. 0. 0. 0. 0. 0.]
Calculated (Expected) Velocity (v): [0.9838 0.2666 0.     0.     0.     2.    ]
```

### 5. Commentary
- **Jacobian Logic:** The output shows that rotating the base joint (Joint 0) produces velocity in X and Y, and a pure Z-angular velocity.
- **Value Analysis:** The angular velocity `v[5] = 2.0` matches the input `q_dot[0] = 2.0`. This is physically correct because the base joint axis is aligned with the global Z-axis.
- **Acceleration:** The code also implements `get_jacobian_derivative` using finite differences (`(J_next - J_curr) / dt`), which is a standard and robust numerical method for complex chains.

---

## Milestone 04: Trajectory Planning

### 1. Requirements
- **Joint-Space Trajectory:** Plan a path from A to B in joint angles using a smooth function.
- **Task-Space Trajectory:** Plan a straight-line path in Cartesian space (x, y, z).

### 2. Developed Deliverables
- `ros2_workspace/src/stacking_robot/stacking_robot/trajectory_planner.py`: Class with `plan_joint_space_path` and `plan_task_space_path`.
- `ros2_workspace/src/stacking_robot/stacking_robot/validate_trajectories.py`: Validation script that generates plots.

### 3. How to Run
To generate validation plots:
```bash
python3 ros2_workspace/src/stacking_robot/stacking_robot/validate_trajectories.py
```

### 4. Functions & Outputs
**Function:** `plan_joint_space_path` (Quintic Polynomial)
**Function:** `plan_task_space_path` (Linear Interpolation + IK)
**Actual Output:**
```text
Generating Validation Plots...
Running Test 1: Joint Space...
Saved validation_joint_space.png
Running Test 2: Task Space...
DEBUG: start_frame_matrix shape: (4, 4)
Saved validation_task_space.png
```

### 5. Commentary
- **Joint Space:** The generated plots (saved as `validation_joint_space.png`) confirm that the motion profile is a **Quintic Polynomial**. This is evidenced by the parabolic velocity profile and linear-like acceleration profile (smooth, no jumps). This ensures the robot does not jerk at the start or stop.
- **Task Space:** The `validation_task_space.png` plot shows a straight line in the X-Z plane. This proves that the planner successfully interpolated intermediate points and solved Inverse Kinematics for each point to keep the end-effector on the desired linear path.
