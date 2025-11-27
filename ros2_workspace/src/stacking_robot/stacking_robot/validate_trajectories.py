#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from stacking_robot.trajectory_planner import TrajectoryPlanner

def main():
    print("Generating Validation Plots...")
    
    # Setup Planner
    urdf_dir = get_package_share_directory('stacking_robot')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')
    mask = [False, False, True, True, True, True, True, True, False]
    planner = TrajectoryPlanner(urdf_path, mask)

    # --- Test 1: Joint Space Validation (Quintic) ---
    print("Running Test 1: Joint Space...")
    q_start = [0, 0, 0, 0, 0, 0]
    q_end = [0.5, 1.0, -0.5, 0.2, 0.0, 0.0]
    t_total = 2.0
    
    t, q, v, a = planner.plan_joint_space_path(q_start, q_end, t_total)
    
    # Plot Position, Velocity, Acceleration for Joint 2 (Shoulder Lift)
    joint_idx = 1 
    
    fig, axs = plt.subplots(3, 1, figsize=(8, 10))
    fig.suptitle('Joint Space Validation (Quintic Polynomial) - Joint 2')
    
    axs[0].plot(t, q[:, joint_idx], 'b')
    axs[0].set_ylabel('Position (rad)')
    axs[0].set_title('Smooth curve from Start to End')
    axs[0].grid(True)
    
    axs[1].plot(t, v[:, joint_idx], 'g')
    axs[1].set_ylabel('Velocity (rad/s)')
    axs[1].set_title('Parabolic profile (Start/End = 0)')
    axs[1].grid(True)
    
    axs[2].plot(t, a[:, joint_idx], 'r')
    axs[2].set_ylabel('Accel (rad/s^2)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_title('Continuous Acceleration (No Jerk Spikes)')
    axs[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('validation_joint_space.png')
    print("Saved validation_joint_space.png")

    # --- Test 2: Task Space Validation (Linearity) ---
    print("Running Test 2: Task Space...")
    q_start_task = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0] # Home-ish pose
    
    # Get start XYZ
    q_full_start = np.concatenate(([0,0], q_start_task, [0]))
    start_fk = planner.chain.forward_kinematics(q_full_start)
    start_xyz = start_fk[:3, 3]
    
    # FIX: Move INWARDS instead of OUTWARDS to stay within workspace radius (0.85m)
    # Previous: +0.2m (Too far). New: -0.2m (Safe retraction).
    target_xyz = start_xyz + np.array([-0.2, 0.0, -0.1])
    
    t_task, q_task_traj = planner.plan_task_space_path(q_start_task, target_xyz, total_time=2.0)
    
    # Validation: Run FK on result to see if it is linear
    actual_x = []
    actual_z = []
    
    for q_step in q_task_traj:
        q_full = np.concatenate(([0,0], q_step, [0]))
        fk = planner.chain.forward_kinematics(q_full)
        actual_x.append(fk[0, 3])
        actual_z.append(fk[2, 3])
        
    plt.figure(figsize=(8, 6))
    plt.plot(actual_x, actual_z, 'b.-', label='Computed Path')
    plt.plot([start_xyz[0], target_xyz[0]], [start_xyz[2], target_xyz[2]], 'r--', label='Ideal Linear Path')
    plt.title('Task Space Validation: Cartesian Linearity check')
    plt.xlabel('X Position (m)')
    plt.ylabel('Z Position (m)')
    plt.legend()
    plt.grid(True)
    plt.savefig('validation_task_space.png')
    print("Saved validation_task_space.png")

if __name__ == '__main__':
    main()