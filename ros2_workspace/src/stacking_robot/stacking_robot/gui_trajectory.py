#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from stacking_robot.trajectory_planner import TrajectoryPlanner

class StackingGui(Node):
    def __init__(self):
        super().__init__('stacking_gui')
        
        # --- ROS Setup ---
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        self.current_joints = np.zeros(6)
        self.create_subscription(JointState, 'ur5e/joint_states', self.joint_state_callback, 10)
        
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.execution_callback)
        
        self.active_trajectory = [] 
        self.execution_index = 0
        self.is_executing = False
        
        # --- Planner Setup ---
        urdf_dir = get_package_share_directory('stacking_robot')
        urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')
        mask = [False, False, True, True, True, True, True, True, False]
        self.planner = TrajectoryPlanner(urdf_path, mask)
        self.print_timer = 0

        # --- GUI Setup ---
        self.root = tk.Tk()
        self.root.title("UR5e Trajectory Control (Milestone 04)")
        self.setup_ui()
        
    def joint_state_callback(self, msg):
        if len(msg.position) >= 6:
            # 1. Update internal state
            self.current_joints = np.array(msg.position[:6])
            
            # 2. Monitor Logic (Throttled to approx every 20 callbacks -> ~25Hz)
            self.print_timer += 1
            if self.print_timer % 20 == 0:
                # Prepare full chain vector (Padding: 2 zeros start, 1 zero end)
                # Matches your mask: [False, False, True, True, True, True, True, True, False]
                q_full = np.concatenate(([0.0, 0.0], self.current_joints, [0.0]))
                
                # Calculate Forward Kinematics
                fk_matrix = self.planner.chain.forward_kinematics(q_full)
                xyz = fk_matrix[:3, 3]
                
                # Print to Console (using \r to stay on the same line like a dashboard)
                print(f"LIVE EE POS: X={xyz[0]:.4f}  Y={xyz[1]:.4f}  Z={xyz[2]:.4f}    ", end='\r', flush=True)

    def execution_callback(self):
        """Called every 0.02s by ROS timer"""
        if self.is_executing and self.execution_index < len(self.active_trajectory):
            q_cmd = self.active_trajectory[self.execution_index]
            msg = Float64MultiArray()
            msg.data = q_cmd.tolist()
            self.publisher_.publish(msg)
            self.execution_index += 1
            
            # Optional: Update progress every 10 steps
            if self.execution_index % 10 == 0:
                 self.get_logger().info(f"Executing step {self.execution_index}/{len(self.active_trajectory)}")
                
        elif self.is_executing:
            # Finished
            self.is_executing = False
            self.get_logger().info("Trajectory Execution Complete.")
            
            # FIX: Reset GUI Label using .after() to be thread-safe
            self.root.after(0, self.reset_gui_state)

    def reset_gui_state(self):
        self.lbl_status.config(text="Status: Trajectory Complete")
        self.btn_exec.state(['!disabled'])

    def setup_ui(self):
        frame = ttk.Frame(self.root, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # --- Section 1: Joint Space ---
        ttk.Label(frame, text="JOINT SPACE CONTROL", font=('Arial', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)
        ttk.Label(frame, text="Target Joints (rads):").grid(row=1, column=0)
        self.entry_joint_target = ttk.Entry(frame, width=30)
        self.entry_joint_target.insert(0, "0.0, -1.57, 1.57, 0.0, 0.0, 0.0") 
        self.entry_joint_target.grid(row=1, column=1)

        self.btn_plan_joint = ttk.Button(frame, text="Plan Joint Trajectory", command=self.plan_joint_click)
        self.btn_plan_joint.grid(row=2, column=0, columnspan=2, pady=5)

        # --- Section 2: Task Space ---
        ttk.Label(frame, text="TASK SPACE CONTROL (Linear)", font=('Arial', 10, 'bold')).grid(row=3, column=0, columnspan=2, pady=(15, 5))
        ttk.Label(frame, text="Target XYZ (meters):").grid(row=4, column=0)
        self.entry_task_target = ttk.Entry(frame, width=30)
        self.entry_task_target.insert(0, "0.4, 0.1, 0.5") 
        self.entry_task_target.grid(row=4, column=1)

        self.btn_plan_task = ttk.Button(frame, text="Plan Linear Trajectory", command=self.plan_task_click)
        self.btn_plan_task.grid(row=5, column=0, columnspan=2, pady=5)

        # --- Section 3: Execution ---
        ttk.Separator(frame, orient='horizontal').grid(row=6, column=0, columnspan=2, sticky='ew', pady=10)
        self.lbl_status = ttk.Label(frame, text="Status: Idle", foreground="blue")
        self.lbl_status.grid(row=7, column=0, columnspan=2)

        self.btn_exec = ttk.Button(frame, text="EXECUTE TRAJECTORY", command=self.execute_click, state="disabled")
        self.btn_exec.grid(row=8, column=0, columnspan=2, pady=10)

    def plan_joint_click(self):
        try:
            target_str = self.entry_joint_target.get()
            q_target = [float(x) for x in target_str.split(',')]
            if len(q_target) != 6: raise ValueError("Need 6 Joint Angles")

            self.lbl_status.config(text="Planning Joint Path...")
            t, q_traj, v, a = self.planner.plan_joint_space_path(
                self.current_joints, q_target, total_time=4.0, dt=self.dt
            )
            self.active_trajectory = q_traj
            self.lbl_status.config(text=f"Joint Plan Ready: {len(q_traj)} steps")
            self.btn_exec.state(['!disabled'])
        except Exception as e:
            self.lbl_status.config(text=f"Error: {str(e)}")

    def plan_task_click(self):
        try:
            target_str = self.entry_task_target.get()
            xyz_target = [float(x) for x in target_str.split(',')]
            if len(xyz_target) != 3: raise ValueError("Need 3 XYZ Coordinates")

            self.lbl_status.config(text="Planning Linear Path...")
            t, q_traj = self.planner.plan_task_space_path(
                self.current_joints, xyz_target, total_time=4.0, dt=self.dt
            )
            self.active_trajectory = q_traj
            self.lbl_status.config(text=f"Task Plan Ready: {len(q_traj)} steps")
            self.btn_exec.state(['!disabled'])
        except Exception as e:
            self.lbl_status.config(text=f"Error: {str(e)}")

    def execute_click(self):
        self.execution_index = 0
        self.is_executing = True
        self.lbl_status.config(text="Executing...")
        self.btn_exec.state(['disabled'])

def main(args=None):
    rclpy.init(args=args)
    node = StackingGui()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()