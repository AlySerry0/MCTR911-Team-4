#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import ikpy.chain
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# --- We use the get_jacobian function we already built ---
def get_jacobian(chain, q_full):
    fk = chain.forward_kinematics(q_full, full_kinematics=True)
    T_end_effector = fk[8] # End-effector is link 8
    p_e = T_end_effector[:3, 3]
    
    active_joint_indices = [2, 3, 4, 5, 6, 7]
    J = np.zeros((6, 6))
    
    for i, joint_index in enumerate(active_joint_indices):
        T_joint = fk[joint_index]
        p_i = T_joint[:3, 3]
        z_i = T_joint[:3, 2]
        
        J_linear = np.cross(z_i, p_e - p_i)
        J_angular = z_i
        
        J[0:3, i] = J_linear
        J[3:6, i] = J_angular
        
    return J

class VelocityValidator(Node):
    def __init__(self, chain):
        super().__init__('velocity_validator')
        self.chain = chain
        
        self.subscription = self.create_subscription(
            JointState,
            'ur5e/joint_states', # Listen to the sim
            self.joint_state_callback,
            10)
        
        self.get_logger().info("Validator node started. Waiting for 'ur5e/joint_states'...")
        self.get_logger().info("Move the robot using the GUI sliders to see the calculations.")

    def joint_state_callback(self, msg):
        # This function runs every time the simulator publishes new data
        
        if len(msg.velocity) == 0:
            return

        # --- 1. Get ACTUAL data from MuJoCo ---
        q_actual_active = np.array(msg.position)
        q_dot_actual_active = np.array(msg.velocity)
        
        # --- 2. Calculate "Expected" End-Effector Velocity ---
        q_full = [0.0, 0.0] + list(q_actual_active) + [0.0]
        J = get_jacobian(self.chain, q_full)
        v_expected = J.dot(q_dot_actual_active)
        
        # --- 3. Print the Live Report (CLEANED UP) ---
        
        # --- NEW ---
        # Set print options for all arrays
        # precision=3 shows 3 decimal places
        # suppress_small=True prints tiny numbers as 0.000
        q_str = np.array_str(q_actual_active, precision=3, suppress_small=True)
        q_dot_str = np.array_str(q_dot_actual_active, precision=3, suppress_small=True)
        v_exp_str = np.array_str(v_expected, precision=3, suppress_small=True)
        
        print(f"--- LIVE KINEMATICS VALIDATION ---"
              f"\n  q (actual): {q_str}"
              f"\nq_dot (actual): {q_dot_str}"
              f"\n  v (expected): {v_exp_str}"
              f"\n" + ("-"*36), end="\r")

def main(args=None):
    rclpy.init(args=args)
    
    urdf_dir = get_package_share_directory('stacking_robot')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')
    correct_mask = [False, False, True, True, True, True, True, True, False]
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, active_links_mask=correct_mask)
    
    validator_node = VelocityValidator(chain)
    
    try:
        rclpy.spin(validator_node)
    except KeyboardInterrupt:
        print("\nShutting down validator.")
    finally:
        if rclpy.ok():
            validator_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()