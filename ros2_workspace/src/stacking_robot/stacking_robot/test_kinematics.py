#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import ikpy.chain
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class KinematicsTester(Node):
    def __init__(self):
        super().__init__('kinematics_tester')

        # --- 1. Load the Kinematic Chain ---
        urdf_dir = get_package_share_directory('stacking_robot')
        urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')

        # Define the correct active links mask
        correct_mask = [False, False, True, True, True, True, True, True, False]
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, active_links_mask=correct_mask)
        self.get_logger().info("Kinematic chain loaded successfully.")

        # --- 2. Calculate the "Expected" Position ---
        # These are the "home" angles from your GUI
        self.home_angles_active = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

        # Create the 9-link "full" joint list for FK
        q_full = [0.0, 0.0, -1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0, 0.0]

        fk_all_links = self.chain.forward_kinematics(q_full, full_kinematics=True)
        T_end_effector = fk_all_links[8] # Get matrix for link 8
        self.pos_expected = T_end_effector[:3, 3] # Get (x, y, z)

        print("\n--- KINEMATICS TEST ---")
        print(f"Calculated (Expected) Position: {np.round(self.pos_expected, 4)}\n")

        # --- 3. Subscribe to the simulation's joint states ---
        self.subscription = self.create_subscription(
            JointState,
            'ur5e/joint_states', # The topic from your new launch file
            self.joint_state_callback,
            10)
        self.get_logger().info("Waiting for 'ur5e/joint_states' from simulation...")
        self.has_run = False # Flag to only print once

    def joint_state_callback(self, msg):
        # This function runs every time the simulator publishes angles

        # We only want to run this test once
        if self.has_run:
            return

        self.get_logger().info("Received joint states from MuJoCo!")

        # Get the joint angles from the message.
        # NOTE: We must re-order them to match our IKPY chain if they are different.
        # For now, we assume the order is [pan, lift, elbow, w1, w2, w3]
        # This is a common source of error!

        # The 'mujoco_ros2' node reverses the order (as you found)
        # It publishes: [w3, w2, w1, elbow, lift, pan]
        # We must reverse it back to [pan, lift, elbow, w1, w2, w3]

        # If the node *doesn't* reverse it, just use:
        # sim_angles_active = msg.position

        sim_angles_active = msg.position

        # Create the 9-link "full" list for FK
        sim_q_full = [0.0, 0.0] + list(sim_angles_active) + [0.0]

        # Run FK on the angles from the simulator
        fk_sim = self.chain.forward_kinematics(sim_q_full, full_kinematics=True)
        T_sim_end_effector = fk_sim[8]
        pos_simulated = T_sim_end_effector[:3, 3]

        print(f"Simulated (Actual) Position:  {np.round(pos_simulated, 4)}\n")

        # --- 4. Compare the results ---
        error = np.linalg.norm(self.pos_expected - pos_simulated)
        print("--- RESULTS ---")
        print(f"Position Error (Difference): {error:.6f} meters")

        if error < 0.01:
            print("TEST PASSED: Kinematic models match!")
        else:
            print("TEST FAILED: Kinematic models DO NOT match.")

        print("\nShutting down test node.")
        self.has_run = True
        self.destroy_node() # Stop the node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tester_node = KinematicsTester()

    try:
        rclpy.spin(tester_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            tester_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()