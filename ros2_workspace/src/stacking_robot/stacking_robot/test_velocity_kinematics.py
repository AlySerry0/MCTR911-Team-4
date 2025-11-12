#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import ikpy.chain
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# --- We copy our get_jacobian function from the other script ---
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

class VelocityKinematicsTester(Node):
    def __init__(self, chain):
        super().__init__('velocity_kinematics_tester')
        self.chain = chain
        self.has_run = False
        self.warmup_counter = 10

        # --- 1. Define our Test Case ---
        # We'll test at the 'home' position
        self.q_home_full = [
            0.0, 0.0, -1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0, 0.0
        ]
        # We'll command the first joint to move at 0.5 rad/s
        self.q_dot_test = np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # --- 2. Calculate the "Expected" Velocity ---
        J_expected = get_jacobian(self.chain, self.q_home_full)
        self.v_expected = J_expected.dot(self.q_dot_test)

        print("\n--- VELOCITY KINEMATICS TEST ---")
        print(f"Test Command (q_dot): {np.round(self.q_dot_test, 4)}")
        print(f"Calculated (Expected) Velocity (v): {np.round(self.v_expected, 4)}\n")

        # --- 3. Create Publisher and Subscriber ---
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.subscription = self.create_subscription(
            JointState,
            'ur5e/joint_states', # Listen to the sim
            self.joint_state_callback,
            10)

        # Timer to repeatedly send the command (since we're in velocity mode)
        self.timer = self.create_timer(0.1, self.publish_command)
        self.get_logger().info("Test node started. Publishing velocity commands...")

    def publish_command(self):
        if self.has_run:
            self.timer.cancel() # Stop publishing if test is done
            return

        msg = Float64MultiArray()
        msg.data = self.q_dot_test.tolist()
        self.publisher.publish(msg)

    def joint_state_callback(self, msg):
        if self.has_run:
            return
        
        if self.warmup_counter > 0:
                self.warmup_counter -= 1
                self.get_logger().info(f'Warming up... {self.warmup_counter}')
                return

        # --- 4. Get "Actual" Velocity from Simulator ---
        # We must use the *simulator's* current joint angles and velocities

        # Check if velocity data is present
        if len(msg.velocity) == 0:
            self.get_logger().warn("Simulator is not publishing velocity data. Test failed.")
            self.has_run = True
            return

        # Create the 9-link "full" list for FK from sim data
        sim_q_active = msg.position
        sim_q_full = [0.0, 0.0] + list(sim_q_active) + [0.0]

        sim_q_dot_active = np.array(msg.velocity)

        # Calculate the Jacobian *at the simulator's current position*
        J_sim = get_jacobian(self.chain, sim_q_full)

        # Calculate the "actual" end-effector velocity
        v_actual = J_sim.dot(sim_q_dot_active)

        print(f"Simulated (Actual) Velocity (v): {np.round(v_actual, 4)}\n")

        # --- 5. Compare Results ---
        error = np.linalg.norm(self.v_expected - v_actual)
        print("--- RESULTS ---")
        print(f"Velocity Error (Difference): {error:.6f} m/s")

        if error < 0.1: # Allow 10cm/s error, as vels are noisy
            print("TEST PASSED: Kinematic models match!")
        else:
            print("TEST FAILED: Kinematic models DO NOT match.")

        print("\nShutting down test node.")
        self.has_run = True
        self.destroy_node() # Stop the node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # --- Load the chain once and pass it to the node ---
    urdf_dir = get_package_share_directory('stacking_robot')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')
    correct_mask = [False, False, True, True, True, True, True, True, False]
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, active_links_mask=correct_mask)

    tester_node = VelocityKinematicsTester(chain)

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