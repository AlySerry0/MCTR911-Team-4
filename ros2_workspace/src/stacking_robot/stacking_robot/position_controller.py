#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class PIDController:
    """Simple PID controller for joint position control"""
    def __init__(self, kp=100.0, ki=10.0, kd=5.0, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        
        self.prev_error = error
        
        # Total output
        output = p_term + i_term + d_term
        
        # Clamp output to reasonable limits
        output = np.clip(output, -50, 50)
        
        return output

class UR5ePositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        
        # PID controllers for each joint
        self.pid_controllers = [PIDController() for _ in range(6)]
        
        # Current and target positions
        self.current_positions = np.zeros(6)
        self.target_positions = np.zeros(6)
        self.trajectory = None
        self.traj_index = 0
        self.is_moving = False
        
        # Subscriber for joint states from MuJoCo
        self.joint_state_sub = self.create_subscription(
            JointState,
            'ur5e/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )
        
        # Timer for control loop (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info("Position Controller initialized")
        
    def joint_state_callback(self, msg):
        """Update current joint positions from MuJoCo"""
        if len(msg.position) >= 6:
            # Reverse the order if needed (as per your earlier code)
            # Based on your kinematics test, the order might need adjustment
            self.current_positions = np.array(msg.position)
            
    def set_target_position(self, target):
        """Set a new target position (6-element array)"""
        self.target_positions = np.array(target)
        self.is_moving = True
        
    def follow_trajectory(self, trajectory):
        """Follow a pre-planned trajectory"""
        self.trajectory = trajectory
        self.traj_index = 0
        self.is_moving = True
        
    def control_loop(self):
        """Main control loop - runs at 100Hz"""
        if not self.is_moving:
            return
            
        # If following a trajectory, update target
        if self.trajectory is not None:
            if self.traj_index < len(self.trajectory):
                self.target_positions = self.trajectory[self.traj_index]
                self.traj_index += 1
            else:
                self.trajectory = None
                self.is_moving = False
                self.get_logger().info("Trajectory complete")
                return
        
        # Compute PID for each joint
        commands = np.zeros(6)
        for i in range(6):
            commands[i] = self.pid_controllers[i].compute(
                self.target_positions[i],
                self.current_positions[i]
            )
        
        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands.tolist()
        self.joint_cmd_pub.publish(cmd_msg)
        
        # Check if we've reached the target (within tolerance)
        error = np.linalg.norm(self.target_positions - self.current_positions)
        if error < 0.01 and self.trajectory is None:
            self.is_moving = False
            self.get_logger().info(f"Target reached with error: {error:.4f}")
            
    def move_to_home(self):
        """Move to home position"""
        home_pos = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        self.set_target_position(home_pos)
        
    def stop(self):
        """Stop all motion"""
        self.is_moving = False
        self.trajectory = None
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0] * 6
        self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = UR5ePositionController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
