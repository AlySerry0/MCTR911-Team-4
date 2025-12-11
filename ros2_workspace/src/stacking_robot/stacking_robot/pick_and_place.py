#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState

class SimplePickAndPlace(Node):
    def __init__(self):
        super().__init__('simple_pick_and_place')
        
        # Try DIFFERENT COORDINATE TRANSFORMATIONS
        # Transformation 1: Original (from debug)
        self.targets_set1 = {
            "home": [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
            "prepick": [-0.103, -0.561, 0.884, -1.54, -2.112, 0.0],
            "pick": [-0.112, -0.541, 1.129, -0.751, -4.253, 0.0],
            "preplace": [0.761, -0.817, 1.409, -0.429, -5.965, 0.0],
            "place": [0.77, -0.606, 1.074, 0.611, -5.804, 0.0]
        }
        
        # Transformation 2: NEGATE X (move to opposite side)
        self.targets_set2 = {
            "home": [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
            "prepick": [0.103, -0.561, 0.884, -1.54, -2.112, 0.0],  # Negated first joint
            "pick": [0.112, -0.541, 1.129, -0.751, -4.253, 0.0],    # Negated first joint
            "preplace": [-0.761, -0.817, 1.409, -0.429, -5.965, 0.0], # Negated first joint
            "place": [-0.77, -0.606, 1.074, 0.611, -5.804, 0.0]     # Negated first joint
        }
        
        # Transformation 3: ADD 180 degrees to first joint (pan)
        self.targets_set3 = {
            "home": [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
            "prepick": [-0.103 + 3.1416, -0.561, 0.884, -1.54, -2.112, 0.0],  # +π
            "pick": [-0.112 + 3.1416, -0.541, 1.129, -0.751, -4.253, 0.0],    # +π
            "preplace": [0.761 + 3.1416, -0.817, 1.409, -0.429, -5.965, 0.0], # +π
            "place": [0.77 + 3.1416, -0.606, 1.074, 0.611, -5.804, 0.0]      # +π
        }
        
        # Transformation 4: COMPLETELY DIFFERENT (from Option 2 debug)
        self.targets_set4 = {
            "home": [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
            "prepick": [-3.4, -0.413, 0.571, 0.479, 0.778, 0.0],
            "pick": [-3.434, -0.462, 0.93, -1.019, -0.2, 0.0],
            "preplace": [-0.767, -3.698, 1.484, 2.957, -5.846, 0.0],
            "place": [-0.806, -3.698, 1.496, 3.242, -5.436, 0.0]
        }
        
        self.current_set = 1  # Start with set 1
        self.targets = self.targets_set1
        self.current_target = "home"
        
        # Publisher
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/pick_and_place_command',
            self.command_callback,
            10
        )
        
        self.get_logger().info("=== Pick and Place Debug ===")
        self.get_logger().info("Current set: 1 (Original)")
        self.get_logger().info("Commands:")
        self.get_logger().info("  set1, set2, set3, set4 - switch coordinate set")
        self.get_logger().info("  prepick, pick, preplace, place - test positions")
        self.get_logger().info("  home - return home")
        self.get_logger().info("  start - run full sequence")
        
    def command_callback(self, msg):
        cmd = msg.data.lower()
        
        if cmd == "set1":
            self.targets = self.targets_set1
            self.current_set = 1
            self.get_logger().info(f"Switched to Set {self.current_set} (Original)")
            
        elif cmd == "set2":
            self.targets = self.targets_set2
            self.current_set = 2
            self.get_logger().info(f"Switched to Set {self.current_set} (Negate X)")
            
        elif cmd == "set3":
            self.targets = self.targets_set3
            self.current_set = 3
            self.get_logger().info(f"Switched to Set {self.current_set} (+180 deg)")
            
        elif cmd == "set4":
            self.targets = self.targets_set4
            self.current_set = 4
            self.get_logger().info(f"Switched to Set {self.current_set} (Option 2)")
            
        elif cmd in self.targets:
            self.get_logger().info(f"Moving to {cmd} (Set {self.current_set})")
            self.move_to_target(cmd)
            
        elif cmd == "start":
            self.run_sequence()
            
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")
            
    def move_to_target(self, target_name):
        """Direct move to target"""
        target = self.targets[target_name]
        cmd_msg = Float64MultiArray()
        cmd_msg.data = target
        self.joint_cmd_pub.publish(cmd_msg)
        self.current_target = target_name
        
    def run_sequence(self):
        """Run sequence with current coordinate set"""
        self.get_logger().info(f"Starting sequence with Set {self.current_set}")
        
        self.move_to_target("prepick")
        time.sleep(3)
        
        self.move_to_target("pick")
        time.sleep(3)
        
        self.move_to_target("preplace")
        time.sleep(3)
        
        self.move_to_target("place")
        time.sleep(3)
        
        self.move_to_target("home")
        self.get_logger().info("Sequence complete!")

def main(args=None):
    rclpy.init(args=args)
    node = SimplePickAndPlace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
