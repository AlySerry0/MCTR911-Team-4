#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
import tf_transformations as tf

class FinalPickAndPlace(Node):
    def __init__(self):
        super().__init__('final_pick_and_place')
        
        # SET 3 COORDINATES (WORKING!)
        π = 3.1416
        
        self.positions = {
            "home": [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
            "prepick": [-0.103 + π, -0.561, 0.884, -1.54, -2.112, 0.0],
            "pick": [-0.112 + π, -0.541, 1.129, -0.751, -4.253, 0.0],
            "lift": [-0.112 + π, -0.541, 0.884, -1.54, -2.112, 0.0],
            "rotate": [-0.112 + π + π/2, -0.541, 0.884, -1.54, -2.112, 0.0],
            "above_stack": [0.761 + π + π/2, -0.817, 1.409, -0.429, -5.965, 0.0],
            "place": [0.77 + π + π/2, -0.606, 1.074, 0.611, -5.804, 0.0],
            "lift_from_stack": [0.77 + π + π/2, -0.817, 1.409, -0.429, -5.965, 0.0],
            "return": [-0.103 + π, -0.561, 0.884, -1.54, -2.112, 0.0]
        }
        
        # Virtual gripper state
        self.box_attached = False
        self.box_position = [0.8, 0.0, 0.08]  # Initial box position on conveyor
        self.gripper_open = True
        
        # State
        self.state = "IDLE"
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        # For visualization: publish box position if we want to show it moving
        self.box_pub = self.create_publisher(PoseStamped, '/virtual_box_position', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/pick_and_place_command', self.command_callback, 10)
        
        self.get_logger().info("=== FINAL PICK AND PLACE WITH VIRTUAL GRIPPER ===")
        self.get_logger().info("Commands: test1-test9, sequence, grip, release, stop")
        
    def command_callback(self, msg):
        cmd = msg.data.lower()
        
        # Test positions
        if cmd.startswith("test"):
            try:
                num = int(cmd[4:])
                position_names = list(self.positions.keys())
                if 1 <= num <= len(position_names):
                    name = position_names[num-1]
                    self.move_to(name, self.positions[name])
            except:
                pass
                
        elif cmd == "grip":
            self.gripper_open = False
            self.box_attached = True
            self.get_logger().info("GRIPPER CLOSED - Box attached")
            
        elif cmd == "release":
            self.gripper_open = True
            self.box_attached = False
            self.get_logger().info("GRIPPER OPENED - Box released")
            
        elif cmd == "sequence":
            if self.state == "IDLE":
                self.state = "RUNNING"
                self.run_sequence()
                
        elif cmd == "stop":
            self.state = "IDLE"
            self.get_logger().info("Stopped")
            
        elif cmd.startswith("manual "):
            try:
                parts = cmd.split()
                joints = [float(x) for x in parts[1:7]]
                self.move_to("manual", joints)
            except:
                self.get_logger().error("Invalid manual command")
                
    def move_to(self, name, joints):
        self.get_logger().info(f"Moving to {name}")
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joints
        self.joint_cmd_pub.publish(cmd_msg)
        
        # If box is attached, update its position (simulate picking)
        if self.box_attached:
            self.update_box_position(joints)
            
    def update_box_position(self, joints):
        """Simple simulation: box follows end effector"""
        # This is a simplification - in reality you'd compute FK
        # For now, just move box up with end effector
        if "pick" in self.state or "place" in self.state:
            # When at pick/place height, box is at end effector
            self.box_position[2] = 0.08  # Box height
        else:
            # When lifted, box is above
            self.box_position[2] = 0.15
            
        # Publish for visualization (optional)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose.position.x = self.box_position[0]
        pose.pose.position.y = self.box_position[1]
        pose.pose.position.z = self.box_position[2]
        self.box_pub.publish(pose)
        
    def run_sequence(self):
        """Complete pick-and-place with virtual gripping"""
        if self.state != "RUNNING":
            return
            
        sequence = [
            ("1. Move above conveyor", "prepick", 3, False),
            ("2. Lower to box", "pick", 2, False),
            ("3. GRIP box", None, 1, True),  # Special: grip action
            ("4. Lift box", "lift", 2, True),  # Box attached
            ("5. Rotate to stack", "rotate", 2, True),  # Box attached
            ("6. Move above stack", "above_stack", 2, True),  # Box attached
            ("7. Lower to place", "place", 2, True),  # Box attached
            ("8. RELEASE box", None, 1, False),  # Special: release action
            ("9. Lift from stack", "lift_from_stack", 2, False),
            ("10. Return home", "return", 2, False),
        ]
        
        for description, position, delay, should_have_box in sequence:
            if self.state != "RUNNING":
                break
                
            self.get_logger().info(description)
            
            if position is None:
                # Special action: grip or release
                if "GRIP" in description:
                    self.gripper_open = False
                    self.box_attached = True
                    self.get_logger().info(">>> VIRTUAL GRIP: Box picked up!")
                elif "RELEASE" in description:
                    self.gripper_open = True
                    self.box_attached = False
                    self.get_logger().info(">>> VIRTUAL RELEASE: Box placed!")
            else:
                # Normal movement
                self.move_to(position, self.positions[position])
                
                # Simulate box following if attached
                if self.box_attached and should_have_box:
                    self.get_logger().info(f">>> Box is being carried (virtual)")
            
            time.sleep(delay)
        
        if self.state == "RUNNING":
            self.get_logger().info("=== PICK-AND-PLACE COMPLETE! ===")
            self.state = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    node = FinalPickAndPlace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
