#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
import math

class JointSliderGUI(Node):
    def __init__(self):
        super().__init__('joint_slider_gui')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        # --- Create the GUI ---
        self.root = tk.Tk()
        self.root.title("UR5e Joint Controller (Position Control)")
        
        self.sliders = []
        self.home_positions = []
        
        # [Joint Name, Min Radians, Max Radians, Home Position]
        joint_specs = [
            ("1: Base (shoulder_pan)", -6.28, 6.28, -1.5708),
            ("2: Shoulder (shoulder_lift)", -6.28, 6.28, -1.5708),
            ("3: Elbow (elbow)", -3.14, 3.14, 1.5708),
            ("4: Wrist 1 (wrist_1)", -6.28, 6.28, -1.5708),
            ("5: Wrist 2 (wrist_2)", -6.28, 6.28, -1.5708),
            ("6: Wrist 3 (wrist_3)", -6.28, 6.28, 0.0)
        ]

        # Create a slider (Scale) for each joint
        for i, (name, min_val, max_val, home_val) in enumerate(joint_specs):
            label = tk.Label(self.root, text=name)
            label.pack()
            
            slider = tk.Scale(
                self.root,
                from_=min_val,
                to=max_val,
                resolution=0.01, # Small steps
                orient=tk.HORIZONTAL,
                length=500, # Make sliders wider
                command=self.publish_joint_commands
            )
            slider.set(home_val) # Set to default 'home' position
            slider.pack()
            self.sliders.append(slider)
            self.home_positions.append(home_val)
    
        self.reset_button = tk.Button(
                self.root,
                text="Reset to Home",
                command=self.reset_sliders
            )
        self.reset_button.pack(pady=10) # Add some vertical padding

    # --- NEW --- Callback function for the reset button
    def reset_sliders(self):
        self.get_logger().info('Resetting sliders to home position...')
        for i, slider in enumerate(self.sliders):
            slider.set(self.home_positions[i])
        # Note: slider.set() automatically triggers the publish_joint_commands
        # callback, so the robot will move immediately.
    
    def publish_joint_commands(self, _=None):
        # This function is called every time a slider is moved
        msg = Float64MultiArray()
        msg.data = [s.get() for s in self.sliders]
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}') # Uncomment for debugging

    def run(self):
        # Start the tkinter main loop
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui_node = JointSliderGUI()
    
    try:
        gui_node.run() # This will block until the GUI is closed
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()