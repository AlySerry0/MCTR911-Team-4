#!/usr/bin/env python3
import ikpy.chain
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

def main():
    # 1. Define the path to the URDF file
    # This finds the file installed by the ROS2 package
    urdf_dir = get_package_share_directory('ur_description')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')

    if not os.path.exists(urdf_path):
        print(f"Error: Could not find URDF at {urdf_path}")
        print("Please ensure 'ros-humble-universal-robot-description' is installed.")
        return

    # 2. Create your kinematic chain from the URDF
    print(f"Loading URDF from: {urdf_path}")
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
    
    print("\n--- Kinematic Chain Links ---")
    print([link.name for link in chain.links])
    print("Chain loaded successfully.\n")

    # --- 3. FORWARD KINEMATICS (FK) ---
    # Answers "What is my (x, y, z) position if my joints are at these angles?"
    
    # The 'home' angles from your ur5e.xml file
    home_angles = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    
    # We must provide an angle for each link (including base and tool)
    # So we pad the list with 0 at the beginning and end.
    q_home = [0] + home_angles + [0] 

    fk = chain.forward_kinematics(q_home)
    fk_position = fk[:3, 3] # Extract the x, y, z position

    print("--- Forward Kinematics (FK) ---")
    print(f"Joint Angles (rad): {home_angles}")
    print(f"Calculated End-Effector Position (x,y,z): [{fk_position[0]:.4f}, {fk_position[1]:.4f}, {fk_position[2]:.4f}]\n")

    # --- 4. INVERSE KINEMATICS (IK) ---
    # Answers "What joint angles do I need to reach a specific (x, y, z) position?"

    target_position = [0.2, 0.3, 0.4]

    # This calculates the joint angles needed to reach the target
    # We give it the 'home' angles as a starting guess
    ik = chain.inverse_kinematics(target_position, initial_position=q_home)
    
    # We slice [1:7] to get only the 6 joint angles
    ik_angles = ik[1:7]

    print("--- Inverse Kinematics (IK) ---")
    print(f"Target Position (x,y,z): {target_position}")
    print(f"Calculated Joint Angles (rad):\n{np.round(ik_angles, 4)}")

if __name__ == '__main__':
    main()