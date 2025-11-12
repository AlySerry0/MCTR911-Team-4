#!/usr/bin/env python3
import ikpy.chain
import os
from ament_index_python.packages import get_package_share_directory

def main():
    urdf_dir = get_package_share_directory('stacking_robot')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')

    print(f"Loading URDF from: {urdf_path}\n")

    # Load the chain
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path)

    print("--- Chain Structure Report ---")
    print(f"Total links in chain: {len(chain.links)}")

    print("\nActive Links Mask:")
    print(f"(A list of Booleans: True = moving joint, False = fixed link)")
    print(chain.active_links_mask)

    print("\nLink Details:")
    for i, link in enumerate(chain.links):
        # Check if this link is active based on the mask
        is_active = chain.active_links_mask[i]

        print(f"  Index {i}: {link.name:<20} (Type: {link.joint_type:<8}) Active: {is_active}")

if __name__ == '__main__':
    main()