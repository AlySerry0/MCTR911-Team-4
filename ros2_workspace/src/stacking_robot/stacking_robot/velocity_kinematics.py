#!/usr/bin/env python3
import ikpy.chain
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# --- (get_jacobian and get_jacobian_derivative functions are unchanged) ---
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

def get_jacobian_derivative(chain, q_full, q_dot_active, dt=0.001):
    J_q = get_jacobian(chain, q_full)
    q_dot_full = np.zeros(len(q_full))
    active_indices = [2, 3, 4, 5, 6, 7]
    np.put(q_dot_full, active_indices, q_dot_active)
    q_next_full = np.array(q_full) + q_dot_full * dt
    J_q_next = get_jacobian(chain, q_next_full.tolist())
    J_dot = (J_q_next - J_q) / dt
    return J_dot
# -----------------------------------------------------------------

def main():
    # Set print precision
    np.set_printoptions(precision=4, suppress=True)

    # --- 1. Load the kinematic chain ---
    urdf_dir = get_package_share_directory('stacking_robot')
    urdf_path = os.path.join(urdf_dir, 'urdf', 'ur5e.urdf')
    print(f"Loading URDF from: {urdf_dir}")
    correct_mask = [False, False, True, True, True, True, True, True, False]
    chain = ikpy.chain.Chain.from_urdf_file(urdf_path, active_links_mask=correct_mask)
    print("Chain loaded successfully.\n")

    # --- 2. Define Joint State ---
    q_full = [
        0.0, 0.0, -1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0, 0.0
    ]
    q_active = np.array(q_full[2:8])
    q_dot_active = np.array([0.5, 0.1, 0.0, 0.2, 0.0, 0.0]) # Use a more complex q_dot
    q_ddot_active = np.zeros(6) 

    # --- 3. Forward Velocity Kinematics (Req #1) ---
    J = get_jacobian(chain, q_full)
    v = J.dot(q_dot_active)

    print("--- Forward Velocity Kinematics (Req #1) ---")
    print(f"Joint Angles (q):\n{q_active}\n")
    print(f"Joint Velocities (q_dot):\n{q_dot_active}\n")
    print(f"Jacobian Matrix (J) 6x6:\n{J}\n")
    print(f"Calculated End-Effector Velocity (v = J * q_dot):\n{v}\n")

    # --- 4. Inverse Velocity Kinematics (Req #1 Code) ---
    # We use the pseudo-inverse (pinv) in case J is singular
    J_inv = np.linalg.pinv(J)
    q_dot_calculated = J_inv.dot(v)

    print("--- Inverse Velocity Kinematics (Req #1) ---")
    print(f"Target End-Effector Velocity (v):\n{v}\n")
    print(f"Jacobian Inverse (J_inv) 6x6:\n{J_inv}\n")
    print(f"Calculated Joint Velocities (q_dot = J_inv * v):\n{q_dot_calculated}\n")
    print(f"Error (q_dot - q_dot_calculated): {np.linalg.norm(q_dot_active - q_dot_calculated)}\n")

    # --- 5. Acceleration Kinematics (Req #2) ---
    J_dot = get_jacobian_derivative(chain, q_full, q_dot_active)
    a = J.dot(q_ddot_active) + J_dot.dot(q_dot_active)

    print("--- Acceleration Kinematics (Req #2) ---")
    print(f"Jacobian Derivative (J_dot) 6x6:\n{J_dot}\n")
    print(f"Joint Accelerations (q_ddot):\n{q_ddot_active}\n")
    print(f"Calculated End-Effector Acceleration (a = J*q_ddot + J_dot*q_dot):\n{a}\n")

if __name__ == '__main__':
    main()