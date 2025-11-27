import numpy as np
import ikpy.chain

class TrajectoryPlanner:
    def __init__(self, urdf_path, active_mask):
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path, active_links_mask=active_mask)

    def get_quintic_scaling(self, t, T):
        """
        Returns s(t), s_dot(t), s_ddot(t) for a quintic polynomial 
        from 0 to 1 over time T.
        """
        tau = t / T
        # Quintic polynomial: s(tau) = 10t^3 - 15t^4 + 6t^5
        # This ensures vel and accel are 0 at start and end.
        if tau < 0: return 0, 0, 0
        if tau > 1: return 1, 0, 0

        s = 10 * (tau**3) - 15 * (tau**4) + 6 * (tau**5)
        s_dot = (30 * (tau**2) - 60 * (tau**3) + 30 * (tau**4)) / T
        s_ddot = (60 * tau - 180 * (tau**2) + 120 * (tau**3)) / (T**2)
        
        return s, s_dot, s_ddot

    def plan_joint_space_path(self, q_start, q_end, total_time, dt=0.02):
        """
        Generates a smooth joint trajectory.
        Returns: time_array, q_matrix (Nx6), v_matrix (Nx6), a_matrix (Nx6)
        """
        q_start = np.array(q_start)
        q_end = np.array(q_end)
        
        steps = int(total_time / dt)
        t_array = np.linspace(0, total_time, steps)
        
        q_traj = []
        v_traj = []
        a_traj = []

        for t in t_array:
            s, s_dot, s_ddot = self.get_quintic_scaling(t, total_time)
            
            # Interpolate
            q_t = q_start + s * (q_end - q_start)
            v_t = s_dot * (q_end - q_start)
            a_t = s_ddot * (q_end - q_start)
            
            q_traj.append(q_t)
            v_traj.append(v_t)
            a_traj.append(a_t)

        return t_array, np.array(q_traj), np.array(v_traj), np.array(a_traj)

    def plan_task_space_path(self, q_start, target_xyz, total_time, dt=0.02):
        """
        Generates a linear Cartesian path for the End Effector.
        Maintains the STARTING orientation throughout the move.
        """
        # 1. Get Starting Pose (FK)
        # Note: q_start needs dummy joints if chain expects 9 links but only 6 active
        # Assuming q_start passed here is already the 6 active joints.
        # We need to reconstruct the full 9-element array for ikpy if needed, 
        # but usually chain.forward_kinematics handles the active ones if configured right.
        # To be safe, we assume q_start is the standard 6DOF.
        
        # Expand q to full chain if necessary (Adding dummy fixed joints)
        # Based on your previous code, you padded with 0s. 
        # Let's auto-pad inside the loop or assume chain handles it.
        # For robustness, we will perform IK using the "seed" method.
        
        # --- 1. SAFETY CHECK: Workspace Radius ---
        # The UR5e reach is ~0.85m. We use 0.83m to be conservative/safe.
        # We calculate distance from the Base (0,0,0) to the Target.
        distance_to_target = np.linalg.norm(target_xyz)
        
        MAX_REACH = 0.85 # meters
        
        if distance_to_target > MAX_REACH:
            # Raise an error that the GUI will catch and display
            raise ValueError(f"Target Unreachable! ({distance_to_target:.2f}m > {MAX_REACH}m)")

        # --- 2. FK to get start pose (Existing Code) ---
        q_full_start = np.concatenate(([0,0], q_start, [0])) 
        start_frame_matrix = self.chain.forward_kinematics(q_full_start)
        
        # --- DEBUG START ---
        print(f"DEBUG: start_frame_matrix shape: {start_frame_matrix.shape}")
        # --- DEBUG END ---

        # FIX IS HERE: Ensure you slice [:3, 3]
        start_xyz = start_frame_matrix[:3, 3] 
        start_orientation = start_frame_matrix[:3, :3]

        # --- DEBUG START ---
        print(f"DEBUG: start_xyz shape: {start_xyz.shape}") # Must be (3,)
        # --- DEBUG END ---
        
        steps = int(total_time / dt)
        t_array = np.linspace(0, total_time, steps)
        q_traj = []
        
        # Linear Interpolation
        x_path = np.linspace(start_xyz[0], target_xyz[0], steps)
        y_path = np.linspace(start_xyz[1], target_xyz[1], steps)
        z_path = np.linspace(start_xyz[2], target_xyz[2], steps)
        
        previous_q_full = q_full_start

        for i in range(steps):
            target_pose = np.eye(4)
            
            # Rotation (Copy 3x3 into 3x3) -> Safe
            target_pose[:3, :3] = start_orientation
            
            # Position (Assign scalars) -> Safe
            target_pose[0, 3] = x_path[i]
            target_pose[1, 3] = y_path[i]
            target_pose[2, 3] = z_path[i]
            
            # Solve IK
            q_sol_full = self.chain.inverse_kinematics_frame(target_pose, initial_position=previous_q_full)
            q_active = q_sol_full[2:8] 
            q_traj.append(q_active)
            previous_q_full = q_sol_full
            
        return t_array, np.array(q_traj)