import numpy as np
from typing import Tuple, Optional

class KukaIIWA14Kinematics:
    """
    Kinematic solver for the KUKA LBR iiwa 14 R820 manipulator.
    
    This class implements Forward Kinematics (FK) and numerical Inverse Kinematics (IK)
    based on the standard Denavit-Hartenberg (DH) parameters for the iiwa 14.
    
    References:
        - KUKA LBR iiwa 14 R820 Specifications
        - Siciliano, B. et al. "Robotics: Modelling, Planning and Control"
    """

    def __init__(self) -> None:
        """
        Initializes the kinematic model with DH parameters and joint limits.
        Units are in meters and radians.
        """
        # Number of joints
        self.num_joints = 7

        # Standard DH Parameters for KUKA LBR iiwa 14 R820
        # Format: [d (offset_z), a (offset_x), alpha (twist_x)]
        # Note: Theta is the joint variable.
        self.dh_params = [
            {'d': 0.360, 'a': 0.0, 'alpha': -np.pi / 2},  # Joint 1
            {'d': 0.0,   'a': 0.0, 'alpha':  np.pi / 2},  # Joint 2
            {'d': 0.420, 'a': 0.0, 'alpha':  np.pi / 2},  # Joint 3
            {'d': 0.0,   'a': 0.0, 'alpha': -np.pi / 2},  # Joint 4
            {'d': 0.400, 'a': 0.0, 'alpha': -np.pi / 2},  # Joint 5
            {'d': 0.0,   'a': 0.0, 'alpha':  np.pi / 2},  # Joint 6
            {'d': 0.126, 'a': 0.0, 'alpha':  0.0}         # Joint 7 (Flange)
        ]

        # Joint limits (radians) based on official documentation
        self.joint_limits_min = np.deg2rad([-170, -120, -170, -120, -170, -120, -175])
        self.joint_limits_max = np.deg2rad([ 170,  120,  170,  120,  170,  120,  175])

    def _compute_dh_matrix(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Computes the standard Denavit-Hartenberg homogeneous transformation matrix.
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])

    def forward_kinematics(self, q: np.ndarray) -> Tuple[np.ndarray, list]:
        """
        Computes the Forward Kinematics given joint angles.

        Args:
            q: Numpy array of 7 joint angles (radians).

        Returns:
            Tuple containing:
                - End-effector homogeneous transformation matrix (4x4).
                - List of transformation matrices for each joint frame (useful for Jacobians).
        """
        if len(q) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles, got {len(q)}.")

        frames = []
        T = np.eye(4)

        for i in range(self.num_joints):
            params = self.dh_params[i]
            A = self._compute_dh_matrix(q[i], params['d'], params['a'], params['alpha'])
            T = T @ A
            frames.append(T)

        return T, frames

    def compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Computes the geometric Jacobian matrix (6x7) for the given joint configuration.

        Args:
            q: Numpy array of 7 joint angles (radians).

        Returns:
            6x7 Geometric Jacobian matrix.
        """
        T_ee, frames = self.forward_kinematics(q)
        p_ee = T_ee[0:3, 3] # End-effector position

        J = np.zeros((6, self.num_joints))
        z_prev = np.array([0, 0, 1])
        p_prev = np.array([0, 0, 0])

        for i in range(self.num_joints):
            if i > 0:
                T_prev = frames[i-1]
                z_prev = T_prev[0:3, 2]
                p_prev = T_prev[0:3, 3]

            # Linear velocity Jacobian (cross product of z-axis and position vector)
            J[0:3, i] = np.cross(z_prev, (p_ee - p_prev))
            
            # Angular velocity Jacobian (z-axis of previous frame)
            J[3:6, i] = z_prev

        return J

    def _enforce_joint_limits(self, q: np.ndarray) -> np.ndarray:
        """
        Clamps the joint angles to the hardware limits of the LBR iiwa 14.
        """
        return np.clip(q, self.joint_limits_min, self.joint_limits_max)

    def inverse_kinematics(
        self, 
        target_pose: np.ndarray, 
        q_guess: Optional[np.ndarray] = None, 
        max_iter: int = 100, 
        tolerance: float = 1e-4,
        damping: float = 0.01
    ) -> Tuple[np.ndarray, bool]:
        """
        Computes the Inverse Kinematics using the Damped Least Squares (DLS) numerical method.
        This handles the 7-DOF redundancy smoothly and avoids singularities.

        Args:
            target_pose: 4x4 homogeneous transformation matrix representing the target.
            q_guess: Initial guess for joint angles. If None, uses zeros.
            max_iter: Maximum number of iterations for the solver.
            tolerance: Convergence threshold for the positional and rotational error.
            damping: Damping factor for the DLS pseudo-inverse (Levenberg-Marquardt).

        Returns:
            Tuple containing:
                - Solved joint angles (clamped to limits).
                - Boolean indicating if the solver converged successfully.
        """
        if q_guess is None:
            q = np.zeros(self.num_joints)
        else:
            q = np.copy(q_guess)

        target_pos = target_pose[0:3, 3]
        target_rot = target_pose[0:3, 0:3]

        for _ in range(max_iter):
            T_current, _ = self.forward_kinematics(q)
            current_pos = T_current[0:3, 3]
            current_rot = T_current[0:3, 0:3]

            # Calculate position error
            error_pos = target_pos - current_pos

            # Calculate orientation error (using cross product of rotation matrix columns)
            # This is a robust way to compute angular error for Jacobians
            error_rot = 0.5 * (
                np.cross(current_rot[:, 0], target_rot[:, 0]) +
                np.cross(current_rot[:, 1], target_rot[:, 1]) +
                np.cross(current_rot[:, 2], target_rot[:, 2])
            )

            # Combine into a 6D twist error vector
            error = np.hstack((error_pos, error_rot))

            # Check convergence
            if np.linalg.norm(error) < tolerance:
                return self._enforce_joint_limits(q), True

            # Get Jacobian
            J = self.compute_jacobian(q)

            # Damped Least Squares (DLS) Pseudo-inverse calculation
            # J_dls = J^T * (J * J^T + lambda^2 * I)^-1
            J_T = J.T
            lambda_matrix = (damping ** 2) * np.eye(6)
            J_dls = J_T @ np.linalg.inv(J @ J_T + lambda_matrix)

            # Update joint angles: delta_q = J_dls * error
            delta_q = J_dls @ error
            q = q + delta_q

            # Keep angles within [-pi, pi] for numerical stability
            q = (q + np.pi) % (2 * np.pi) - np.pi

        # Did not converge within max iterations
        return self._enforce_joint_limits(q), False

# Example usage (for internal testing)
if __name__ == "__main__":
    robot = KukaIIWA14Kinematics()

    start_point = [0.5, -0.2, 0.4]
    end_point = [0.5, 0.2, 0.6]
    
    # Target defined as a 4x4 matrix
    target_T = np.array([
        [1.0, 0.0, 0.0, 0.50],
        [0.0, -1.0, 0.0, 0.00],
        [0.0, 0.0, -1.0, 0.20],
        [0.0, 0.0, 0.0, 1.00]
    ])
    
    q_initial_guess = np.zeros(7)
    q_sol, success = robot.inverse_kinematics(target_T, q_initial_guess)
    
    print(f"Converged: {success}")
    print(f"Joint Angles (rad): {q_sol}")
