import numpy as np
from .transforms import (
    compute_forward_kinematics,
    transform_to_pose,
    pose_to_transform,
)
from .configuration import RobotConfiguration
from .exceptions import KinematicsError


class ForwardKinematics:
    def __init__(self, robot_config: RobotConfiguration):
        self.robot = robot_config

    def compute(self, joint_angles_deg):
        errors = self.robot.validate_configuration(joint_angles_deg)
        if errors:
            raise KinematicsError("; ".join(errors))

        joint_angles_rad = self.robot.get_joint_angles_rad(joint_angles_deg)
        dh_table = self.robot.get_dh_table()
        T = compute_forward_kinematics(dh_table, joint_angles_rad)
        pose = transform_to_pose(T)
        return pose

    def get_position(self, joint_angles_deg):
        pose = self.compute(joint_angles_deg)
        return (pose["x"], pose["y"], pose["z"])

    def get_orientation(self, joint_angles_deg):
        pose = self.compute(joint_angles_deg)
        return (pose["roll"], pose["pitch"], pose["yaw"])


class InverseKinematics:
    def __init__(self, robot_config: RobotConfiguration,
                 max_iterations=500, tolerance=1e-3,
                 position_tolerance=0.15, orientation_tolerance=0.05):
        self.robot = robot_config
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.iterations_used = 0

    def solve(self, target_pose, initial_guess=None):
        if initial_guess is None:
            initial_guess = self.robot.home_position

        errors = self.robot.validate_configuration(initial_guess)
        if errors:
            raise KinematicsError("Initial guess out of joint limits: " + "; ".join(errors))

        reachable, max_reach, dist = self.robot.is_reachable(
            target_pose["x"], target_pose["y"], target_pose["z"]
        )
        if not reachable:
            raise KinematicsError(
                f"Target ({target_pose['x']:.3f}, {target_pose['y']:.3f}, "
                f"{target_pose['z']:.3f}) is beyond reach "
                f"(max={max_reach:.3f}, dist={dist:.3f})"
            )

        if len(initial_guess) != self.robot.dof:
            initial_guess = self.robot.home_position

        q = np.array(self.robot.get_joint_angles_rad(initial_guess))
        T_target = pose_to_transform(target_pose)

        q_solution = self._numerical_ik(q, T_target)

        if q_solution is None:
            for _ in range(100):
                q = np.random.uniform(-np.pi, np.pi, self.robot.dof)
                q_solution = self._numerical_ik(q, T_target)
                if q_solution is not None:
                    break

        if q_solution is None:
            raise KinematicsError(
                f"IK did not converge after {self.max_iterations} iterations with random restarts"
            )

        q_deg = self.robot.get_joint_angles_deg(q_solution)
        clamped = self.robot.clamp_configuration(q_deg)

        errors = self.robot.validate_configuration(clamped)
        if errors:
            raise KinematicsError("Solution violates joint limits: " + "; ".join(errors))

        fk = ForwardKinematics(self.robot)
        achieved_pose = fk.compute(clamped)

        pos_error = np.sqrt(
            (achieved_pose["x"] - target_pose["x"])**2 +
            (achieved_pose["y"] - target_pose["y"])**2 +
            (achieved_pose["z"] - target_pose["z"])**2
        )

        return {
            "joint_angles": clamped,
            "iterations": self.iterations_used,
            "position_error": float(pos_error),
            "achieved_pose": achieved_pose,
        }

    def _numerical_ik(self, q_init, T_target):
        q = q_init.copy()
        best_q = q.copy()
        best_pos_err = np.inf
        
        for iteration in range(self.max_iterations):
            self.iterations_used = iteration + 1
            
            T_current = compute_forward_kinematics(self.robot.get_dh_table(), q)
            pos_current = T_current[:3, 3]
            pos_err = np.linalg.norm(pos_current - T_target[:3, 3])
            
            if pos_err < best_pos_err:
                best_pos_err = pos_err
                best_q = q.copy()
            
            if pos_err < 0.01:
                return q
            
            J = self._compute_jacobian(q)
            J_pos = J[:3, :]
            
            damp = 0.01
            delta = np.linalg.pinv(J_pos.T @ J_pos + damp * np.eye(J_pos.shape[1])) @ J_pos.T @ (T_target[:3, 3] - pos_current)
            
            q_new = q + delta * 0.3
            q_new = np.clip(q_new, -np.pi, np.pi)
            T_new = compute_forward_kinematics(self.robot.get_dh_table(), q_new)
            new_err = np.linalg.norm(T_new[:3, 3] - T_target[:3, 3])
            
            if new_err < pos_err:
                q = q_new
            
            if iteration % 50 == 0 and iteration > 0:
                q = best_q + np.random.uniform(-0.3, 0.3, self.robot.dof)
                q = np.clip(q, -np.pi, np.pi)
        
        return best_q

    def _compute_jacobian(self, q):
        dh = self.robot.get_dh_table()
        T = np.eye(4)
        transforms = []
        for i, (a, alpha, d, theta) in enumerate(dh):
            T_i = self._dh_transform(a, alpha, d, theta + q[i])
            T = T @ T_i
            transforms.append(T)

        J = np.zeros((6, self.robot.dof))
        pos = T[:3, 3]

        for i in range(self.robot.dof):
            z_i = transforms[i][:3, 2]
            p_i = transforms[i][:3, 3]
            J[:3, i] = np.cross(z_i, pos - p_i)
            J[3:, i] = z_i

        return J

    def _dh_transform(self, a, alpha, d, theta):
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ])

    def solve_multiple(self, target_pose, num_solutions=3):
        solutions = []
        initial_guesses = [
            self.robot.home_position,
            [0, -90, 90, 0, 0, 0],
            [45, -45, 90, 0, 45, 0],
            [90, 0, 0, 0, 90, 0],
            [180, 90, -90, 0, 0, 0],
            [0, 0, 90, 0, 0, 0],
            [45, 45, 45, 45, 45, 45],
            [30, -60, 90, 30, -30, 60],
            [60, -30, 120, 0, 60, 0],
            [120, -45, 45, 0, 90, 45],
        ]

        for guess in initial_guesses[:num_solutions]:
            try:
                result = self.solve(target_pose, initial_guess=guess)
                is_duplicate = False
                for existing in solutions:
                    if self._is_duplicate_joints(result["joint_angles"], existing["joint_angles"]):
                        is_duplicate = True
                        break
                if not is_duplicate:
                    solutions.append(result)
                if len(solutions) >= num_solutions:
                    break
            except KinematicsError:
                continue

        return solutions

    def _is_duplicate_joints(self, joints_a, joints_b, tol=5.0):
        return all(abs(a - b) < tol for a, b in zip(joints_a, joints_b))