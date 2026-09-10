import numpy as np
from .configuration import RobotConfiguration
from .validation import SafetyValidator


class TrajectoryPlanner:
    def __init__(self, robot_config: RobotConfiguration):
        self.robot = robot_config
        self.safety = SafetyValidator(robot_config)

    def joint_space_trajectory(self, start_config, end_config,
                               num_points=50, method="linear"):
        errors = self.safety.validate_configuration(start_config)
        if errors:
            raise ValueError(f"Start config invalid: {'; '.join(errors)}")
        errors = self.safety.validate_configuration(end_config)
        if errors:
            raise ValueError(f"End config invalid: {'; '.join(errors)}")

        start = np.array(start_config, dtype=float)
        end = np.array(end_config, dtype=float)

        if method == "linear":
            t = np.linspace(0, 1, num_points)
            points = [start + ti * (end - start) for ti in t]
        elif method == "smooth":
            t = np.linspace(0, np.pi, num_points)
            s = (1 - np.cos(t)) / 2
            points = [start + si * (end - start) for si in s]
        elif method == "s_curve":
            t = np.linspace(0, 1, num_points)
            s = 3*t**2 - 2*t**3
            points = [start + si * (end - start) for si in s]
        else:
            points = [start + (i/(num_points-1)) * (end - start) for i in range(num_points)]

        return [list(p) for p in points]

    def cartesian_trajectory(self, start_pose, end_pose,
                              num_points=50, method="linear"):
        from .transforms import pose_to_transform

        T_start = pose_to_transform(start_pose)
        T_end = pose_to_transform(end_pose)

        if method == "linear":
            t = np.linspace(0, 1, num_points)
            T_points = [T_start + ti * (T_end - T_start) for ti in t]
        elif method == "smooth":
            t = np.linspace(0, np.pi, num_points)
            s = (1 - np.cos(t)) / 2
            T_points = [T_start + si * (T_end - T_start) for si in s]
        else:
            T_points = [T_start + (i/(num_points-1)) * (T_end - T_start) for i in range(num_points)]

        poses = []
        for T in T_points:
            R = T[:3, :3]
            r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
            r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
            r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
            pitch = np.arcsin(-r31)
            if np.cos(pitch) > 1e-6:
                roll = np.arctan2(r32, r33)
                yaw = np.arctan2(r21, r11)
            else:
                roll = np.arctan2(-r23, r12)
                yaw = 0.0

            poses.append({
                "x": float(T[0, 3]),
                "y": float(T[1, 3]),
                "z": float(T[2, 3]),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            })

        return poses

    def smooth_move(self, start_angle, end_angle, num_steps=50):
        angles = []
        for i in range(num_steps):
            t = i / (num_steps - 1)
            s = 3*t**2 - 2*t**3
            angle = start_angle + s * (end_angle - start_angle)
            angles.append(angle)
        return angles

    def trapezoidal_velocity_profile(self, start, end, max_vel,
                                      accel_time=None):
        distance = abs(end - start)
        if accel_time is None:
            accel_time = max_vel / 2.0 if max_vel > 0 else distance

        num_points = max(int(distance / max_vel * 10), 10)
        angles = []
        for i in range(num_points):
            t = i / (num_points - 1)
            if t < 0.5:
                frac = t / 0.5
                angle = start + frac * (end - start) * 0.5
            else:
                frac = (t - 0.5) / 0.5
                angle = start + (end - start) * 0.5 + frac * (end - start) * 0.5
            angles.append(angle)
        return angles