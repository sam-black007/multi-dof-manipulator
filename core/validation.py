import numpy as np
from .configuration import RobotConfiguration


class SafetyValidator:
    def __init__(self, robot_config: RobotConfiguration):
        self.robot = robot_config
        self.position_tolerance = 0.01
        self.orientation_tolerance = 0.0175

    def validate_joint_position(self, joint_id, angle):
        if joint_id < 0 or joint_id >= self.robot.dof:
            return False, f"Joint ID {joint_id} out of range [0, {self.robot.dof-1}]"
        valid = self.robot.joints[joint_id].validate_angle(angle)
        if not valid:
            j = self.robot.joints[joint_id]
            return False, f"Joint {joint_id+1} ({j.name}): {angle}° outside [{j.min_angle}, {j.max_angle}]"
        return True, ""

    def validate_configuration(self, joint_angles_deg):
        errors = []
        if len(joint_angles_deg) != self.robot.dof:
            errors.append(f"Expected {self.robot.dof} joints, got {len(joint_angles_deg)}")
        for i, angle in enumerate(joint_angles_deg):
            if not isinstance(angle, (int, float, np.floating)):
                errors.append(f"Joint {i}: invalid type {type(angle)}")
            elif not np.isfinite(angle):
                errors.append(f"Joint {i}: non-finite value {angle}")
            elif not self.robot.joints[i].validate_angle(angle):
                j = self.robot.joints[i]
                errors.append(
                    f"Joint {i+1} ({j.name}): {angle}° outside [{j.min_angle}, {j.max_angle}]"
                )
        return errors

    def validate_joint_configuration(self, joint_angles_deg):
        errors = self.validate_configuration(joint_angles_deg)
        if errors:
            return False, "; ".join(errors)
        return True, ""

    def validate_trajectory(self, trajectory_points, max_velocity=None,
                            max_acceleration=None):
        errors = []
        if len(trajectory_points) < 2:
            errors.append("Trajectory must have at least 2 points")
            return False, "; ".join(errors)

        for i, point in enumerate(trajectory_points):
            errs = self.validate_configuration(point)
            if errs:
                errors.append(f"Point {i}: {'; '.join(errs)}")

        if max_velocity is not None:
            for i in range(1, len(trajectory_points)):
                for j in range(self.robot.dof):
                    delta = abs(trajectory_points[i][j] - trajectory_points[i-1][j])
                    if delta > max_velocity[j]:
                        errors.append(
                            f"Velocity limit exceeded on joint {j+1} between points "
                            f"{i-1} and {i}: {delta} > {max_velocity[j]}"
                        )

        if max_acceleration is not None:
            for i in range(2, len(trajectory_points)):
                for j in range(self.robot.dof):
                    v1 = trajectory_points[i][j] - trajectory_points[i-1][j]
                    v0 = trajectory_points[i-1][j] - trajectory_points[i-2][j]
                    accel = abs(v1 - v0)
                    if accel > max_acceleration[j]:
                        errors.append(
                            f"Acceleration limit exceeded on joint {j+1} at point {i}"
                        )

        if errors:
            return False, "; ".join(errors)
        return True, ""

    def validate_target_reachability(self, x, y, z):
        reachable, max_reach, dist = self.robot.is_reachable(x, y, z)
        if not reachable:
            return False, f"Target ({x:.3f}, {y:.3f}, {z:.3f}) beyond reach ({max_reach:.3f})"
        return True, ""

    def validate_pose(self, pose):
        reachable, max_reach, dist = self.robot.is_reachable(
            pose["x"], pose["y"], pose["z"]
        )
        if not reachable:
            return False, f"Pose beyond reach"
        return True, ""

    def emergency_stop(self):
        return {
            "status": "EMERGENCY_STOP",
            "message": "All joint commands halted",
            "timestamp": "now",
        }

    def set_position_tolerance(self, tol):
        self.position_tolerance = tol

    def set_orientation_tolerance(self, tol):
        self.orientation_tolerance = tol