import yaml
import os
import numpy as np
from .exceptions import ConfigurationError


class JointSpec:
    def __init__(self, name, min_angle, max_angle, min_velocity=0.0,
                 max_velocity=0.0, min_acceleration=0.0, max_acceleration=0.0,
                 offset=0.0, direction=1.0, servo_id=None, pin=None):
        self.name = name
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.min_velocity = float(min_velocity)
        self.max_velocity = float(max_velocity)
        self.min_acceleration = float(min_acceleration)
        self.max_acceleration = float(max_acceleration)
        self.offset = float(offset)
        self.direction = float(direction)
        self.servo_id = servo_id
        self.pin = pin

    def validate_angle(self, angle):
        adjusted = (angle - self.offset) * self.direction
        return self.min_angle <= adjusted <= self.max_angle

    def clamp_angle(self, angle):
        adjusted = (angle - self.offset) * self.direction
        clamped = np.clip(adjusted, self.min_angle, self.max_angle)
        return (clamped / self.direction) + self.offset

    def to_dict(self):
        return {
            "name": self.name,
            "min_angle": self.min_angle,
            "max_angle": self.max_angle,
            "min_velocity": self.min_velocity,
            "max_velocity": self.max_velocity,
            "min_acceleration": self.min_acceleration,
            "max_acceleration": self.max_acceleration,
            "offset": self.offset,
            "direction": self.direction,
            "servo_id": self.servo_id,
            "pin": self.pin,
        }

    @classmethod
    def from_dict(cls, data):
        return cls(
            name=data["name"],
            min_angle=data["min_angle"],
            max_angle=data["max_angle"],
            min_velocity=data.get("min_velocity", 0.0),
            max_velocity=data.get("max_velocity", 0.0),
            min_acceleration=data.get("min_acceleration", 0.0),
            max_acceleration=data.get("max_acceleration", 0.0),
            offset=data.get("offset", 0.0),
            direction=data.get("direction", 1.0),
            servo_id=data.get("servo_id"),
            pin=data.get("pin"),
        )


class DHParameterSet:
    def __init__(self, a, alpha, d, theta_offset=0.0):
        self.a = float(a)
        self.alpha = float(alpha)
        self.d = float(d)
        self.theta_offset = float(theta_offset)

    def to_list(self):
        return [self.a, self.alpha, self.d, self.theta_offset]

    @classmethod
    def from_list(cls, data):
        return cls(a=data[0], alpha=data[1], d=data[2], theta_offset=data[3])

    def to_dict(self):
        return {"a": self.a, "alpha": self.alpha, "d": self.d, "theta_offset": self.theta_offset}

    @classmethod
    def from_dict(cls, data):
        return cls(a=data["a"], alpha=data["alpha"], d=data["d"], theta_offset=data.get("theta_offset", 0.0))


class RobotConfiguration:
    def __init__(self, name, dof=6, joints=None, dh_params=None,
                 base_frame="base_link", tool_frame="tool_link",
                 home_position=None, units="degrees"):
        self.name = name
        self.dof = dof
        self.joints = joints or []
        self.dh_params = dh_params or []
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.home_position = home_position or [90.0] * dof
        self.units = units
        self._validate()

    def _validate(self):
        if self.dof < 1:
            raise ConfigurationError("DOF must be at least 1")
        if len(self.joints) != self.dof:
            raise ConfigurationError(
                f"Expected {self.dof} joints, got {len(self.joints)}"
            )
        if len(self.dh_params) != self.dof:
            raise ConfigurationError(
                f"Expected {self.dof} DH parameter sets, got {len(self.dh_params)}"
            )
        for j in self.joints:
            if not isinstance(j, JointSpec):
                raise ConfigurationError(f"Invalid joint spec: {j}")
        for d in self.dh_params:
            if not isinstance(d, DHParameterSet):
                raise ConfigurationError(f"Invalid DH parameter set: {d}")

    def get_dh_table(self):
        return [d.to_list() for d in self.dh_params]

    def get_joint_angles_deg(self, joint_angles_rad):
        if self.units == "degrees":
            return [np.degrees(a) for a in joint_angles_rad]
        return list(joint_angles_rad)

    def get_joint_angles_rad(self, joint_angles_deg):
        if self.units == "degrees":
            return [np.radians(a) for a in joint_angles_deg]
        return list(joint_angles_deg)

    def validate_configuration(self, joint_angles_deg):
        errors = []
        if len(joint_angles_deg) != self.dof:
            errors.append(f"Expected {self.dof} joint values, got {len(joint_angles_deg)}")
        for i, angle in enumerate(joint_angles_deg):
            if not self.joints[i].validate_angle(angle):
                errors.append(
                    f"Joint {i+1} ({self.joints[i].name}): {angle}° out of range "
                    f"[{self.joints[i].min_angle}, {self.joints[i].max_angle}]"
                )
        return errors

    def clamp_configuration(self, joint_angles_deg):
        return [self.joints[i].clamp_angle(a) for i, a in enumerate(joint_angles_deg)]

    def is_reachable(self, x, y, z, tolerance=0.001):
        from .transforms import compute_forward_kinematics
        home_rad = self.get_joint_angles_rad(self.home_position)
        dh_table = self.get_dh_table()
        T = compute_forward_kinematics(dh_table, home_rad)
        home_pos = (T[0, 3], T[1, 3], T[2, 3])
        max_reach = sum(abs(d.d) + abs(d.a) for d in self.dh_params)
        dist = ((x - home_pos[0])**2 + (y - home_pos[1])**2 + (z - home_pos[2])**2) ** 0.5
        return dist <= max_reach, max_reach, dist

    def to_dict(self):
        return {
            "name": self.name,
            "dof": self.dof,
            "joints": [j.to_dict() for j in self.joints],
            "dh_params": [d.to_dict() for d in self.dh_params],
            "base_frame": self.base_frame,
            "tool_frame": self.tool_frame,
            "home_position": self.home_position,
            "units": self.units,
        }

    @classmethod
    def from_dict(cls, data):
        joints = [JointSpec.from_dict(j) for j in data["joints"]]
        dh = [DHParameterSet.from_dict(d) for d in data["dh_params"]]
        return cls(
            name=data["name"],
            dof=data["dof"],
            joints=joints,
            dh_params=dh,
            base_frame=data.get("base_frame", "base_link"),
            tool_frame=data.get("tool_frame", "tool_link"),
            home_position=data.get("home_position", [90.0] * data["dof"]),
            units=data.get("units", "degrees"),
        )

    @classmethod
    def load(cls, filepath):
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    def save(self, filepath):
        with open(filepath, "w") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    def __repr__(self):
        return f"RobotConfiguration(name='{self.name}', dof={self.dof})"


def load_robot(filepath):
    return RobotConfiguration.load(filepath)