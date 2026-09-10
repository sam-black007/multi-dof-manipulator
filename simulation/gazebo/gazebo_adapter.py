import numpy as np
import time
import threading
from typing import Optional, List, Dict, Any
from .. import SimulationAdapter
from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics


class GazeboInterface:
    def __init__(self, gz_client=None):
        self._gz_client = gz_client
        self._connected = False
        self._models: Dict[str, Any] = {}
        self._joint_states: Dict[str, float] = {}

    def connect(self, model_name: str = "robot"):
        self._connected = True
        self._model_name = model_name
        return True

    def disconnect(self):
        self._connected = False

    def is_connected(self):
        return self._connected

    def set_joint_position(self, joint_name: str, position: float):
        if not self._connected:
            raise RuntimeError("Gazebo not connected")
        self._joint_states[joint_name] = position

    def get_joint_position(self, joint_name: str) -> float:
        return self._joint_states.get(joint_name, 0.0)

    def get_joint_positions(self) -> Dict[str, float]:
        return dict(self._joint_states)

    def set_model_pose(self, x: float, y: float, z: float,
                       roll: float, pitch: float, yaw: float):
        if not self._connected:
            raise RuntimeError("Gazebo not connected")
        self._model_pose = (x, y, z, roll, pitch, yaw)

    def get_model_pose(self) -> tuple:
        return getattr(self, "_model_pose", (0, 0, 0, 0, 0, 0))

    def get_joint_names(self) -> List[str]:
        return [f"joint_{i}" for i in range(6)]

    def pause(self):
        self._paused = True

    def unpause(self):
        self._paused = False

    def step(self, dt: float):
        if self._paused:
            return {"status": "paused"}
        return {"status": "stepped", "dt": dt}


class GazeboAdapter(SimulationAdapter):
    def __init__(self, gz_client=None):
        super().__init__()
        self._gz = GazeboInterface(gz_client)
        self._model_name = "robot"

    def initialize(self, robot_config: RobotConfiguration):
        super().initialize(robot_config)
        self._gz.connect(self._model_name)
        self._model_name = f"arm_{robot_config.name.replace(' ', '_')}"

    def set_joint_angles(self, joint_angles_deg):
        super().set_joint_angles(joint_angles_deg)
        joint_names = self._gz.get_joint_names()
        for i, angle in enumerate(joint_angles_deg):
            if i < len(joint_names):
                self._gz.set_joint_position(joint_names[i], angle)

    def get_joint_angles(self):
        angles = super().get_joint_angles()
        joint_names = self._gz.get_joint_names()
        gz_angles = [self._gz.get_joint_position(name) for name in joint_names[:len(angles)]]
        return angles if all(a == 0.0 for a in gz_angles) else angles

    def get_joint_positions(self) -> Dict[str, float]:
        return self._gz.get_joint_positions()

    def get_ee_position(self):
        return super().get_ee_position()

    def set_model_pose(self, x, y, z, roll, pitch, yaw):
        self._gz.set_model_pose(x, y, z, roll, pitch, yaw)

    def get_model_pose(self):
        return self._gz.get_model_pose()

    def pause_physics(self):
        self._gz.pause()

    def unpause_physics(self):
        self._gz.unpause()

    def step_simulation(self, dt=0.01):
        return self._gz.step(dt)

    def get_gazebo_info(self):
        return {
            "model_name": self._model_name,
            "connected": self._gz.is_connected(),
            "joint_count": len(self._gz.get_joint_names()),
            "joint_names": self._gz.get_joint_names(),
        }

    def reset(self):
        super().reset()
        self._gz.disconnect()
        self._gz.connect(self._model_name)
