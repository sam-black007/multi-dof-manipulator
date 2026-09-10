import numpy as np
import time
import threading
from typing import Optional, List, Dict, Any
from .. import SimulationAdapter
from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics


class IsaacInterface:
    def __init__(self, sim_app=None):
        self._sim_app = sim_app
        self._connected = False
        self._assets: Dict[str, Any] = {}
        self._physics_dt = 1.0 / 60.0
        self._rendering_enabled = True
        self._joint_states: Dict[str, float] = {}
        self._sensor_data: Dict[str, Any] = {}

    def connect(self, environment_name: str = "IsaacLab"):
        self._connected = True
        self._env_name = environment_name
        return True

    def disconnect(self):
        self._connected = False

    def is_connected(self):
        return self._connected

    def set_joint_position(self, joint_name: str, position: float):
        if not self._connected:
            raise RuntimeError("Isaac Sim not connected")
        self._joint_states[joint_name] = position

    def get_joint_position(self, joint_name: str) -> float:
        return self._joint_states.get(joint_name, 0.0)

    def get_joint_positions(self) -> Dict[str, float]:
        return dict(self._joint_states)

    def set_prim_path(self, prim_path: str):
        self._prim_path = prim_path

    def get_prim_path(self) -> str:
        return getattr(self, "_prim_path", "/World/Robot")

    def get_sensor_data(self, sensor_name: str) -> Any:
        return self._sensor_data.get(sensor_name)

    def step(self, dt: float):
        return {"status": "stepped", "dt": dt, "frame": int(time.time() / dt)}

    def get_frame_time(self) -> float:
        return getattr(self, "_frame_time", 0.0)

    def reset(self):
        self._joint_states = {}
        self._sensor_data = {}


class IsaacSimAdapter(SimulationAdapter):
    def __init__(self, sim_app=None, headless=True):
        super().__init__()
        self._isaac = IsaacInterface(sim_app)
        self._headless = headless
        self._camera_enabled = False
        self._sensor_config: Dict[str, Any] = {}

    def initialize(self, robot_config: RobotConfiguration):
        super().initialize(robot_config)
        self._isaac.connect(f"arm_{robot_config.name.replace(' ', '_')}")
        self._setup_sensors()

    def _setup_sensors(self):
        self._sensor_config = {
            "camera": {
                "type": "RGB",
                "resolution": (1920, 1080),
                "fov": 90.0,
                "near": 0.1,
                "far": 1000.0,
            },
            "lidar": {
                "type": "2D_Lidar",
                "range": 10.0,
                "resolution": 0.5,
            },
            "force_torque": {
                "type": "FT_SENSOR",
                "range": 50.0,
            },
            "joint_state": {
                "type": "JOINT_STATE",
                "count": self.robot_config.dof,
            },
        }

    def set_joint_angles(self, joint_angles_deg):
        super().set_joint_angles(joint_angles_deg)
        joint_names = [f"joint_{i}" for i in range(self.robot_config.dof)]
        for i, angle in enumerate(joint_angles_deg):
            if i < len(joint_names):
                self._isaac.set_joint_position(joint_names[i], angle)

    def get_joint_positions(self) -> Dict[str, float]:
        return self._isaac.get_joint_positions()

    def enable_camera(self, enabled=True):
        self._camera_enabled = enabled

    def is_camera_enabled(self):
        return self._camera_enabled

    def get_sensor_data(self, sensor_name: str) -> Any:
        return self._isaac.get_sensor_data(sensor_name)

    def get_all_sensor_config(self) -> Dict[str, Any]:
        return dict(self._sensor_config)

    def add_sensor(self, name: str, sensor_type: str, config: Dict[str, Any]):
        self._sensor_config[name] = {"type": sensor_type, **config}

    def step_simulation(self, dt=1.0/60.0):
        return self._isaac.step(dt)

    def get_isaac_info(self):
        return {
            "env_name": getattr(self._isaac, "_env_name", "unknown"),
            "connected": self._isaac.is_connected(),
            "headless": self._headless,
            "camera_enabled": self._camera_enabled,
            "sensor_count": len(self._sensor_config),
            "prim_path": self._isaac.get_prim_path(),
        }

    def reset(self):
        super().reset()
        self._isaac.reset()
