# Multi-DOF Manipulator Framework

A universal 6-DOF robotic arm framework with hardware-first architecture, simulation validation, and ROS2 integration.

## Overview

This framework provides a complete software stack for controlling 6-DOF robotic arms. The design philosophy is **hardware-first**: the control system does not assume a simulator, and simulation is treated as an additional validation environment.

### Key Features

- **Forward Kinematics (FK)**: Computes end-effector pose from joint angles using Denavit-Hartenberg parameters
- **Inverse Kinematics (IK)**: Numerical Jacobian-based solver with reachability checking and multiple solutions
- **Safety System**: State machine with emergency stop, limit checking, and fault detection
- **Trajectory Planning**: Joint-space and Cartesian trajectories with smooth interpolation
- **Hardware Abstraction**: Mock hardware for testing, WebSocket interface for real deployment
- **Multi-Simulation**: OmniSim, Gazebo, and Isaac Sim adapters
- **Vision Integration**: Camera pipeline, pose estimation, and target detection
- **ROS2 Control**: `ros2_control` compatible interface
- **Configuration-Driven**: Add new robots via YAML files only
- **Automated Tests**: 49+ tests with pytest

## Installation

```bash
pip install -r requirements.txt
pip install -e .
```

## Quick Start

```python
from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics

config = RobotConfiguration.load("config/robot_a.yaml")
fk = ForwardKinematics(config)

# Forward Kinematics
pose = fk.compute(config.home_position)
print(f"EE Position: {pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f}")

# Inverse Kinematics
ik = InverseKinematics(config)
target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
result = ik.solve(target)
print(f"Joint Angles: {result['joint_angles']}")
```

## Architecture

```
Application Layer
    ↓
ROS2 Control Layer (optional)
    ↓
Network Layer (WebSocket)
    ↓
Safety State Machine
    ↓
Motion/Trajectory Layer
    ↓
Kinematics Layer (FK/IK)
    ↓
Robot Configuration (YAML)
    ↓
Hardware Interface → Real Hardware / Mock / Simulation
```

## Module Index

- **`core/`** — Generic framework (kinematics, configuration, validation, trajectory)
- **`hardware/`** — Hardware abstraction layer (base interface, mock)
- **`simulation/`** — Simulation adapters (OmniSim, Gazebo, Isaac Sim)
- **`core/safety/`** — Safety state machine and limits
- **`core/network/`** — WebSocket interface and robot server
- **`core/vision/`** — Vision pipeline and pose estimation
- **`examples/`** — Example scripts and calibration tools
- **`tests/`** — Automated test suite
- **`config/`** — Robot configuration files (YAML)
- **`urdf/`** — URDF robot descriptions
- **`Arduino/`** — Original Arduino firmware (preserved)

## Running Tests

```bash
pytest tests/ -v
pytest tests/ --cov=core --cov=hardware --cov=simulation
```

## License

MIT License. See [LICENSE](LICENSE) for details.
