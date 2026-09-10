# Quick Start Tutorial

## Prerequisites

- Python 3.9+
- NumPy, PyYAML, pytest

```bash
pip install -r requirements.txt
pip install -e .
```

## Step 1: Load a Robot Configuration

```python
from core.configuration import RobotConfiguration

config = RobotConfiguration.load("config/robot_a.yaml")
print(f"Robot: {config.name}, DOF: {config.dof}")
```

## Step 2: Run Forward Kinematics

```python
from core.kinematics import ForwardKinematics

fk = ForwardKinematics(config)
pose = fk.compute(config.home_position)
print(f"End-Effector: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")
```

## Step 3: Run Inverse Kinematics

```python
from core.kinematics import InverseKinematics

ik = InverseKinematics(config)
target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
result = ik.solve(target)
print(f"Solution: {result['joint_angles']}")
print(f"Position Error: {result['position_error']:.6f}")
print(f"Iterations: {result['iterations']}")
```

## Step 4: Validate with Safety Checks

```python
from core.validation import SafetyValidator

validator = SafetyValidator(config)
ok, msg = validator.validate_joint_configuration(result["joint_angles"])
print(f"Configuration Valid: {ok}")
```

## Step 5: Generate a Trajectory

```python
from core.trajectory import TrajectoryPlanner

planner = TrajectoryPlanner(config)
start = config.home_position
end = [45, 45, 45, 45, 45, 45]
traj = planner.joint_space_trajectory(start, end, num_points=50, method="smooth")
print(f"Generated {len(traj)} trajectory points")
```

## Step 6: Test with Mock Hardware

```python
from hardware.mock import MockHardwareInterface

hw = MockHardwareInterface(config)
hw.connect()
hw.move_all(config.home_position)
print(f"Position: {hw.get_current_position()}")
hw.emergency_stop()
hw.disconnect()
```

## Step 7: Run in Simulation

```python
from simulation import OmniSimAdapter

sim = OmniSimAdapter()
sim.initialize(config)
sim.set_joint_angles(config.home_position)
print(f"EE Position: {sim.get_ee_position()}")
sim.simulate_trajectory(traj)
sim.reset()
```

## Step 8: Use WebSocket Server

```python
from core.network import RobotServer

server = RobotServer(config)
server.start()
# Connect from any WebSocket client
# Commands: get_state, solve_ik, set_joint_angles, get_ee_position, etc.
```

## Step 9: Add Vision

```python
from core.vision import VisionPipeline, VisionTarget

pipeline = VisionPipeline(config)
pipeline.add_target(VisionTarget("marker", position_3d=(0.1, 0.1, 0.1)))
pipeline.enable()
results = pipeline.process_frame(image)
```

---

## Next Steps

- [Adding a New Robot →](tutorials/new_robot.md)
- [Calibration →](tutorials/calibration.md)
- [API Reference →](api/core.md)
