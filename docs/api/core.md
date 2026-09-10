# Core API Reference

## `core.configuration` — Robot Configuration

### `RobotConfiguration`

Main configuration class for a 6-DOF robot arm. All robot-specific parameters live in YAML files.

```python
from core.configuration import RobotConfiguration, load_robot

config = load_robot("config/robot_a.yaml")
# or
config = RobotConfiguration(name="My Arm", dof=6, joints=..., dh_params=...)
```

**Key Methods:**
- `get_dh_table()` — Returns DH parameter table as list of tuples
- `get_joint_angles_rad(deg)` — Convert degrees to radians
- `get_joint_angles_deg(rad)` — Convert radians to degrees
- `validate_configuration(angles_deg)` — Returns list of validation errors
- `clamp_configuration(angles_deg)` — Clamps angles to joint limits
- `is_reachable(x, y, z)` — Check if target is reachable
- `save(filepath)` — Save configuration to YAML

### `JointSpec`

Defines individual joint properties: limits, velocity, acceleration, servo ID, pin.

### `DHParameterSet`

Denavit-Hartenberg parameters: `a`, `alpha`, `d`, `theta_offset`.

---

## `core.kinematics` — Forward and Inverse Kinematics

### `ForwardKinematics`

Computes end-effector pose from joint angles.

```python
from core.kinematics import ForwardKinematics

fk = ForwardKinematics(config)
pose = fk.compute([90, 90, 90, 90, 90, 90])
# Returns: {"x", "y", "z", "roll", "pitch", "yaw"}
```

### `InverseKinematics`

Numerical Jacobian-based IK solver with damped least squares.

```python
from core.kinematics import InverseKinematics

ik = InverseKinematics(config, max_iterations=500, tolerance=1e-3)
result = ik.solve({"x": 0.2, "y": 0.15, "z": 0.25})
# Returns: {"joint_angles", "iterations", "position_error", "achieved_pose"}

# Multiple solutions
solutions = ik.solve_multiple(target, num_solutions=3)
```

**Parameters:**
- `max_iterations` — Maximum IK iterations (default 500)
- `tolerance` — Convergence tolerance (default 1e-3)
- `position_tolerance` — Position tolerance for solution (default 0.15)
- `orientation_tolerance` — Orientation tolerance (default 0.05)

---

## `core.transforms` — Homogeneous Transforms

Utility functions for 4x4 homogeneous transformation matrices.

```python
from core.transforms import rotation_x, rotation_y, rotation_z, translation
from core.transforms import homogeneous_transform, compute_forward_kinematics
from core.transforms import transform_to_pose, pose_to_transform

# Rotation matrices
Rz = rotation_z(np.pi/2)  # 90-degree rotation around Z

# Homogeneous transform from DH params
T = homogeneous_transform((a, alpha, d, theta))

# FK computation
T = compute_forward_kinematics(dh_table, joint_angles_rad)

# Pose conversion
pose = transform_to_pose(T)  # 4x4 matrix → {x,y,z,roll,pitch,yaw}
T = pose_to_transform(pose)  # {x,y,z,roll,pitch,yaw} → 4x4 matrix
```

---

## `core.validation` — Safety Validation

### `SafetyValidator`

Pre-execution validation for joint limits, trajectory constraints, and reachability.

```python
from core.validation import SafetyValidator

validator = SafetyValidator(config)
ok, msg = validator.validate_joint_configuration([90, 90, 90, 90, 90, 90])
ok, msg = validator.validate_trajectory(trajectory, max_velocity=[...])
ok, msg = validator.validate_target_reachability(x, y, z)
```

---

## `core.trajectory` — Trajectory Planning

### `TrajectoryPlanner`

Generates joint-space and Cartesian trajectories with multiple interpolation methods.

```python
from core.trajectory import TrajectoryPlanner

planner = TrajectoryPlanner(config)
traj = planner.joint_space_trajectory(start, end, num_points=50, method="smooth")
traj = planner.cartesian_trajectory(start_pose, end_pose)
angles = planner.trapezoidal_velocity_profile(start, end, max_vel=90)
```

**Methods:** `linear`, `smooth` (cosine), `s_curve` (S-curve), default linear

---

## `core.exceptions` — Custom Exceptions

- `RobotError` — Base exception
- `KinematicsError` — FK/IK errors
- `ValidationError` — Safety validation errors
- `ConfigurationError` — Invalid configuration
- `HardwareError` — Hardware interface errors
- `TrajectoryError` — Trajectory planning errors

---

## `core.safety` — Safety State Machine

### `SafetyStateMachine`

Manages robot operational states: DISABLED → IDLE → ARMED → MOVING → EMERGENCY_STOP → FAULT

```python
from core.safety import SafetyStateMachine, SafetyState

sm = SafetyStateMachine()
sm.arm()           # DISABLED → IDLE → ARMED
sm.move()          # ARMED → MOVING
sm.emergency_stop()  # → EMERGENCY_STOP
sm.reset()         # EMERGENCY_STOP → IDLE
```

### `SafetyLimits`

Configurable velocity, acceleration, force, and temperature limits.

---

## `core.network` — WebSocket Interface

### `WebSocketInterface`

JSON-based WebSocket server for remote robot control.

```python
from core.network import WebSocketInterface

ws = WebSocketInterface(host="0.0.0.0", port=8765)
ws.register_handler("solve_ik", my_ik_handler)
ws.start()
```

### `RobotServer`

Pre-configured server with handlers for all robot operations.

```python
from core.network import RobotServer

server = RobotServer(config)
server.start()
```

**Commands:** `get_state`, `get_joint_angles`, `set_joint_angles`, `solve_ik`, `get_ee_position`, `get_kinematics_info`

---

## `core.vision` — Vision Integration

### `VisionPipeline`

Camera-based target detection and 3D position estimation.

```python
from core.vision import VisionPipeline, VisionTarget

pipeline = VisionPipeline(config)
pipeline.add_target(VisionTarget("marker", position_3d=(0.1, 0.1, 0.1)))
pipeline.enable()
results = pipeline.process_frame(image)
```

### `PoseEstimator`

Estimates robot pose from vision results.

---

## `core.safety.safety_state_machine` — Safety State Machine

### `SafetyState` Enum

- `DISABLED` — System powered off
- `IDLE` — Ready for commands
- `ARMED` — Armed for movement
- `MOVING` — Currently executing trajectory
- `EMERGENCY_STOP` — Emergency stop activated
- `FAULT` — Error condition

### `SafetyEvent` Enum

Events that trigger state transitions: `SYSTEM_START`, `ARM_COMMAND`, `MOVE_COMMAND`, `STOP_COMMAND`, `EMERGENCY_PRESSED`, `FAULT_DETECTED`, `FAULT_CLEARED`, `RESET_COMMAND`, `LIMIT_EXCEEDED`, `COLLISION_DETECTED`

---

## `core.safety.safety_limits` — Safety Limits

### `SafetyLimits`

Configurable physical limits for safe operation:

- `max_joint_velocity` (default 180°/s)
- `max_joint_acceleration` (default 360°/s²)
- `max_cartesian_velocity` (default 0.5 m/s)
- `max_cartesian_acceleration` (default 1.0 m/s²)
- `max_force` (default 50 N)
- `temperature_threshold` (default 80°C)
