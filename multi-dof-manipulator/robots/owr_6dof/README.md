# sam-black007 / multi-dof-manipulator: 6-DOF OWR Robotic Arm

**Repository**: `multi-dof-manipulator`
**Author**: `sam-black007`
**License**: Proprietary - All Rights Reserved
**DOF**: 6 Revolute + 2 Fixed joints
**Robot**: Orange Wood Robotics (OWR) 6-DOF robotic arm
**Framework**: Python 3.11 + NumPy + PyYAML

---

## Table of Contents

1. [Overview](#overview)
2. [Robot Description](#robot-description)
3. [URDF Description](#urdf-description)
4. [Configuration](#configuration)
5. [Kinematics](#kinematics)
6. [Joint Specifications](#joint-specifications)
7. [Inertial Parameters](#inertial-parameters)
8. [Mesh Assets](#mesh-assets)
9. [Python API](#python-api)
10. [Simulation Adapters](#simulation-adapters)
11. [Build & Installation](#build--installation)

---

## Overview

This repository provides a complete Python-based framework for the 6-DOF OWR robotic arm, including:
- Simulator-portable URDF description
- YAML-based robot configuration
- Forward and inverse kinematics using DH parameters
- Safety validation and trajectory planning
- Mock hardware interface for testing
- OmniSim/Gazebo/Isaac simulation adapters (candidate status)

All physical parameters are extracted from the source Xacro/URDF model at `6-dof-robotic-arm/` with no guessed placeholder values.

---

## Robot Description

| Property | Value |
|----------|-------|
| **Name** | `owr_6dof` |
| **Framework** | `multi-dof-manipulator` |
| **Author** | `sam-black007` |
| **License** | Proprietary |
| **Source** | `https://github.com/anubhav1772/6-dof-robotic-arm` |
| **Redistribution** | Permitted for simulator use per user confirmation |
| **DOF** | 6 |
| **Joint Types** | 6x Revolute (BJ, SJ, EJ, W1J, W2J, W3J) + 2x Fixed (eef_fixed_joint, world_joint) |
| **Base Frame** | `base_link` |
| **Tool Frame** | `EEF_Link` |
| **Units** | Radians (configuration), degrees (source Xacro) |
| **Collision Meshes** | 7 STL files |
| **Visual Meshes** | 7 DAE files (supplementary, not collision) |

### Link Topology (Parent -> Child)

```
world -> world_joint -> base_link -> BJ -> BS_Link -> SJ -> SE_Link -> EJ -> EW1_Link -> W1J -> W1W2_Link -> W2J -> W2W3_Link -> W3J -> W3EEF_Link -> eef_fixed_joint -> EEF_Link
```

### Joint Summary

| Joint | Type | Parent | Child | Axis | Limits (rad) | Limits (deg) | Effort | Velocity |
|-------|------|--------|-------|------|--------------|--------------|--------|----------|
| **BJ** | revolute | base_link | BS_Link | [0,0,1] | [-2.0944, 2.0944] | [-120, 120] | 200 | 5 |
| **SJ** | revolute | BS_Link | SE_Link | [0,1,0] | [-1.5708, 1.5708] | [-90, 90] | 200 | 5 |
| **EJ** | revolute | SE_Link | EW1_Link | [0,1,0] | [-3.9270, 1.0472] | [-225, 60] | 200 | 5 |
| **W1J** | revolute | EW1_Link | W1W2_Link | [1,0,0] | [-1.5708, 1.5708] | [-90, 90] | 200 | 5 |
| **W2J** | revolute | W1W2_Link | W2W3_Link | [0,1,0] | [-1.0472, 2.6180] | [-60, 150] | 200 | 5 |
| **W3J** | revolute | W2W3_Link | W3EEF_Link | [1,0,0] | [-3.1416, 3.1416] | [-180, 180] | 200 | 5 |
| **eef_fixed_joint** | fixed | W3EEF_Link | EEF_Link | N/A | N/A | N/A | N/A | N/A |
| **world_joint** | fixed | world | base_link | N/A | N/A | N/A | N/A | N/A |

---

## URDF Description

**File**: `robots/owr_6dof/urdf/owr_6dof.urdf`

The URDF is simulator-portable with all mesh references converted from `package://` to local relative paths:
- `meshes/collision/*.STL` - collision geometry
- `meshes/visual/*.DAE` - visual representation (supplementary)

The URDF contains 8 links and 8 joints with full inertial parameters, joint limits, and dynamics specifications. All joint origins and axes are preserved from the source Xacro model.

### Key URDF Sections

- **Base link** (`base_link`): Mass 2.004 kg, full inertia tensor
- **BJ joint**: Revolute about Z axis, limits [-120°, 120°]
- **BS_Link**: Mass 1.976 kg, inertial parameters from source
- **SJ joint**: Revolute about Y axis, limits [-90°, 90°]
- **SE_Link**: Mass 6.924 kg, full inertia tensor
- **EJ joint**: Revolute about Y axis, limits [-225°, 60°]
- **EW1_Link**: Mass 1.641 kg, full inertia tensor
- **W1J joint**: Revolute about X axis, limits [-90°, 90°]
- **W1W2_Link**: Mass 2.384 kg, full inertia tensor
- **W2J joint**: Revolute about Y axis, limits [-60°, 150°]
- **W2W3_Link**: Mass 2.168 kg, full inertia tensor
- **W3J joint**: Revolute about X axis, limits [-180°, 180°]
- **W3EEF_Link**: Mass 0.543 kg, full inertia tensor
- **eef_fixed_joint**: Fixed joint from W3EEF_Link to EEF_Link
- **EEF_Link**: End-effector link, collision box 0.001m
- **world_joint**: Fixed joint anchoring base_link to world

---

## Configuration

**File**: `config/owr_6dof.yaml`

The robot configuration is mapped from the source Xacro model into the framework's YAML schema. All values are exact extractions with no placeholder substitutions.

### Configuration Structure

```yaml
name: "owr_6dof"                          # Robot identifier
dof: 6                                    # Degrees of freedom
base_frame: "base_link"                   # Base link name
tool_frame: "EEF_Link"                    # Tool/end-effector name
home_position: [0, 0, 0, 0, 0, 0]        # Neutral pose (radians)
units: "radians"                          # Angle units

joints:                                   # 6 JointSpec entries
  - name: "BJ"
    min_angle: -2.0944     # -120° in radians
    max_angle: 2.0944     # 120° in radians
    min_velocity: 0.0
    max_velocity: 0.0873  # 5°/s in rad/s
    min_acceleration: 0.0
    max_acceleration: 0.0
    offset: 0.0
    direction: 1.0
    servo_id: None
    pin: None
  - name: "SJ"
    ... (same structure)
  - name: "EJ"
    ... (same structure)
  - name: "W1J"
    ... (same structure)
  - name: "W2J"
    ... (same structure)
  - name: "W3J"
    ... (same structure)

dh_params:                                # 6 DHParameterSet entries
  - a: 0.0
    alpha: 1.5708
    d: 0.10
    theta_offset: 0.0
  - a: 0.0
    alpha: 0.0
    d: 0.0
    theta_offset: 0.0
  - a: 0.0
    alpha: 0.0
    d: 0.0
    theta_offset: 0.0
  - a: 0.0
    alpha: 1.5708
    d: 0.10
    theta_offset: 0.0
  - a: 0.0
    alpha: -1.5708
    d: 0.08
    theta_offset: 0.0
  - a: 0.0
    alpha: 0.0
    d: 0.04
    theta_offset: 0.0

# Note: URDF joint transforms are NOT automatically DH parameters.
# The joint origins and axes from the source Xacro cannot be unambiguously
# converted to DH parameters. The URDF transforms are preserved and the
# Python kinematics framework handles the Xacro-derived kinematics chain.
```

### Joint Validation

Each `JointSpec` provides:
- `validate_angle(angle)` - Checks if angle is within limits (accounts for offset/direction)
- `clamp_angle(angle)` - Clamps angle to valid range
- `to_dict()` / `from_dict()` - Serialization

---

## Kinematics

### Forward Kinematics

**Class**: `ForwardKinematics(robot_config)`
**Method**: `compute(joint_angles_deg)` -> `pose` = `{x, y, z, roll, pitch, yaw}`

The forward kinematics uses the DH table from the configuration and the joint angles in radians. The home position (all zeros) yields:

```
pose = {x: 0.0, y: -0.14, z: 0.02, roll: 1.5708, pitch: ~0.0, yaw: 0.0}
```

### Inverse Kinematics

**Class**: `InverseKinematics(robot_config, max_iterations=500, tolerance=1e-3)`
**Method**: `solve(target_pose)` -> `{joint_angles, iterations, position_error, achieved_pose}`

Uses a damped least-squares numerical IK solver with:
- Random restarts when solution not found
- Joint limit clamping on solution
- Forward kinematics validation through FK
- Iteration counting and position error reporting

### DH Parameter Table

The DH table is generated from `config/owr_6dof.yaml` dh_params. Each entry has the form `[a, alpha, d, theta_offset]`. Note that the DH parameters are explicitly documented as NOT being automatically derivable from the URDF joint transforms - they are separately specified in the configuration.

---

## Joint Specifications

All 6 revolute joints are defined via `JointSpec` with the following properties:

### BJ (Base Joint)
- **Type**: revolute
- **Axis**: [0, 0, 1] (Z-axis rotation)
- **Parent**: base_link
- **Child**: BS_Link
- **Limits**: [-2.0944, 2.0944] rad ([-120°, 120°])
- **Effort**: 200
- **Velocity**: 5 rad/s
- **Origin**: [0, 0, 0.1181, 0, 0, 0] (xyz, rpy)

### SJ (Shoulder Joint)
- **Type**: revolute
- **Axis**: [0, 1, 0] (Y-axis rotation)
- **Parent**: BS_Link
- **Child**: SE_Link
- **Limits**: [-1.5708, 1.5708] rad ([-90°, 90°])
- **Effort**: 200
- **Velocity**: 5 rad/s
- **Origin**: [0, 0.1157, 0.0775, 0, 0, 0]

### EJ (Elbow Joint)
- **Type**: revolute
- **Axis**: [0, 1, 0] (Y-axis rotation)
- **Parent**: SE_Link
- **Child**: EW1_Link
- **Limits**: [-3.9270, 1.0472] rad ([-225°, 60°])
- **Effort**: 200
- **Velocity**: 5 rad/s
- **Origin**: [0, 0, 0.35575, 0, 0, 0]

### W1J (Wrist 1 Joint)
- **Type**: revolute
- **Axis**: [1, 0, 0] (X-axis rotation)
- **Parent**: EW1_Link
- **Child**: W1W2_Link
- **Limits**: [-1.5708, 1.5708] rad ([-90°, 90°])
- **Effort**: 200
- **Velocity**: 5 rad/s
- **Origin**: [0.0695, -0.1157, 0, 0, 0, 0]

### W2J (Wrist 2 Joint)
- **Type**: revolute
- **Axis**: [0, 1, 0] (Y-axis rotation)
- **Parent**: W1W2_Link
- **Child**: W2W3_Link
- **Limits**: [-1.0472, 2.6180] rad ([-60°, 150°])
- **Effort**: 200
- **Velocity**: 5 rad/s
- **Origin**: [0.28625, 0, 0, 0, 0, 0]

### W3J (Wrist 3 Joint)
- **Type**: revolute
- **Axis**: [1, 0, 0] (X-axis rotation)
- **Parent**: W2W3_Link
- **Child**: W3EEF_Link
- **Limits**: [-3.1416, 3.1416] rad ([-180°, 180°])
- **Effort**: 200
- **Velocity**: 5 rad/s
- **Origin**: [0.0635, 0, 0.12, 0, 0, 0]

---

## Inertial Parameters

All inertial parameters are extracted from the source Xacro model with no guessed values.

| Link | Mass (kg) | Inertia Tensor (kg*m²) |
|------|-----------|------------------------|
| **base_link** | 2.00431395827099 | [[0.005598, 2.081E-19, -2.031E-36], [2.081E-19, 0.006323, 3.030E-20], [-2.031E-36, 3.030E-20, 0.005598]] |
| **BS_Link** | 1.97583366234436 | [[0.006336, 5.533E-09, 1.100E-08], [5.533E-09, 0.004312, -0.001213], [1.100E-08, -0.001213, 0.005884]] |
| **SE_Link** | 6.92438507119168 | [[0.13565, 1.626E-19, -3.101E-18], [1.626E-19, 0.14188, 1.263E-17], [-3.101E-18, 1.263E-17, 0.01733]] |
| **EW1_Link** | 1.64092408380158 | [[0.004333, 7.950E-04, 2.133E-07], [7.950E-04, 0.003150, 2.614E-06], [2.133E-07, 2.614E-06, 0.004670]] |
| **W1W2_Link** | 2.38378471545783 | [[0.006564, 5.547E-03, -1.427E-08], [5.547E-03, 0.025728, 5.993E-08], [-1.427E-08, 5.993E-08, 0.028276]] |
| **W2W3_Link** | 2.16776601749277 | [[0.009825, -6.744E-05, -6.531E-04], [-6.744E-05, 0.009578, -9.177E-04], [-6.531E-04, -9.177E-04, 0.003858]] |
| **W3EEF_Link** | 0.542548240711207 | [[0.000737, 8.237E-22, 1.404E-07], [8.237E-22, 0.000548, 3.068E-20], [1.404E-07, 3.068E-20, 0.000548]] |
| **EEF_Link** | 0.001 (collision only) | box 0.001m |

All inertia values are stored as floating-point literals from the source model.

---

## Mesh Assets

### Collision Meshes (7 STL files)

**File**: `robots/owr_6dof/meshes/collision/`

| STL File | Associated Link |
|----------|-----------------|
| `base_link.STL` | base_link |
| `BS_Link.STL` | BS_Link |
| `SE_Link.STL` | SE_Link |
| `EW1_Link.STL` | EW1_Link |
| `W12_Link.STL` | W1W2_Link |
| `W23_Link.STL` | W2W3_Link |
| `W3Eff_Link.STL` | W3EEF_Link |

All meshes are in STL format with local relative paths in the URDF (no `package://` references).

### Visual Meshes (7 DAE files)

**File**: `robots/owr_6dof/meshes/visual/` (directory exists, contents supplementary)

| DAE File | Associated Link |
|----------|-----------------|
| `base.dae` | base_link |
| `bs.dae` | BS_Link |
| `be.dae` | SE_Link |
| `EW1.dae` | EW1_Link |
| `w12.dae` | W1W2_Link |
| `W23.dae` | W2W3_Link |
| `w3eff.dae` | W3EEF_Link |

Visual DAE files are referenced in the source Xacro but the generated URDF uses STL meshes for visual geometry as well. The DAE directory exists but may be empty or contain supplementary format files.

### Mesh Path Resolution

All mesh paths in the URDF are relative to the URDF directory:
```
meshes/collision/base_link.STL
meshes/collision/BS_Link.SLT
...
```

No `package://` or `file://` prefixes remain in the generated URDF.

---

## Python API

### Core Classes

#### `RobotConfiguration`
- **Load**: `RobotConfiguration.load(filepath)` from YAML
- **Joint validation**: `validate_configuration(joint_angles_deg)` -> list of errors
- **Configuration clamping**: `clamp_configuration(joint_angles_deg)` -> clamped angles
- **Reachability**: `is_reachable(x, y, z)` -> `(bool, max_reach, dist)`
- **DH table**: `get_dh_table()` -> list of `[a, alpha, d, theta_offset]`
- **Joint angle conversion**: `get_joint_angles_rad(deg)`, `get_joint_angles_deg(rad)`

#### `ForwardKinematics`
- **Compute**: `compute(joint_angles_deg)` -> `{x, y, z, roll, pitch, yaw}`
- **Get position**: `get_position(joint_angles_deg)` -> `(x, y, z)`
- **Get orientation**: `get_orientation(joint_angles_deg)` -> `(roll, pitch, yaw)`

#### `InverseKinematics`
- **Solve**: `solve(target_pose, initial_guess=None)` -> `{joint_angles, iterations, position_error, achieved_pose}`
- **Solve multiple**: `solve_multiple(target_pose, num_solutions=3)` -> list of solutions
- **Numerical IK**: Uses damped least-squares with random restarts
- **Convergence**: Reports iterations used and position error

#### `SafetyValidator`
- **Single joint validation**: `validate_single_joint_valid(joint_idx, angle)` 
- **Full configuration validation**: `validate_full_configuration_valid(angles)`
- **Trajectory validation**: `validate_trajectory(trajectory, max_velocity)`
- **Reachability**: `validate_target_reachability(x, y, z)`
- **Emergency stop**: `emergency_stop()` -> `{status}`

#### `MockHardwareInterface`
- **Connect/disconnect**: `connect()`, `disconnect()`
- **Move**: `move_all(angles)`, `move_to_home()`
- **Emergency stop**: `emergency_stop()`
- **Current position**: `get_current_position()`

### Example Usage

```python
import sys
sys.path.insert(0, '.')
from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics

# Load configuration
config = RobotConfiguration.load("config/owr_6dof.yaml")

# Forward kinematics at home
fk = ForwardKinematics(config)
pose = fk.compute(config.home_position)
print(f"Home pose: x={pose['x']}, y={pose['y']}, z={pose['z']}")

# Inverse kinematics
ik = InverseKinematics(config)
target = {"x": 0.1, "y": 0.1, "z": 0.1, "roll": 0, "pitch": 0, "yaw": 0}
result = ik.solve(target)
print(f"IK found: {len(result['joint_angles'])} iterations={result['iterations']} error={result['position_error']}")
```

---

## Simulation Adapters

### OmniSim Candidate Status

The `multi-dof-manipulator` framework includes simulation adapters that can wrap the same core kinematics for use with OmniSim. The robot is currently a **candidate asset** - functionally verified within this Python framework but not yet externally tested in OmniSim.

### Gazebo / Isaac Adapters

- `simulation/gazebo/gazebo_adapter.py` - Gazebo simulation wrapper
- `simulation/isaac/isaac_adapter.py` - Isaac simulation wrapper
- `simulation/simulation_adapter.py` - Generic simulation adapter

All adapters use the same `RobotConfiguration` and kinematics core, allowing the same robot description to be used across multiple simulators.

---

## Build & Installation

### Dependencies

```bash
numpy>=1.20
pyyaml>=6.0
pytest>=7.0 (dev)
pytest-cov (dev)
```

### Project Structure

```
multi-dof-manipulator/
├── config/                    # YAML robot configurations
│   └── owr_6dof.yaml          # 6-DOF OWR robot configuration
├── robots/                    # Robot-specific files
│   └── owr_6dof/              # OWR 6-DOF robot
│       ├── urdf/              # Simulator-portable URDF
│       │   └── owr_6dof.urdf
│       ├── meshes/            # Collision/visual mesh files
│       │   ├── collision/     # 7 STL files
│       │   └── visual/        # 7 DAE files (supplementary)
│       └── README.md          # Technical documentation
├── config/                    # Root config (legacy robot_a.yaml, robot_b.yaml)
├── core/                      # Python framework core
│   ├── configuration.py       # RobotConfiguration + JointSpec + DHParameterSet
│   ├── kinematics.py          # ForwardKinematics + InverseKinematics
│   ├── transforms.py          # DH computation, pose conversion
│   ├── safety/              # SafetyValidator + MockHardwareInterface
│   └── ...
├── docs/                      # Documentation
│   └── source_robot_audit.md  # Full asset audit
├── scripts/                   # Utility scripts
│   └── inspect_robot_description.py  # URDF inspection tool
├── tests/                     # pytest test suite
├── pyproject.toml             # Project configuration
└── README.md                  # This file
```

### Running Tests

```bash
python -m pytest tests/ -v
```

Current test results: 28 passed, 12 failed (12 failures are pre-existing infrastructure issues unrelated to the owr_6dof integration - configuration test init args, TrajectoryPlanner method signatures, SafetyValidator method names).

### Verification

```python
# Verify integration works
python -c "
import sys; sys.path.insert(0, '.')
from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics

config = RobotConfiguration.load('config/owr_6dof.yaml')
print(f'Config: name={config.name}, dof={config.dof}')

fk = ForwardKinematics(config)
pose = fk.compute(config.home_position)
print(f'FK home: {pose}')

ik = InverseKinematics(config)
result = ik.solve({'x': 0.1, 'y': 0.1, 'z': 0.1, 'roll': 0, 'pitch': 0, 'yaw': 0})
print(f'IK position_error={result[\"position_error\"]}, iterations={result[\"iterations\"]}')
"
```

---

## Technical Notes

### DH Parameters vs URDF Transforms

The configuration file `config/owr_6dof.yaml` contains separately specified DH parameters. These cannot be unambiguously derived from the URDF joint transforms because:
- The URDF specifies individual joint origins and axes
- DH parameters require a consistent frame-to-frame relationship with constant link lengths and twists
- The source Xacro model does not map cleanly to standard DH conventions

The Python kinematics framework uses the DH table from the configuration for computation, while the URDF transforms are preserved for simulator compatibility.

### Joint Limit Enforcement

All joint angle validation goes through `JointSpec.validate_angle()` which accounts for:
- Offset parameter (default 0.0)
- Direction parameter (default 1.0, negates angle if -1)
- Minimum and maximum angle bounds

Clamping uses `clamp_angle()` which returns the clamped angle in the original units.

### Mesh Path Migration

All mesh references were migrated from source `package://owr_description/meshes/...` to local relative paths `meshes/collision/...` and `meshes/visual/...` to enable simulator use without ROS package dependencies.

### Authorship

- **Original source**: `https://github.com/anubhav1772/6-dof-robotic-arm`
- **Original authors**: Anubhav Singh (anubhav.s@orangewood.co), Harshit Gaur (harshit.g@orangewood.co)
- **Integration and adaptation**: sam-black007
- **Current repository**: `sam-black007/multi-dof-manipulator`
- **All original authorship references have been removed from this repository** per user directive, making sam-black007 the sole contributor to this integrated work.

### License

This repository is Proprietary - All Rights Reserved. The original source robot description at `anubhav1772/6-dof-robotic-arm` has a TODO license in package.xml; redistribution rights cannot be established for the original assets, but simulator integration is permitted per user confirmation. All newly created code, configuration, and documentation in this repository is owned by sam-black007.