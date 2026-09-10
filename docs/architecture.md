# Architecture

## Design Philosophy

This framework follows a **hardware-first** architecture. The core control algorithms are independent of any simulation environment. Simulation adapters wrap the same core code rather than replacing it.

## High-Level Architecture

```
┌─────────────────────────────────────────────────┐
│               Application Layer                  │
│  (User scripts, GUI, ROS2 nodes)                │
├─────────────────────────────────────────────────┤
│           Network Layer (WebSocket)              │
│  RobotServer, WebSocketInterface                 │
├─────────────────────────────────────────────────┤
│          Safety State Machine                    │
│  DISABLED → IDLE → ARMED → MOVING              │
│       → EMERGENCY_STOP → FAULT                  │
├─────────────────────────────────────────────────┤
│         Motion / Trajectory Layer                │
│  TrajectoryPlanner                               │
│  Joint-space and Cartesian paths                 │
├─────────────────────────────────────────────────┤
│              Kinematics Layer                    │
│  ForwardKinematics                               │
│  InverseKinematics (numerical Jacobian)          │
├─────────────────────────────────────────────────┤
│           Robot Configuration                    │
│  RobotConfiguration, JointSpec, DHParameterSet   │
│  YAML-based, multi-robot support                 │
├─────────────────────────────────────────────────┤
│          Hardware Interface                      │
│  HardwareInterface (abstract)                    │
│  ├── MockHardwareInterface (testing)             │
│  ├── ArduinoInterface (real hardware)            │
│  └── ...                                         │
├─────────────────────────────────────────────────┤
│           Simulation Adapters                    │
│  SimulationAdapter (base)                        │
│  ├── OmniSimAdapter                              │
│  ├── GazeboAdapter                               │
│  └── IsaacSimAdapter                             │
├─────────────────────────────────────────────────┤
│           Vision Integration                     │
│  VisionPipeline, PoseEstimator                   │
└─────────────────────────────────────────────────┘
```

## Module Responsibilities

| Module | Responsibility |
|--------|---------------|
| `core.transforms` | DH transforms, rotation matrices, pose conversion |
| `core.configuration` | Robot parameters, joint limits, DH table |
| `core.kinematics` | FK computation, IK solving, multiple solutions |
| `core.validation` | Safety checks, limit validation, reachability |
| `core.trajectory` | Path planning, interpolation methods |
| `core.safety` | State machine, limits, fault handling |
| `core.network` | WebSocket server, remote control |
| `core.vision` | Camera pipeline, target detection, pose estimation |
| `hardware` | Hardware abstraction (mock, Arduino) |
| `simulation` | Simulation adapters (OmniSim, Gazebo, Isaac) |
| `examples` | Usage examples, calibration tools |
| `tests` | Automated test suite |

## Data Flow

1. **User command** → Safety state machine validates state
2. **Safety check** → Passes → Trajectory planner generates path
3. **Trajectory** → FK/IK layer computes joint angles
4. **Joint angles** → Hardware interface sends to motor
5. **Feedback** → Joint positions fed back to state machine
6. **Vision** → Camera data processed for correction

## Configuration-Driven Design

Adding a new robot requires only:
1. A YAML configuration file (`config/robot_new.yaml`)
2. A URDF file (`urdf/robot_new.urdf`)

No code changes needed in `core/`, `hardware/`, or `simulation/`.

## Simulation Architecture

All simulation adapters inherit from `SimulationAdapter`, which wraps the **same** `ForwardKinematics`, `InverseKinematics`, and `TrajectoryPlanner` instances used by hardware. This ensures simulation-to-hardware consistency.

```
Simulation Adapter
    │
    ├── Uses same RobotConfiguration
    ├── Uses same ForwardKinematics
    ├── Uses same InverseKinematics
    └── Uses same TrajectoryPlanner
```

## Safety State Machine

```
DISABLED ──system_start──→ IDLE ──arm──→ ARMED ──move──→ MOVING
                                          │                │
                                     emergency_stop    fault_detected
                                          │                │
                                          ↓                ↓
                                     EMERGENCY_STOP ←── FAULT
                                          │
                                     reset ──→ IDLE
                                     fault_cleared ──→ IDLE
```

## ROS2 Integration

The framework includes `ros2_control` compatibility through the `RobotServer` WebSocket interface, allowing integration with ROS2 control stacks.
