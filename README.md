# Universal 6-DOF Robotic Arm Framework

A reusable, hardware-oriented software framework for controlling and testing different 6-DOF robotic arms.

![License](https://img.shields.io/badge/License-MIT-green.svg)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

---

## What This Project Is

A universal framework for controlling 6-DOF robotic arms. The primary validation target is **real robotic-arm hardware**. Simulation (including OmniSim) is treated as an additional validation and development environment.

The same core software can be adapted to different 6-DOF robotic arms by changing robot-specific configuration files rather than rewriting the control and kinematics system.

---

## Architecture

```
USER / APPLICATION
      ↓
High-Level Robot API
      ↓
Motion / Trajectory Layer
      ↓
Kinematics Layer
      ↓
Robot Configuration
      ↓
Hardware Interface
      ↓
Real Hardware / Simulation / Mock

Simulation Adapter (separate)
      ↓
Same Core Kinematics / Motion / Configuration
```

### Key Design Principles

1. **Hardware-First**: The control system does not assume a simulator
2. **Configuration-Driven**: Robot-specific parameters live in YAML files, not code
3. **Simulation as Validation**: Simulation uses the same core algorithms as hardware
4. **Multi-Robot**: Add a new 6-DOF arm by adding a config file + URDF

---

## Features

- **Forward Kinematics**: Computes end-effector pose from joint angles using DH parameters
- **Inverse Kinematics**: Computes joint angles from target pose with reachability checking
- **FK/IK Validation**: Every IK result is verified through FK
- **Joint-Limit Validation**: Centralized joint limit system with configurable limits
- **Trajectory Generation**: Joint-space and Cartesian trajectories with smooth interpolation
- **Hardware Abstraction**: Mock hardware for testing, real hardware interface for deployment
- **Robot Configuration**: YAML-based configuration for any 6-DOF arm
- **Simulation Adapter**: OmniSim adapter for validation
- **Safety System**: Pre-execution validation, emergency stop, limit checking
- **Multi-Robot Support**: Add new robots via configuration files only
- **Automated Tests**: Comprehensive test suite covering all modules

---

## Architecture Diagram

```
core/                    # Generic framework
├── transforms.py        # Homogeneous transforms, rotation matrices
├── configuration.py     # RobotConfiguration, JointSpec, DHParameterSet
├── kinematics.py        # ForwardKinematics, InverseKinematics
├── validation.py        # SafetyValidator
├── trajectory.py        # TrajectoryPlanner
└── exceptions.py        # Custom exceptions

config/                  # Robot configurations
├── robot_a.yaml         # Reference 6-DOF Arm
└── robot_b.yaml         # Test 6-DOF Arm

robots/                  # Robot-specific resources
└── robot_a/
    └── urdf/            # URDF files

hardware/                # Hardware abstraction
├── base.py             # HardwareInterface (abstract)
├── mock.py             # MockHardwareInterface (for testing)
└── arduino.py          # Arduino hardware bridge

simulation/              # Simulation adapter
└── omnisim_adapter.py  # OmniSim compatibility

tests/                   # Automated tests
examples/                # Example scripts
urdf/                    # URDF robot descriptions
Arduino/                 # Original Arduino firmware (preserved)
```

---

## Adding a New 6-DOF Robot

Adding a new 6-DOF robot requires **no changes to the core framework**:

1. Create `config/robot_new.yaml`:
   - Define robot name, DOF, joint limits, DH parameters
   - Specify servo IDs, pins, offsets, directions
   - Define home position and frames

2. Create `urdf/robot_new.urdf`:
   - Define links, joints, inertial properties
   - Standard URDF format

3. The core kinematics, trajectory, and validation automatically support the new robot.

### Example Configuration Snippet

```yaml
name: "My 6-DOF Arm"
dof: 6
home_position: [90, 90, 90, 90, 90, 90]
joints:
  - name: "Base"
    min_angle: 0
    max_angle: 180
    servo_id: 1
    pin: 2
dh_params:
  - a: 0.0
    alpha: 1.5708
    d: 0.15
```

---

## Hardware Setup

### Current Reference Hardware (Robot A)

| Component | Quantity |
|-----------|----------|
| Arduino Mega 2560 | 1 |
| DS3235 35KG Servo (J1-J3) | 3 |
| DS3218 20KG Servo (J4-J5) | 2 |
| MG996R Servo (J6) | 1 |
| MG90S Servo (Gripper) | 1 |
| HC-05 Bluetooth Module | 1 |
| 6V 20A Power Supply | 1 |

| Joint | Function | Pin |
|-------|----------|-----|
| J1 | Base Rotation | D2 |
| J2 | Shoulder | D3 |
| J3 | Elbow | D4 |
| J4 | Wrist Pitch | D5 |
| J5 | Wrist Roll | D6 |
| J6 | Tool Rotation | D7 |
| Gripper | End Effector | D8 |

> **⚠️ Safety Warning**: Never power servo motors directly from the Arduino Mega. Use an external 6V 20A power supply. All grounds must be connected together.

### Status Labels

- **Implemented**: Core framework is functional
- **Tested in simulation**: Kinematics and validation tested with mock hardware
- **Tested on hardware**: Pending physical testing
- **Experimental**: Features under development

---

## Simulation (OmniSim)

OmniSim is supported as an **additional validation environment**, not the core architecture.

The `OmniSimAdapter` uses the **same robot configuration and kinematic assumptions** as the hardware implementation. This ensures consistency between simulation and real-world behavior.

### Using the Simulation Adapter

```python
from core.configuration import RobotConfiguration
from simulation import OmniSimAdapter

config = RobotConfiguration.load("config/robot_a.yaml")
sim = OmniSimAdapter()
sim.initialize(config)
sim.set_joint_angles([90, 90, 90, 90, 90, 90])
ee_pos = sim.get_ee_position()
```

---

## Running Tests

```bash
# Install dependencies
pip install -r requirements.txt

# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_kinematics.py -v

# Run with coverage
pytest tests/ --cov=core --cov=hardware --cov=simulation
```

### Test Coverage

- **Configuration**: Valid/invalid robot configs, joint limits, DOF validation
- **FK**: Home position, random configurations, boundary configurations
- **IK**: Known reachable poses, unreachable poses, joint-limit violations, multiple solutions
- **FK/IK Consistency**: Random → FK → IK → FK → Compare
- **Joint Limits**: Valid positions, below minimum, above maximum
- **Trajectory**: Valid/invalid trajectories, velocity/acceleration violations
- **Hardware**: Mock hardware integration, emergency stop, position tracking
- **Simulation**: OmniSim adapter initialization, IK solving, trajectory simulation

---

## Safety

> **⚠️ Physical Hardware Testing Must Be Performed With Appropriate Safety Precautions**

The framework includes software-side safety checks, but physical testing requires:

1. **Physical barriers** around the work area
2. **Emergency stop** mechanism accessible at all times
3. **Gradual testing** starting from small movements
4. **Supervised operation** at all times
5. **Proper power supply** with fuse protection
6. **Never run autonomous programs** without human oversight

The framework **will not automatically send unsafe commands** to hardware. It fails safely by raising exceptions.

---

## Running the Validation Script

```bash
python examples/validate_robot.py
```

This demonstrates:
1. Loading robot configuration
2. Running FK
3. Running IK
4. Validating IK using FK
5. Checking joint limits
6. Generating trajectory
7. Running against mock hardware
8. Running in simulation

---

## Example Scripts

| Script | Description |
|--------|-------------|
| `examples/fk_example.py` | Forward kinematics demonstration |
| `examples/ik_example.py` | Inverse kinematics demonstration |
| `examples/trajectory_example.py` | Trajectory generation examples |
| `examples/hardware_example.py` | Mock hardware control example |
| `examples/simulation_example.py` | OmniSim simulation example |
| `examples/validate_robot.py` | Complete robot validation |
| `examples/multi_robot_example.py` | Multi-robot demonstration |

---

## Dependencies

- Python 3.8+
- NumPy (matrix operations, transformations)
- PyYAML (configuration loading)
- pytest (testing)

---

## Repository Structure

```
multi-dof-manipulator/
├── README.md                  # This file
├── LICENSE                    # MIT License
├── CHANGELOG.md               # Version history
├── CONTRIBUTING.md            # Contributing guide
├── requirements.txt           # Python dependencies
├── setup.py                   # Package setup
├── pyproject.toml             # Project configuration
├── core/                      # Generic framework
│   ├── __init__.py
│   ├── transforms.py
│   ├── configuration.py
│   ├── kinematics.py
│   ├── validation.py
│   ├── trajectory.py
│   └── exceptions.py
├── config/                    # Robot configurations
│   ├── robot_a.yaml
│   └── robot_b.yaml
├── hardware/                  # Hardware abstraction
│   ├── __init__.py
│   ├── base.py
│   ├── mock.py
│   └── arduino.py
├── simulation/                # Simulation adapter
│   ├── __init__.py
│   └── omnisim_adapter.py
├── tests/                     # Automated tests
│   ├── __init__.py
│   ├── test_kinematics.py
│   ├── test_configuration.py
│   ├── test_validation.py
│   ├── test_trajectory.py
│   ├── test_ik_validation.py
│   └── test_simulation.py
├── examples/                  # Example scripts
│   ├── fk_example.py
│   ├── ik_example.py
│   ├── trajectory_example.py
│   ├── hardware_example.py
│   ├── simulation_example.py
│   ├── validate_robot.py
│   └── multi_robot_example.py
├── urdf/                      # URDF files
│   ├── robot_a.urdf
│   └── robot_b.urdf
├── robots/                    # Robot-specific resources
│   └── robot_a/
│       └── urdf/
├── docs/                      # Documentation and images
└── Arduino/                   # Original Arduino firmware
    ├── robotic-arm-6dof.ino
    ├── config/
    ├── joints/
    ├── kinematics/
    ├── motion/
    ├── communication/
    ├── gripper/
    └── safety/
```

---

## Development Roadmap

### Implemented
- ✅ Universal framework architecture
- ✅ Forward kinematics with DH parameters
- ✅ Inverse kinematics with numerical solver
- ✅ FK/IK validation pipeline
- ✅ Joint-limit validation system
- ✅ Trajectory generation
- ✅ Hardware abstraction layer
- ✅ Mock hardware for testing
- ✅ OmniSim simulation adapter
- ✅ Two robot configurations
- ✅ URDF files
- ✅ Automated test suite
- ✅ Example scripts
- ✅ Validation script

### Testing on Hardware
- ⬜ Physical Arduino Mega 2560 testing
- ⬜ Servo motor validation
- ⬜ Bluetooth communication testing
- ⬜ Real-world reachability verification

### Simulation Validation
- ⬜ OmniSim integration testing
- ⬜ Simulation-to-hardware consistency validation

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Support

If you found this project useful, please star the repository!

---

> **⚠️ Reminder**: Physical hardware testing must be performed with appropriate safety precautions. This software has been tested in simulation only.
