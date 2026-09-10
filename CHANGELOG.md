# CHANGELOG

## [1.0.0] - 2026-09-10

### Added
- Universal 6-DOF robotic arm framework
- Forward kinematics with DH parameters
- Inverse kinematics with numerical solver
- FK/IK validation pipeline
- Joint-limit validation system
- Trajectory planner (joint-space and Cartesian)
- Hardware abstraction layer (MockHardwareInterface)
- OmniSim simulation adapter
- Robot configuration system (YAML-based)
- Two robot configurations (robot_a, robot_b)
- URDF files for both robots
- Automated test suite
- Example scripts
- Safety validator with emergency stop
- Multi-robot support demonstration

### Changed
- Restructured from Arduino-only firmware to Python framework with Arduino support
- Separated core kinematics from hardware-specific code
- Created hardware-first architecture with simulation as validation layer

### Fixed
- Forward kinematics was a stub; now implements proper DH parameter transforms
- Inverse kinematics was a stub; now implements numerical solver with validation
- Trajectory generation was non-functional; now generates smooth trajectories
- Safety limits were hardcoded; now configurable per joint

## [0.1.0] - Initial (Arduino Firmware)

### Added
- Arduino Mega 2560 firmware
- Servo control for 6-DOF arm
- Bluetooth communication (HC-05)
- Serial command interface
- Gripper control
- Basic safety limits