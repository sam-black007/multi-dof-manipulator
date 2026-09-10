# Contributing

## Development Workflow

1. Fork the repository
2. Create a feature branch
3. Make changes
4. Run tests: `pytest tests/`
5. Run validation: `python examples/validate_robot.py`
6. Submit a Pull Request

## Adding a New Robot

1. Create `config/robot_new.yaml` with:
   - Robot name and DOF
   - Joint specifications (limits, velocities, pins)
   - DH parameters (a, alpha, d, theta_offset)
   - Home position
   - Base/tool frames

2. Create `urdf/robot_new.urdf` with the URDF description

3. The core framework automatically supports the new robot
   without any code changes to kinematics, trajectory, or validation modules

## Code Style

- Use Python 3.8+
- Use type hints where appropriate
- Follow existing code structure
- Add tests for new features

## Safety

- All code must validate joint limits before execution
- Never send commands outside physical limits
- Use mock hardware for testing
- Physical hardware testing requires safety precautions