# Adding a New Robot

This guide shows how to add a new 6-DOF robot arm to the framework without modifying any core code.

## Step 1: Create the YAML Configuration

Create `config/robot_new.yaml` with your robot's parameters:

```yaml
name: "My Custom Arm"
dof: 6
base_frame: "base_link"
tool_frame: "tool_link"
home_position: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
units: "degrees"

joints:
  - name: "Base"
    min_angle: 0
    max_angle: 180
    min_velocity: 0
    max_velocity: 90
    servo_id: 1
    pin: 2
  # ... define all 6 joints

dh_params:
  - a: 0.0
    alpha: 1.5708
    d: 0.12
    theta_offset: 0.0
  # ... define all 6 DH parameter sets
```

### DH Parameter Convention

Each DH row defines `[a, alpha, d, theta_offset]`:

| Parameter | Meaning | Typical Values |
|-----------|---------|---------------|
| `a` | Link length (m) | 0.0 - 0.3 |
| `alpha` | Link twist (rad) | 0 or ±1.5708 |
| `d` | Link offset (m) | 0.0 - 0.15 |
| `theta_offset` | Initial joint offset (rad) | 0.0 |

## Step 2: Create the URDF File

Create `urdf/robot_new.urdf`:

```xml
<?xml version="1.0"?>
<robot name="robot_new">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <!-- Define all links and joints -->
</robot>
```

## Step 3: Verify the Configuration

```python
from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics

config = RobotConfiguration.load("config/robot_new.yaml")
fk = ForwardKinematics(config)
pose = fk.compute(config.home_position)
print(f"Home position: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")
```

## Step 4: Test with Simulation

```python
from simulation import OmniSimAdapter

sim = OmniSimAdapter()
sim.initialize(config)
sim.set_joint_angles(config.home_position)
print(f"Sim EE Position: {sim.get_ee_position()}")
```

## Step 5: Run Validation

```python
from examples.validate_robot import validate_robot

# Or manually:
from core.validation import SafetyValidator
from core.trajectory import TrajectoryPlanner

validator = SafetyValidator(config)
planner = TrajectoryPlanner(config)
```

## Verification Checklist

- [ ] YAML loads without errors
- [ ] FK produces valid pose
- [ ] IK converges for reachable targets
- [ ] Joint limits are enforced
- [ ] URDF matches DH parameters
- [ ] Simulation produces same FK results as hardware
- [ ] All tests pass

## Tips

- Match DH parameters to your physical robot's geometry
- Use `config.save("path")` to export configurations
- Check `config.is_reachable(x, y, z)` before planning trajectories
- The `home_position` should be a safe, collision-free pose
