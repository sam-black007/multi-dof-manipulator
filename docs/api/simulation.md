# Simulation API Reference

## `simulation` — Simulation Adapters

### `SimulationAdapter`

Base class for all simulation environments. Uses the same core kinematics as hardware.

```python
from simulation import SimulationAdapter

sim = SimulationAdapter()
sim.initialize(config)
sim.set_joint_angles([90, 45, 90, 45, 90, 45])
pos = sim.get_ee_position()
sim.solve_ik(target_pose)
sim.simulate_trajectory(trajectory)
sim.reset()
```

### `OmniSimAdapter`

OmniSim-compatible adapter (extends `SimulationAdapter`).

```python
from simulation import OmniSimAdapter

sim = OmniSimAdapter()
sim.initialize(config)
```

---

## `simulation.gazebo` — Gazebo Adapter

### `GazeboAdapter`

Gazebo simulation adapter with physics integration.

```python
from simulation.gazebo import GazeboAdapter

sim = GazeboAdapter()
sim.initialize(config)
sim.set_joint_angles([90, 45, 90, 45, 90, 45])
sim.pause_physics()
sim.unpause_physics()
sim.step_simulation(dt=0.01)
info = sim.get_gazebo_info()
```

### `GazeboInterface`

Low-level Gazebo interface for model and joint control.

```python
from simulation.gazebo import GazeboInterface

gz = GazeboInterface()
gz.connect("robot_model")
gz.set_joint_position("joint_0", 1.57)
gz.set_model_pose(0, 0, 0.5, 0, 0, 0)
```

**Key Features:**
- Joint position control
- Model pose setting
- Physics pause/unpause
- Step simulation
- Joint state querying

---

## `simulation.isaac` — Isaac Sim Adapter

### `IsaacSimAdapter`

NVIDIA Isaac Sim adapter with advanced sensor integration.

```python
from simulation.isaac import IsaacSimAdapter

sim = IsaacSimAdapter(headless=True)
sim.initialize(config)
sim.set_joint_angles([90, 45, 90, 45, 90, 45])
sim.enable_camera()
sim.step_simulation(dt=1/60.0)
info = sim.get_isaac_info()
```

### `IsaacInterface`

Low-level Isaac Sim interface.

```python
from simulation.isaac import IsaacInterface

isaac = IsaacInterface()
isaac.connect("IsaacLab")
isaac.set_joint_position("joint_0", 1.57)
isaac.set_prim_path("/World/Robot")
sensor_data = isaac.get_sensor_data("camera")
```

**Key Features:**
- RGB camera with configurable FOV and resolution
- 2D LiDAR ranging
- Force/torque sensor
- Sensor configuration management
- Headless mode support
- Physics stepping at configurable rates
