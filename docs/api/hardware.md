# Hardware API Reference

## `hardware.base` — Hardware Interface

### `HardwareInterface` (ABC)

Abstract base class for all hardware interfaces. Implement this for real hardware drivers.

```python
from hardware.base import HardwareInterface

class MyHardware(HardwareInterface):
    def connect(self): ...
    def disconnect(self): ...
    def is_connected(self): ...
    def move_joint(self, joint_id: int, angle: float): ...
    def move_all(self, angles: list): ...
    def get_current_position(self): ...
    def emergency_stop(self): ...
    def set_speed(self, joint_id: int, speed: float): ...
    def get_firmware_version(self): ...
```

---

## `hardware.mock` — Mock Hardware

### `MockHardwareInterface`

Mock hardware for testing without physical equipment.

```python
from hardware.mock import MockHardwareInterface

hw = MockHardwareInterface(robot_config, num_joints=6)
hw.connect()
hw.move_all([90, 45, 90, 45, 90, 45])
positions = hw.get_current_position()
hw.emergency_stop()
hw.disconnect()
```

**Methods:**
- `connect()` / `disconnect()` / `is_connected()`
- `move_joint(joint_id, angle)` — Move single joint
- `move_all(angles)` — Move all joints simultaneously
- `get_current_position()` — Returns list of current angles
- `emergency_stop()` — Resets all joints to 0
- `set_speed(joint_id, speed)` — Set joint speed
- `get_firmware_version()` — Returns mock version string
