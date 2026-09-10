class MockHardwareInterface:
    def __init__(self, robot_config=None, num_joints=6):
        self._connected = False
        self._current_angles = [0.0] * num_joints
        self._robot_config = robot_config
        self._num_joints = num_joints

    def connect(self):
        self._connected = True
        return True

    def disconnect(self):
        self._connected = False

    def is_connected(self):
        return self._connected

    def move_joint(self, joint_id: int, angle: float):
        if not self._connected:
            raise ConnectionError("Mock hardware not connected")
        if joint_id < 0 or joint_id >= self._num_joints:
            raise ValueError(f"Joint ID {joint_id} out of range [0, {self._num_joints-1}]")
        self._current_angles[joint_id] = angle
        return True

    def move_all(self, angles: list):
        if not self._connected:
            raise ConnectionError("Mock hardware not connected")
        if len(angles) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} angles, got {len(angles)}")
        self._current_angles = list(angles)
        return True

    def get_current_position(self):
        return list(self._current_angles)

    def emergency_stop(self):
        self._current_angles = [0.0] * self._num_joints
        return {"status": "EMERGENCY_STOP"}

    def set_speed(self, joint_id: int, speed: float):
        return True

    def get_firmware_version(self):
        return "mock-1.0.0"

    def get_current_joint_positions(self):
        return list(self._current_angles)