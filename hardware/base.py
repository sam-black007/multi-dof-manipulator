from abc import ABC, abstractmethod


class HardwareInterface(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def is_connected(self):
        pass

    @abstractmethod
    def move_joint(self, joint_id: int, angle: float):
        pass

    @abstractmethod
    def move_all(self, angles: list):
        pass

    @abstractmethod
    def get_current_position(self):
        pass

    @abstractmethod
    def emergency_stop(self):
        pass

    @abstractmethod
    def set_speed(self, joint_id: int, speed: float):
        pass

    @abstractmethod
    def get_firmware_version(self):
        pass