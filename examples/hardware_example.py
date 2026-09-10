import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics
from hardware.mock import MockHardwareInterface

print("=" * 60)
print("Hardware Mock Example")
print("=" * 60)

config = RobotConfiguration.load("config/robot_a.yaml")
print(f"Robot: {config.name}")

hw = MockHardwareInterface(config)
print("\nConnecting to mock hardware...")
hw.connect()
print(f"Connected: {hw.is_connected()}")
print(f"Firmware: {hw.get_firmware_version()}")

print(f"\nHome Position: {config.home_position}")
print("Moving to home...")
hw.move_all(config.home_position)
print(f"Current position: {hw.get_current_position()}")

print("\nMoving to joint angles [45, 90, 135, 90, 45, 90]...")
hw.move_all([45, 90, 135, 90, 45, 90])
print(f"Current position: {hw.get_current_position()}")

print("\nMoving joint 0 to 180...")
hw.move_joint(0, 180)
print(f"Current position: {hw.get_current_position()}")

print("\nEmergency stop...")
hw.emergency_stop()
print(f"Current position after E-STOP: {hw.get_current_position()}")

hw.disconnect()
print("\nDisconnected.")