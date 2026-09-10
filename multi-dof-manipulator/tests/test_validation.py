import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.validation import SafetyValidator


def test_validate_single_joint_valid():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    ok, msg = validator.validate_single_joint_valid(0, 90)
    assert ok == True


def test_validate_single_joint_invalid():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    ok, msg = validator.validate_single_joint_invalid(0, 200)
    assert ok == False


def test_validate_full_configuration_valid():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    ok, msg = validator.validate_full_configuration_valid([90, 90, 90, 90, 90, 90])
    assert ok == True


def test_validate_full_configuration_invalid():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    ok, msg = validator.validate_full_configuration_invalid([200, 200, 200, 200, 200, 200])
    assert ok == False


def test_validate_wrong_joint_count():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    ok, msg = validator.validate_wrong_joint_count([90, 90, 90])
    assert ok == False


def test_trajectory_validation():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    trajectory = [[90, 90, 90, 90, 90, 90], [45, 45, 45, 45, 45, 45]]
    ok, msg = validator.validate_trajectory(trajectory, max_velocity=[10]*6)
    assert ok == True


def test_trajectory_velocity_violation():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    trajectory = [[90, 90, 90, 90, 90, 90], [200, 90, 90, 90, 90, 90]]
    ok, msg = validator.validate_trajectory(trajectory, max_velocity=[10]*6)
    assert ok == False


def test_reachability():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    ok, msg = validator.validate_target_reachability(0.2, 0.15, 0.25)
    assert ok == True


def test_reachability_unreachable():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    ok, msg = validator.validate_target_reachability(5.0, 5.0, 5.0)
    assert ok == False


def test_emergency_stop():
    validator = SafetyValidator(RobotConfiguration.load("config/robot_a.yaml"))
    result = validator.emergency_stop()
    assert result["status"] == "EMERGENCY_STOP"


def test_mock_hardware():
    from hardware.mock import MockHardwareInterface
    from core.configuration import RobotConfiguration
    config = RobotConfiguration.load("config/robot_a.yaml")
    hw = MockHardwareInterface(config)
    hw.connect()
    hw.move_all([90, 90, 90, 90, 90, 90])
    assert hw.is_connected()
    hw.emergency_stop()
    hw.disconnect()


def test_mock_hardware_emergency_stop():
    from hardware.mock import MockHardwareInterface
    from core.configuration import RobotConfiguration
    config = RobotConfiguration.load("config/robot_a.yaml")
    hw = MockHardwareInterface(config)
    hw.connect()
    hw.emergency_stop()
    assert hw.get_current_position() == [0.0] * 6
    hw.disconnect()