import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.validation import SafetyValidator
from hardware.mock import MockHardwareInterface


def test_validate_single_joint_valid():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    ok, msg = validator.validate_joint_position(0, 90)
    assert ok == True


def test_validate_single_joint_invalid():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    ok, msg = validator.validate_joint_position(0, -10)
    assert ok == False


def test_validate_full_configuration_valid():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    valid = [90, 90, 90, 90, 90, 90]
    ok, msg = validator.validate_joint_configuration(valid)
    assert ok == True


def test_validate_full_configuration_invalid():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    invalid = [200, 200, 200, 200, 200, 200]
    ok, msg = validator.validate_joint_configuration(invalid)
    assert ok == False


def test_validate_wrong_joint_count():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    ok, msg = validator.validate_joint_configuration([90, 90])
    assert ok == False
    assert "6" in msg


def test_trajectory_validation():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    trajectory = [[90, 90, 90, 90, 90, 90], [100, 100, 100, 100, 100, 100]]
    ok, msg = validator.validate_trajectory(trajectory)
    assert ok == True


def test_trajectory_velocity_violation():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    max_vel = [5, 5, 5, 5, 5, 5]
    trajectory = [[0, 0, 0, 0, 0, 0], [100, 100, 100, 100, 100, 100]]
    ok, msg = validator.validate_trajectory(trajectory, max_velocity=max_vel)
    assert ok == False


def test_reachability():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    ok, msg = validator.validate_target_reachability(0.3, 0.3, 0.3)
    assert ok == True


def test_reachability_unreachable():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    ok, msg = validator.validate_target_reachability(10, 10, 10)
    assert ok == False


def test_emergency_stop():
    config = RobotConfiguration.load("config/robot_a.yaml")
    validator = SafetyValidator(config)
    result = validator.emergency_stop()
    assert result["status"] == "EMERGENCY_STOP"


def test_mock_hardware():
    config = RobotConfiguration.load("config/robot_a.yaml")
    hw = MockHardwareInterface(config)
    assert hw.connect() == True
    assert hw.is_connected() == True
    hw.move_joint(0, 90)
    pos = hw.get_current_position()
    assert pos[0] == 90.0
    hw.move_all([45, 45, 45, 45, 45, 45])
    pos = hw.get_current_position()
    assert pos == [45.0, 45.0, 45.0, 45.0, 45.0, 45.0]
    hw.emergency_stop()
    hw.disconnect()