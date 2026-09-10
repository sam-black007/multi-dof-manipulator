import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration, JointSpec, DHParameterSet


def test_invalid_dof():
    from core.configuration import ConfigurationError
    try:
        RobotConfiguration(dof=0)
        assert False, "Should have raised ConfigurationError"
    except ConfigurationError:
        pass


def test_missing_joints():
    from core.configuration import ConfigurationError
    try:
        RobotConfiguration(dof=6, joints=[])
        assert False, "Should have raised ConfigurationError"
    except ConfigurationError:
        pass


def test_mismatched_dh():
    from core.configuration import ConfigurationError
    try:
        RobotConfiguration(dof=6, joints=[None]*6, dh_params=[None]*5)
        assert False, "Should have raised ConfigurationError"
    except ConfigurationError:
        pass


def test_save_and_load():
    config = RobotConfiguration.load("config/robot_a.yaml")
    data = config.to_dict()
    config2 = RobotConfiguration.from_dict(data)
    assert config2.name == config.name
    assert config2.dof == config.dof


def test_to_dict():
    config = RobotConfiguration.load("config/robot_a.yaml")
    d = config.to_dict()
    assert "name" in d
    assert "dof" in d
    assert "dh_params" in d


def test_from_dict():
    data = {
        "name": "Test Robot",
        "dof": 6,
        "joints": [
            {"name": "J1", "min_angle": 0, "max_angle": 180, "min_velocity": 0, "max_velocity": 100,
             "min_acceleration": 0, "max_acceleration": 200, "offset": 0, "direction": 1,
             "servo_id": 1, "pin": 2}
            for _ in range(6)
        ],
        "dh_params": [
            {"a": 0.0, "alpha": 1.5708, "d": 0.1, "theta_offset": 0.0}
            for _ in range(6)
        ]
    }
    config = RobotConfiguration.from_dict(data)
    assert config.name == "Test Robot"
    assert config.dof == 6