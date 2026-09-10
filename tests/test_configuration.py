import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration, JointSpec, DHParameterSet


def test_invalid_dof():
    with pytest.raises(Exception):
        RobotConfiguration(name="Bad", dof=0, joints=[], dh_params=[])


def test_missing_joints():
    with pytest.raises(Exception):
        RobotConfiguration(name="Bad", dof=6, joints=[], dh_params=[])


def test_mismatched_dh():
    with pytest.raises(Exception):
        config = RobotConfiguration(
            name="Bad", dof=6,
            joints=[JointSpec("j", 0, 180) for _ in range(6)],
            dh_params=[DHParameterSet(0, 0, 0) for _ in range(3)]
        )


def test_save_and_load():
    import tempfile, os
    config = RobotConfiguration.load("config/robot_a.yaml")
    tmp = tempfile.mktemp(suffix=".yaml")
    config.save(tmp)
    loaded = RobotConfiguration.load(tmp)
    assert loaded.name == config.name
    assert loaded.dof == config.dof
    os.unlink(tmp)


def test_from_dict():
    data = {
        "name": "Test", "dof": 6,
        "joints": [{"name": "j", "min_angle": 0, "max_angle": 180} for _ in range(6)],
        "dh_params": [{"a": 0, "alpha": 0, "d": 0.1, "theta_offset": 0} for _ in range(6)],
        "home_position": [90]*6,
        "units": "degrees"
    }
    config = RobotConfiguration.from_dict(data)
    assert config.name == "Test"


def test_to_dict():
    config = RobotConfiguration.load("config/robot_a.yaml")
    d = config.to_dict()
    assert d["name"] == "Reference 6-DOF Arm"
    assert d["dof"] == 6