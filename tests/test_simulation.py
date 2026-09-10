import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from simulation import OmniSimAdapter


def test_omnisim_init():
    config = RobotConfiguration.load("config/robot_a.yaml")
    sim = OmniSimAdapter()
    sim.initialize(config)
    assert sim.is_running() == True


def test_omnisim_set_get_angles():
    config = RobotConfiguration.load("config/robot_a.yaml")
    sim = OmniSimAdapter()
    sim.initialize(config)
    angles = [90, 45, 90, 45, 90, 45]
    sim.set_joint_angles(angles)
    assert sim.get_joint_angles() == angles


def test_omnisim_ee_position():
    config = RobotConfiguration.load("config/robot_a.yaml")
    sim = OmniSimAdapter()
    sim.initialize(config)
    pos = sim.get_ee_position()
    assert len(pos) == 3


def test_omnisim_solve_ik():
    config = RobotConfiguration.load("config/robot_a.yaml")
    sim = OmniSimAdapter()
    sim.initialize(config)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    result = sim.solve_ik(target)
    assert len(result["joint_angles"]) == 6


def test_omnisim_reset():
    config = RobotConfiguration.load("config/robot_a.yaml")
    sim = OmniSimAdapter()
    sim.initialize(config)
    sim.set_joint_angles([45, 45, 45, 45, 45, 45])
    sim.reset()
    assert sim.get_joint_angles() == config.home_position


def test_omnisim_simulate_trajectory():
    config = RobotConfiguration.load("config/robot_a.yaml")
    sim = OmniSimAdapter()
    sim.initialize(config)
    trajectory = [[90, 90, 90, 90, 90, 90], [45, 45, 45, 45, 45, 45]]
    assert sim.simulate_trajectory(trajectory) == True


def test_omnisim_not_initialized():
    sim = OmniSimAdapter()
    with pytest.raises(RuntimeError):
        sim.get_ee_position()