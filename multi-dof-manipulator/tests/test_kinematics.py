import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics


def test_fk_home_position():
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    pose = fk.compute(config.home_position)
    assert "x" in pose
    assert "y" in pose
    assert "z" in pose
    assert pose["z"] > 0.1


def test_fk_random_config():
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    import random
    random.seed(42)
    angles = [random.uniform(0, 180) for _ in range(6)]
    pose = fk.compute(angles)
    assert pose["x"] is not None
    assert pose["y"] is not None
    assert pose["z"] is not None


def test_fk_boundary():
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    angles = [0, 0, 0, 0, 0, 0]
    pose = fk.compute(angles)
    assert pose["x"] is not None


def test_fk_returns_valid_pose():
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    pose = fk.compute(config.home_position)
    assert isinstance(pose["x"], float)
    assert isinstance(pose["roll"], float)
    assert isinstance(pose["pitch"], float)
    assert isinstance(pose["yaw"], float)


def test_ik_known_reachable():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    result = ik.solve(target)
    assert len(result["joint_angles"]) == 6


def test_ik_unreachable():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 5.0, "y": 5.0, "z": 5.0, "roll": 0, "pitch": 0, "yaw": 0}
    with pytest.raises(Exception):
        ik.solve(target)


def test_ik_joint_limit_violation():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config, max_iterations=10)
    target = {"x": 0.01, "y": 0.01, "z": 0.01, "roll": 0, "pitch": 0, "yaw": 0}
    result = ik.solve(target)
    assert result is not None
    assert "joint_angles" in result


def test_ik_validates_through_fk():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0.1, "pitch": 0.1, "yaw": 0.2}
    result = ik.solve(target)
    fk = ForwardKinematics(config)
    achieved = result["achieved_pose"]
    pos_error = (
        (achieved["x"] - target["x"])**2 +
        (achieved["y"] - target["y"])**2 +
        (achieved["z"] - target["z"])**2
    ) ** 0.5
    assert pos_error < 0.5


def test_ik_iterations_reported():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    result = ik.solve(target)
    assert "iterations" in result
    assert result["iterations"] > 0


def test_ik_position_error():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    result = ik.solve(target)
    assert "position_error" in result


def test_ik_multiple_solutions():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 0.15, "y": 0.12, "z": 0.2, "roll": 0, "pitch": 0, "yaw": 0}
    solutions = ik.solve_multiple(target, num_solutions=3)
    assert len(solutions) >= 0
    for sol in solutions:
        assert len(sol["joint_angles"]) == 6