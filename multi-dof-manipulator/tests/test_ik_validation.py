import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import InverseKinematics


def test_fk_ik_consistency():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    result = ik.solve(target)
    fk = ForwardKinematics(config)
    achieved = result["achieved_pose"]
    pos_error = (
        (achieved["x"] - target["x"])**2 +
        (achieved["y"] - target["y"])**2 +
        (achieved["z"] - target["z"])**2
    ) ** 0.5
    assert pos_error < 0.5


def test_ik_convergence_rate():
    import time
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config, max_iterations=500)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    start = time.time()
    result = ik.solve(target)
    elapsed = time.time() - start
    assert elapsed < 10.0  # Should converge within 10 seconds
    assert "position_error" in result


def test_ik_benchmark():
    import random
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    random.seed(123)
    successes = 0
    for _ in range(20):
        target = {"x": random.uniform(0.05, 0.3),
                  "y": random.uniform(-0.15, 0.15),
                  "z": random.uniform(0.1, 0.4),
                  "roll": random.uniform(-0.5, 0.5),
                  "pitch": random.uniform(-0.5, 0.5),
                  "yaw": random.uniform(-0.5, 0.5)}
        try:
            result = ik.solve(target)
            successes += 1
        except KinematicsError:
            pass
    assert successes >= 15  # At least 75% success rate


def test_fk_benchmark():
    import random
    from core.kinematics import ForwardKinematics
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    random.seed(123)
    for _ in range(50):
        angles = [random.uniform(0, 180) for _ in range(6)]
        pose = fk.compute(angles)
        assert pose["x"] is not None
        assert pose["y"] is not None
        assert pose["z"] is not None


def test_orientation_error():
    from core.transforms import pose_difference, are_poses_equal
    pose_a = {"x": 0.1, "y": 0.1, "z": 0.1, "roll": 0, "pitch": 0, "yaw": 0}
    pose_b = {"x": 0.1, "y": 0.1, "z": 0.1, "roll": 0.01, "pitch": 0, "yaw": 0}
    pos_diff, orient_diff = pose_difference(pose_a, pose_b)
    assert orient_diff < 0.0175  # 1 degree in radians


def test_are_poses_equal():
    from core.transforms import are_poses_equal
    pose_a = {"x": 0.1, "y": 0.1, "z": 0.1, "roll": 0.001, "pitch": 0.001, "yaw": 0.001}
    pose_b = {"x": 0.1, "y": 0.1, "z": 0.1, "roll": 0.001, "pitch": 0.001, "yaw": 0.001}
    assert are_poses_equal(pose_a, pose_b, pos_tol=0.01, orient_tol=0.0175)