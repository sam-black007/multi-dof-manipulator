import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics
from hardware.mock import MockHardwareInterface


def test_fk_ik_consistency():
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    ik = InverseKinematics(config)
    import random
    random.seed(42)
    for _ in range(5):
        angles = [random.uniform(0, 180) for _ in range(6)]
        pose = fk.compute(angles)
        result = ik.solve(pose)
        fk2 = ForwardKinematics(config)
        achieved = result["achieved_pose"]
        pos_diff = (
            (achieved["x"] - pose["x"])**2 +
            (achieved["y"] - pose["y"])**2 +
            (achieved["z"] - pose["z"])**2
        ) ** 0.5
        assert pos_diff < 0.5


def test_ik_convergence_rate():
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config, max_iterations=100)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0.1, "pitch": 0.1, "yaw": 0.2}
    result = ik.solve(target)
    assert result["iterations"] <= 100
    assert result["iterations"] > 0


def test_mock_hardware_integration():
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    ik = InverseKinematics(config)
    hw = MockHardwareInterface(config)
    hw.connect()
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    result = ik.solve(target)
    hw.move_all(result["joint_angles"])
    hw_pos = hw.get_current_position()
    assert len(hw_pos) == 6
    for i in range(6):
        assert abs(hw_pos[i] - result["joint_angles"][i]) < 0.01
    hw.disconnect()


def test_mock_hardware_emergency_stop():
    config = RobotConfiguration.load("config/robot_a.yaml")
    hw = MockHardwareInterface(config)
    hw.connect()
    hw.move_all([90, 90, 90, 90, 90, 90])
    hw.emergency_stop()
    pos = hw.get_current_position()
    assert pos == [0, 0, 0, 0, 0, 0]
    hw.disconnect()


def test_ik_benchmark():
    import time
    config = RobotConfiguration.load("config/robot_a.yaml")
    ik = InverseKinematics(config)
    target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
    start_time = time.time()
    result = ik.solve(target)
    elapsed = time.time() - start_time
    assert elapsed < 10.0


def test_fk_benchmark():
    import time
    config = RobotConfiguration.load("config/robot_a.yaml")
    fk = ForwardKinematics(config)
    angles = [90, 90, 90, 90, 90, 90]
    start_time = time.time()
    pose = fk.compute(angles)
    elapsed = time.time() - start_time
    assert elapsed < 1.0


def test_orientation_error():
    import sys
    from core.transforms import pose_difference
    pose_a = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
    pose_b = {"x": 0, "y": 0, "z": 0, "roll": 0.01, "pitch": 0.01, "yaw": 0.01}
    pos_diff, orient_diff = pose_difference(pose_a, pose_b)
    assert orient_diff < 0.02


def test_are_poses_equal():
    from core.transforms import are_poses_equal
    pose_a = {"x": 0.3, "y": 0.2, "z": 0.3, "roll": 0, "pitch": 0, "yaw": 0}
    pose_b = {"x": 0.301, "y": 0.201, "z": 0.301, "roll": 0.005, "pitch": 0.005, "yaw": 0.005}
    assert are_poses_equal(pose_a, pose_b) == True