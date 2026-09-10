import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.trajectory import TrajectoryPlanner


def test_joint_space_trajectory():
    config = RobotConfiguration.load("config/robot_a.yaml")
    planner = TrajectoryPlanner(config)
    start = [90, 90, 90, 90, 90, 90]
    end = [45, 45, 45, 45, 45, 45]
    traj = planner.joint_space_trajectory(start, end, num_points=10)
    assert len(traj) == 10
    assert traj[0] == start
    assert traj[-1] == end


def test_joint_space_trajectory_smooth():
    config = RobotConfiguration.load("config/robot_a.yaml")
    planner = TrajectoryPlanner(config)
    start = [90, 90, 90, 90, 90, 90]
    end = [45, 45, 45, 45, 45, 45]
    traj = planner.joint_space_trajectory(start, end, num_points=10, method="smooth")
    assert len(traj) == 10
    assert traj[0] == start
    assert traj[-1] == end


def test_cartesian_trajectory():
    config = RobotConfiguration.load("config/robot_a.yaml")
    planner = TrajectoryPlanner(config)
    start = {"x": 0.3, "y": 0.2, "z": 0.3, "roll": 0, "pitch": 0, "yaw": 0}
    end = {"x": 0.2, "y": 0.1, "z": 0.2, "roll": 0, "pitch": 0, "yaw": 0}
    traj = planner.cartesian_trajectory(start, end, num_points=5)
    assert len(traj) == 5
    assert traj[0]["x"] == start["x"]
    assert traj[-1]["x"] == end["x"]


def test_smooth_move():
    config = RobotConfiguration.load("config/robot_a.yaml")
    planner = TrajectoryPlanner(config)
    angles = planner.smooth_move(0, 90, num_steps=10)
    assert len(angles) == 10
    assert angles[0] == 0
    assert angles[-1] == 90
    assert angles[0] < angles[-1]


def test_invalid_start_config():
    config = RobotConfiguration.load("config/robot_a.yaml")
    planner = TrajectoryPlanner(config)
    start = [200, 200, 200, 200, 200, 200]
    end = [45, 45, 45, 45, 45, 45]
    with pytest.raises(ValueError):
        planner.joint_space_trajectory(start, end)


def test_trapezoidal_profile():
    config = RobotConfiguration.load("config/robot_a.yaml")
    planner = TrajectoryPlanner(config)
    angles = planner.trapezoidal_velocity_profile(0, 90, max_vel=45)
    assert len(angles) >= 10
    assert angles[0] == 0
    assert angles[-1] == 90