import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.trajectory import TrajectoryPlanner


def test_joint_space_trajectory():
    planner = TrajectoryPlanner(RobotConfiguration.load("config/robot_a.yaml"))
    traj = planner.joint_space_trajectory([90, 90, 90, 90, 90, 90], [45, 45, 45, 45, 45, 45], num_points=5)
    assert len(traj) == 5


def test_joint_space_trajectory_smooth():
    planner = TrajectoryPlanner(RobotConfiguration.load("config/robot_a.yaml"))
    traj = planner.joint_space_trajectory([90, 90, 90, 90, 90, 90], [45, 45, 45, 45, 45, 45], num_points=5, method="smooth")
    assert len(traj) == 5


def test_cartesian_trajectory():
    planner = TrajectoryPlanner(RobotConfiguration.load("config/robot_a.yaml"))
    traj = planner.cartesian_trajectory({"x": 0.3, "y": 0, "z": 0.2}, {"x": 0.2, "y": 0.15, "z": 0.25}, num_points=5)
    assert len(traj) == 5


def test_smooth_move():
    angles = TrajectoryPlanner.smooth_move(90, 45, num_steps=10)
    assert len(angles) == 10


def test_trapezoidal_velocity_profile():
    angles = TrajectoryPlanner.trapezoidal_velocity_profile(0, 90, max_vel=90)
    assert len(angles) >= 10