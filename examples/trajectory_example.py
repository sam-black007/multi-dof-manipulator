import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.trajectory import TrajectoryPlanner

print("=" * 60)
print("Trajectory Generation Example")
print("=" * 60)

config = RobotConfiguration.load("config/robot_a.yaml")
planner = TrajectoryPlanner(config)

print("\n--- Joint-Space Trajectory ---")
start = [90, 90, 90, 90, 90, 90]
end = [45, 45, 45, 45, 45, 45]
traj = planner.joint_space_trajectory(start, end, num_points=5)
for i, point in enumerate(traj):
    print(f"  Point {i}: {[round(a, 1) for a in point]}")

print("\n--- Smooth Joint-Space Trajectory ---")
traj_smooth = planner.joint_space_trajectory(start, end, num_points=5, method="smooth")
for i, point in enumerate(traj_smooth):
    print(f"  Point {i}: {[round(a, 1) for a in point]}")

print("\n--- S-Curve Trajectory ---")
traj_sc = planner.joint_space_trajectory(start, end, num_points=5, method="s_curve")
for i, point in enumerate(traj_sc):
    print(f"  Point {i}: {[round(a, 1) for a in point]}")

print("\n--- Cartesian Trajectory ---")
start_pose = {"x": 0.3, "y": 0.2, "z": 0.3, "roll": 0, "pitch": 0, "yaw": 0}
end_pose = {"x": 0.2, "y": 0.1, "z": 0.2, "roll": 0, "pitch": 0, "yaw": 0}
cart_traj = planner.cartesian_trajectory(start_pose, end_pose, num_points=3)
for i, pose in enumerate(cart_traj):
    print(f"  Point {i}: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")

print("\n--- Smooth Move ---")
angles = planner.smooth_move(0, 90, num_steps=5)
print(f"  Angles: {[round(a, 1) for a in angles]}")