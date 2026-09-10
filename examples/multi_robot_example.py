import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics

print("=" * 60)
print("Multi-Robot Example")
print("=" * 60)

# Load both robot configurations
print("\n[1/3] Loading Robot A (Reference)...")
config_a = RobotConfiguration.load("config/robot_a.yaml")
print(f"  Robot A: {config_a.name}, DOF={config_a.dof}")

print("\n[2/3] Loading Robot B (Test)...")
config_b = RobotConfiguration.load("config/robot_b.yaml")
print(f"  Robot B: {config_b.name}, DOF={config_b.dof}")
print(f"  Robot B Shoulder max: {config_b.joints[1].max_angle}")
print(f"  Robot A Shoulder max: {config_a.joints[1].max_angle}")

# Same core framework, different configurations
print("\n[3/3] Running FK on both robots with same angles...")
angles = [90, 90, 90, 90, 90, 90]

fk_a = ForwardKinematics(config_a)
pose_a = fk_a.compute(angles)
print(f"\n  Robot A FK: ({pose_a['x']:.3f}, {pose_a['y']:.3f}, {pose_a['z']:.3f})")

fk_b = ForwardKinematics(config_b)
pose_b = fk_b.compute(angles)
print(f"  Robot B FK: ({pose_b['x']:.3f}, {pose_b['y']:.3f}, {pose_b['z']:.3f})")

print(f"\n  Same angles produce different poses because the robots")
print(f"  have different link lengths (DH parameters).")

# Run IK on both robots
print("\nRunning IK on both robots...")
target = {"x": 0.3, "y": 0.2, "z": 0.3, "roll": 0, "pitch": 0, "yaw": 0}

ik_a = InverseKinematics(config_a)
try:
    result_a = ik_a.solve(target)
    print(f"  Robot A IK: {[round(a, 2) for a in result_a['joint_angles']]}")
except Exception as e:
    print(f"  Robot A IK: Failed - {e}")

ik_b = InverseKinematics(config_b)
try:
    result_b = ik_b.solve(target)
    print(f"  Robot B IK: {[round(a, 2) for a in result_b['joint_angles']]}")
except Exception as e:
    print(f"  Robot B IK: Failed - {e}")

print("\n" + "=" * 60)
print("CONCLUSION: Same core framework works with different robots")
print("=" * 60)