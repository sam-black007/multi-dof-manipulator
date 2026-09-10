import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics

print("=" * 60)
print("Forward Kinematics Example")
print("=" * 60)

config = RobotConfiguration.load("config/robot_a.yaml")
print(f"Robot: {config.name}")
print(f"DOF: {config.dof}")
print(f"Home Position: {config.home_position}")

fk = ForwardKinematics(config)
angles = config.home_position
print(f"\nJoint Angles: {angles}")
pose = fk.compute(angles)
print(f"End-Effector Position: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")
print(f"End-Effector Orientation: roll={pose['roll']:.3f}, pitch={pose['pitch']:.3f}, yaw={pose['yaw']:.3f}")

print("\n" + "=" * 60)
print("Random Configuration FK")
print("=" * 60)
import random
random.seed(42)
rand_angles = [random.uniform(0, 180) for _ in range(6)]
print(f"Random Angles: {rand_angles}")
pose = fk.compute(rand_angles)
print(f"End-Effector Position: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")