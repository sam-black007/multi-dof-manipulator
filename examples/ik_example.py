import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import InverseKinematics

print("=" * 60)
print("Inverse Kinematics Example")
print("=" * 60)

config = RobotConfiguration.load("config/robot_a.yaml")
print(f"Robot: {config.name}")
print(f"DOF: {config.dof}")

ik = InverseKinematics(config)
target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
print(f"\nTarget Pose: x={target['x']}, y={target['y']}, z={target['z']}")
print(f"              roll={target['roll']}, pitch={target['pitch']}, yaw={target['yaw']}")

try:
    result = ik.solve(target)
    print(f"\nIK Solution Found!")
    print(f"Joint Angles: {[round(a, 2) for a in result['joint_angles']]}")
    print(f"Iterations: {result['iterations']}")
    print(f"Position Error: {result['position_error']:.6f}")
    print(f"Achieved Pose: ({result['achieved_pose']['x']:.3f}, "
          f"{result['achieved_pose']['y']:.3f}, {result['achieved_pose']['z']:.3f})")
except Exception as e:
    print(f"\nIK Failed: {e}")

print("\n" + "=" * 60)
print("Multiple Solutions Example")
print("=" * 60)
solutions = ik.solve_multiple(target, num_solutions=3)
print(f"Found {len(solutions)} solution(s)")
for i, sol in enumerate(solutions):
    print(f"  Solution {i+1}: {[round(a, 2) for a in sol['joint_angles']]} "
          f"(error={sol['position_error']:.6f}, iterations={sol['iterations']})")