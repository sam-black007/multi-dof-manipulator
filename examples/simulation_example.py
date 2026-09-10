import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics
from simulation import OmniSimAdapter

print("=" * 60)
print("Simulation Example")
print("=" * 60)

config = RobotConfiguration.load("config/robot_a.yaml")
print(f"Robot: {config.name}")

sim = OmniSimAdapter()
sim.initialize(config)
print(f"Simulation initialized: {sim.is_running()}")

print(f"\nHome EE Position: {sim.get_ee_position()}")

print("\nMoving to [45, 90, 135, 90, 45, 90]...")
sim.set_joint_angles([45, 90, 135, 90, 45, 90])
print(f"Current EE Position: {sim.get_ee_position()}")
print(f"Current Joint Angles: {sim.get_joint_angles()}")

print("\nSolving IK for target (0.25, 0.15, 0.25)...")
target = {"x": 0.25, "y": 0.15, "z": 0.25, "roll": 0, "pitch": 0, "yaw": 0}
try:
    result = sim.solve_ik(target)
    print(f"Solution: {[round(a, 2) for a in result['joint_angles']]}")
    print(f"Iterations: {result['iterations']}")
    print(f"Position Error: {result['position_error']:.6f}")
except Exception as e:
    print(f"IK Failed: {e}")

print("\nSimulating trajectory...")
trajectory = [[90, 90, 90, 90, 90, 90], [45, 45, 45, 45, 45, 45]]
sim.simulate_trajectory(trajectory)

sim.reset()
print(f"\nAfter reset: {sim.get_joint_angles()}")
print("Simulation complete.")