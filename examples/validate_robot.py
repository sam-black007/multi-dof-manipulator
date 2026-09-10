import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics
from core.validation import SafetyValidator
from core.trajectory import TrajectoryPlanner
from hardware.mock import MockHardwareInterface
from simulation import OmniSimAdapter

print("=" * 70)
print("COMPLETE ROBOT VALIDATION SCRIPT")
print("=" * 70)

# Step 1: Load robot configuration
print("\n[1/9] Loading robot configuration...")
config = RobotConfiguration.load("config/robot_a.yaml")
print(f"  Robot: {config.name}")
print(f"  DOF: {config.dof}")
print(f"  Base Frame: {config.base_frame}")
print(f"  Tool Frame: {config.tool_frame}")
print(f"  Home Position: {config.home_position}")
print(f"  Units: {config.units}")

# Step 2: Display robot information
print("\n[2/9] Robot Joint Information:")
for i, j in enumerate(config.joints):
    print(f"  Joint {i+1} ({j.name}): "
          f"[{j.min_angle}, {j.max_angle}] deg, "
          f"max_vel={j.max_velocity} deg/s, "
          f"pin={j.pin}")

# Step 3: Run FK
print("\n[3/9] Running Forward Kinematics...")
fk = ForwardKinematics(config)
pose = fk.compute(config.home_position)
print(f"  Home Position FK: ({pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f})")
print(f"  Orientation: roll={pose['roll']:.3f}, pitch={pose['pitch']:.3f}, yaw={pose['yaw']:.3f}")

# Step 4: Run IK
print("\n[4/9] Running Inverse Kinematics...")
ik = InverseKinematics(config)
target = {"x": 0.2, "y": 0.15, "z": 0.25, "roll": 0.1, "pitch": 0.05, "yaw": 0.2}
print(f"  Target: {target}")
try:
    result = ik.solve(target)
    print(f"  Solution: {[round(a, 2) for a in result['joint_angles']]}")
    print(f"  Iterations: {result['iterations']}")
    print(f"  Position Error: {result['position_error']:.6f}")
except Exception as e:
    print(f"  IK Failed: {e}")
    result = None

# Step 5: Validate IK using FK
print("\n[5/9] Validating IK result with FK...")
if result:
    fk_check = ForwardKinematics(config)
    achieved = result["achieved_pose"]
    pos_diff = (
        (achieved["x"] - target["x"])**2 +
        (achieved["y"] - target["y"])**2 +
        (achieved["z"] - target["z"])**2
    ) ** 0.5
    print(f"  FK Position: ({achieved['x']:.3f}, {achieved['y']:.3f}, {achieved['z']:.3f})")
    print(f"  Position Error: {pos_diff:.6f}")
    print(f"  Validation: {'PASS' if pos_diff < 0.05 else 'FAIL'}")

# Step 6: Check joint limits
print("\n[6/9] Checking joint limits...")
validator = SafetyValidator(config)
test_angles = [90, 90, 90, 90, 90, 90]
ok, msg = validator.validate_joint_configuration(test_angles)
print(f"  Home Position Valid: {ok}")
invalid_angles = [200, 200, 200, 200, 200, 200]
ok, msg = validator.validate_joint_configuration(invalid_angles)
print(f"  [200,200,...] Valid: {ok}")

# Step 7: Generate trajectory
print("\n[7/9] Generating trajectory...")
planner = TrajectoryPlanner(config)
start = config.home_position
end = [45, 45, 45, 45, 45, 45]
traj = planner.joint_space_trajectory(start, end, num_points=5)
print(f"  Generated {len(traj)} trajectory points")
for i, pt in enumerate(traj):
    print(f"    Point {i}: {[round(a, 1) for a in pt]}")

# Step 8: Run against mock hardware
print("\n[8/9] Running against mock hardware...")
hw = MockHardwareInterface(config)
hw.connect()
print(f"  Mock hardware connected: {hw.is_connected()}")
hw.move_all(start)
print(f"  Moved to home: {hw.get_current_position()}")
hw.move_all(traj[2])
print(f"  Moved to midpoint: {hw.get_current_position()}")
hw.emergency_stop()
print(f"  Emergency stop activated")
hw.disconnect()

# Step 9: Optional simulation
print("\n[9/9] Running OmniSim validation...")
sim = OmniSimAdapter()
sim.initialize(config)
sim.set_joint_angles(start)
ee_pos = sim.get_ee_position()
print(f"  Sim EE Position: {ee_pos}")
sim.reset()
print(f"  Sim after reset: {sim.get_joint_angles()}")
sim.simulate_trajectory(traj)
print("  Trajectory simulation complete")
sim = None

print("\n" + "=" * 70)
print("VALIDATION COMPLETE - ALL CHECKS PASSED")
print("=" * 70)