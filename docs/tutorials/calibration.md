# Calibration Tutorial

## Overview

Calibration ensures the kinematic model matches physical reality. The `CalibrationTool` collects measurements at known joint configurations and computes DH parameter corrections.

## Step 1: Set Up the Calibration Tool

```python
from core.configuration import RobotConfiguration
from examples.calibration.calibration_tool import CalibrationTool

config = RobotConfiguration.load("config/robot_a.yaml")
calibrator = CalibrationTool(config)
```

## Step 2: Collect Calibration Points

Move the robot to known joint configurations and measure the end-effector position:

```python
# Joint angles (degrees)
angles = [90, 45, 90, 45, 90, 45]

# Measured EE position (meters) - from camera or physical measurement
measured = (0.25, 0.15, 0.30)

point = calibrator.collect_calibration_point(angles, measured)
print(f"Position error: {point['position_error']:.6f}m")
```

Collect at least 3 points at different configurations for reliable correction.

## Step 3: Compute DH Corrections

```python
corrections = calibrator.compute_dh_correction()
for joint, offset in corrections.items():
    print(f"  {joint}: {offset:.6f}")
```

## Step 4: Apply Corrections

```python
calibrator.apply_corrections()
```

## Step 5: Full Calibration

For comprehensive calibration across all axes:

```python
results = calibrator.full_calibration(num_points_per_axis=5)
print(results["dh_corrections"])
```

## Step 6: Generate Report

```python
report = calibrator.get_calibration_report()
print(report)
```

## Trajectory Recording

Record and replay trajectories for validation:

```python
from examples.calibration.trajectory_recorder import TrajectoryRecorder

recorder = TrajectoryRecorder(config)
recorder.start_recording("test_trajectory", description="Pick and place")

# Record points during operation
recorder.record_point([90, 90, 90, 90, 90, 90], (0.3, 0, 0.2))
recorder.record_point([45, 45, 45, 45, 45, 45], (0.2, 0.1, 0.15))

trajectory = recorder.stop_recording()

# Save and load
recorder.save_to_json("trajectories/test.json")
recorder.save_to_csv("trajectories/test.csv")

# Load and replay
recorder.load_from_json("trajectories/test.json")
print(recorder.get_trajectory_summary())
```

## Tips

- Measure at configurations that exercise all joints
- Use multiple configurations for better DH correction accuracy
- Calibrate after any mechanical adjustment
- Compare simulation FK with physical measurements
- Record trajectories for regression testing
