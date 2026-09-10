import numpy as np
from typing import List, Optional, Dict, Any, Tuple
from core.configuration import RobotConfiguration
from core.kinematics import ForwardKinematics, InverseKinematics
from core.validation import SafetyValidator
from core.trajectory import TrajectoryPlanner


class CalibrationTool:
    def __init__(self, robot_config: RobotConfiguration):
        self.robot = robot_config
        self.fk = ForwardKinematics(robot_config)
        self.ik = InverseKinematics(robot_config)
        self.safety = SafetyValidator(robot_config)
        self.planner = TrajectoryPlanner(robot_config)
        self._calibration_points: List[Dict] = []
        self._offset_data: Dict[str, float] = {}

    def collect_calibration_point(self, joint_angles_deg: List[float],
                                   measured_ee_position: Tuple[float, float, float]):
        errors = self.safety.validate_configuration(joint_angles_deg)
        if errors:
            raise ValueError(f"Invalid calibration angles: {'; '.join(errors)}")

        predicted = self.fk.compute(joint_angles_deg)
        predicted_pos = (predicted["x"], predicted["y"], predicted["z"])
        position_error = np.sqrt(
            (predicted_pos[0] - measured_ee_position[0])**2 +
            (predicted_pos[1] - measured_ee_position[1])**2 +
            (predicted_pos[2] - measured_ee_position[2])**2
        )

        point = {
            "joint_angles": list(joint_angles_deg),
            "measured_ee": measured_ee_position,
            "predicted_ee": predicted_pos,
            "position_error": float(position_error),
        }
        self._calibration_points.append(point)
        return point

    def compute_dh_correction(self) -> Dict[str, float]:
        if len(self._calibration_points) < 3:
            raise ValueError("Need at least 3 calibration points for DH correction")

        corrections = {}
        for i in range(len(self.robot.dh_params)):
            errors = [p["position_error"] for p in self._calibration_points]
            corrections[f"joint_{i}_offset"] = float(np.mean(errors))

        self._offset_data = corrections
        return corrections

    def apply_corrections(self):
        if not self._offset_data:
            raise ValueError("No corrections computed yet")

        for key, value in self._offset_data.items():
            print(f"  Applying correction {key}: {value:.6f}")

    def calibrate_joint(self, joint_id: int, test_angles: List[float]) -> Dict[str, Any]:
        if joint_id < 0 or joint_id >= self.robot.dof:
            raise ValueError(f"Joint ID {joint_id} out of range")

        results = []
        for angle in test_angles:
            predicted = self.fk.compute([angle if i == joint_id else 90.0 for i in range(self.robot.dof)])
            results.append({
                "angle": angle,
                "predicted_ee": (predicted["x"], predicted["y"], predicted["z"]),
            })

        offsets = []
        for r in results:
            offset = np.sqrt(r["predicted_ee"][0]**2 + r["predicted_ee"][1]**2 + r["predicted_ee"][2]**2)
            offsets.append(offset)

        avg_offset = float(np.mean(offsets))
        return {
            "joint_id": joint_id,
            "test_angles": test_angles,
            "avg_offset": avg_offset,
            "results": results,
        }

    def full_calibration(self, num_points_per_axis=5) -> Dict[str, Any]:
        calibration_results = {}

        for axis_idx in range(3):
            axis_name = ["X", "Y", "Z"][axis_idx]
            test_angles = list(np.linspace(0, 180, num_points_per_axis))
            result = self.calibrate_joint(axis_idx, test_angles)
            calibration_results[axis_name] = result

        corrections = self.compute_dh_correction()

        return {
            "robot": self.robot.name,
            "calibration_points": len(self._calibration_points),
            "dh_corrections": corrections,
            "axis_calibrations": calibration_results,
            "status": "complete",
        }

    def get_calibration_report(self) -> str:
        lines = [
            f"Calibration Report: {self.robot.name}",
            f"{'=' * 50}",
            f"Points collected: {len(self._calibration_points)}",
            f"DH Parameters:",
        ]
        for i, dh in enumerate(self.robot.dh_params):
            lines.append(f"  Joint {i}: a={dh.a:.4f}, alpha={dh.alpha:.4f}, d={dh.d:.4f}")

        if self._offset_data:
            lines.append(f"\nCorrections:")
            for k, v in self._offset_data.items():
                lines.append(f"  {k}: {v:.6f}")

        return "\n".join(lines)
