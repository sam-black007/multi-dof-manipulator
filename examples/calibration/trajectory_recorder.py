import json
import time
import csv
from typing import List, Optional, Dict, Any, Tuple
from datetime import datetime
import numpy as np


class TrajectoryRecorder:
    def __init__(self, robot_config=None):
        self.robot_config = robot_config
        self._trajectories: List[Dict] = []
        self._current_trajectory: Optional[Dict] = None
        self._start_time = None

    def start_recording(self, name: str = "trajectory",
                        description: str = "",
                        tags: Optional[List[str]] = None):
        self._current_trajectory = {
            "name": name,
            "description": description,
            "tags": tags or [],
            "start_time": datetime.now().isoformat(),
            "timestamps": [],
            "joint_angles": [],
            "ee_positions": [],
            "metadata": {},
        }
        self._start_time = time.time()
        return self._current_trajectory["name"]

    def record_point(self, joint_angles: List[float],
                     ee_position: Optional[Tuple[float, float, float]] = None):
        if self._current_trajectory is None:
            raise RuntimeError("No trajectory recording in progress")

        self._current_trajectory["timestamps"].append(time.time() - self._start_time)
        self._current_trajectory["joint_angles"].append(list(joint_angles))
        if ee_position is not None:
            self._current_trajectory["ee_positions"].append(list(ee_position))

    def stop_recording(self) -> Dict:
        if self._current_trajectory is None:
            raise RuntimeError("No trajectory recording in progress")

        self._current_trajectory["end_time"] = datetime.now().isoformat()
        self._current_trajectory["duration"] = time.time() - self._start_time
        self._current_trajectory["num_points"] = len(
            self._current_trajectory["joint_angles"]
        )
        self._trajectories.append(self._current_trajectory)
        traj = self._current_trajectory
        self._current_trajectory = None
        return traj

    def get_trajectory(self, index: int) -> Dict:
        return self._trajectories[index]

    def get_all_trajectories(self) -> List[Dict]:
        return list(self._trajectories)

    def save_to_json(self, filepath: str):
        data = {
            "robot": self.robot_config.name if self.robot_config else "unknown",
            "created": datetime.now().isoformat(),
            "trajectories": self._trajectories,
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

    def save_to_csv(self, filepath: str):
        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "joint_0", "joint_1", "joint_2",
                           "joint_3", "joint_4", "joint_5",
                           "ee_x", "ee_y", "ee_z"])

            for traj in self._trajectories:
                angles = traj["joint_angles"]
                ee = traj["ee_positions"]
                for i, a in enumerate(angles):
                    row = [traj["timestamps"][i]] if i < len(traj["timestamps"]) else [0]
                    row.extend(a)
                    if i < len(ee):
                        row.extend(ee[i])
                    else:
                        row.extend([0, 0, 0])
                    writer.writerow(row)

    def load_from_json(self, filepath: str):
        with open(filepath, "r") as f:
            data = json.load(f)
        self._trajectories = data.get("trajectories", [])
        return self._trajectories

    def get_average_duration(self) -> float:
        if not self._trajectories:
            return 0.0
        return sum(t.get("duration", 0) for t in self._trajectories) / len(self._trajectories)

    def get_total_points(self) -> int:
        return sum(len(t.get("joint_angles", [])) for t in self._trajectories)

    def filter_by_duration(self, min_duration: float = 0) -> List[Dict]:
        return [t for t in self._trajectories if t.get("duration", 0) >= min_duration]

    def get_trajectory_summary(self) -> str:
        lines = [
            f"Trajectory Summary",
            f"{'=' * 50}",
            f"Total trajectories: {len(self._trajectories)}",
            f"Total points: {self.get_total_points()}",
            f"Average duration: {self.get_average_duration():.3f}s",
        ]
        for i, t in enumerate(self._trajectories):
            lines.append(
                f"  [{i}] {t['name']}: {t.get('num_points', 0)} pts, "
                f"{t.get('duration', 0):.3f}s"
            )
        return "\n".join(lines)
