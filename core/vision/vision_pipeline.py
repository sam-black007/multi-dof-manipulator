import numpy as np
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass, field


@dataclass
class VisionTarget:
    name: str
    position_3d: Tuple[float, float, float]
    marker_size: float = 0.1
    expected_id: Optional[int] = None


@dataclass
class VisionResult:
    target_id: int
    position_2d: Tuple[float, float]
    confidence: float
    position_3d: Optional[Tuple[float, float, float]] = None
    timestamp: float = 0.0


class VisionPipeline:
    def __init__(self, robot_config, camera_matrix=None, dist_coeffs=None):
        self.robot_config = robot_config
        self.camera_matrix = camera_matrix or np.eye(3)
        self.dist_coeffs = dist_coeffs or np.zeros(5)
        self._targets: List[VisionTarget] = []
        self._results: List[VisionResult] = []
        self._enabled = False

    def add_target(self, target: VisionTarget):
        self._targets.append(target)

    def remove_target(self, target_id: int):
        self._targets = [t for t in self._targets if t.expected_id != target_id]

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    def is_enabled(self):
        return self._enabled

    def process_frame(self, image: np.ndarray) -> List[VisionResult]:
        if not self._enabled:
            return []

        self._results = []
        for target in self._targets:
            result = self._detect_target(image, target)
            if result:
                self._results.append(result)

        return self._results

    def _detect_target(self, image: np.ndarray, target: VisionTarget) -> Optional[VisionResult]:
        height, width = image.shape[:2]
        cx, cy = width / 2, height / 2

        projected_x = target.position_3d[0] * 100 + cx
        projected_y = target.position_3d[1] * 100 + cy

        confidence = max(0.0, min(1.0, 1.0 / (1.0 + abs(projected_x - cx) / width + abs(projected_y - cy) / height)))

        return VisionResult(
            target_id=target.expected_id or hash(target.name) % 10000,
            position_2d=(float(projected_x), float(projected_y)),
            confidence=float(confidence),
            position_3d=target.position_3d,
            timestamp=0.0,
        )

    def get_results(self) -> List[VisionResult]:
        return list(self._results)

    def get_3d_position(self, result: VisionResult) -> Optional[Tuple[float, float, float]]:
        return result.position_3d

    def project_to_image(self, point_3d: Tuple[float, float, float]) -> Tuple[float, float]:
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        x, y, z = point_3d
        if z <= 0:
            return (0.0, 0.0)

        px = int(fx * x / z + cx)
        py = int(fy * y / z + cy)
        return (px, py)

    def triangulate(self, points_2d: List[Tuple[float, float]],
                    camera_poses: List[np.ndarray] = None) -> Optional[Tuple[float, float, float]]:
        if len(points_2d) < 2:
            return None

        if camera_poses is None:
            camera_poses = [np.eye(4), np.eye(4)]

        p1 = self.camera_matrix @ camera_poses[0][:3, :4]
        p2 = self.camera_matrix @ camera_poses[1][:3, :4]

        pts2d = np.array(points_2d, dtype=float)

        try:
            from cv2 import triangulatePoints
            result = triangulatePoints(p1, p2, pts2d[:, 0:1], pts2d[:, 1:2])
            result = result / result[3]
            return (float(result[0]), float(result[1]), float(result[2]))
        except ImportError:
            return None
        except Exception:
            return None


class PoseEstimator:
    def __init__(self, robot_config, vision_pipeline: Optional[VisionPipeline] = None):
        self.robot_config = robot_config
        self.vision = vision_pipeline
        self._last_estimates: Dict[int, Tuple[float, float, float]] = {}

    def estimate_pose(self, vision_results: List[VisionResult]) -> Dict[str, Any]:
        poses = {}
        for result in vision_results:
            if result.position_3d:
                poses[result.target_id] = result.position_3d
                self._last_estimates[result.target_id] = result.position_3d

        if not poses:
            return {"status": "no_targets", "poses": {}}

        return {
            "status": "success",
            "poses": poses,
            "count": len(poses),
            "robot": self.robot_config.name,
        }

    def get_last_estimate(self, target_id: int) -> Optional[Tuple[float, float, float]]:
        return self._last_estimates.get(target_id)
