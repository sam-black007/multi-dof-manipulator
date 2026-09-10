import numpy as np
from typing import Optional, List, Tuple, Dict, Any


def triangulate_points(points_2d: List[Tuple[float, float]],
                       projections_1: np.ndarray,
                       projections_2: np.ndarray) -> Optional[Tuple[float, float, float]]:
    if len(points_2d) < 2:
        return None

    try:
        from cv2 import triangulatePoints
        pts2d_1 = np.array([[p[0]] for p in points_2d], dtype=float)
        pts2d_2 = np.array([[p[1]] for p in points_2d], dtype=float)
        result = triangulatePoints(projections_1, projections_2, pts2d_1, pts2d_2)
        result = result / result[3]
        return (float(result[0]), float(result[1]), float(result[2]))
    except ImportError:
        return None
    except Exception:
        return None


def estimate_relative_pose(pose_a: Tuple[float, float, float],
                           pose_b: Tuple[float, float, float]) -> Dict[str, Any]:
    delta = (pose_b[0] - pose_a[0], pose_b[1] - pose_a[1], pose_b[2] - pose_a[2])
    distance = np.sqrt(delta[0]**2 + delta[1]**2 + delta[2]**2)
    return {
        "delta": delta,
        "distance": distance,
        "pose_a": pose_a,
        "pose_b": pose_b,
    }
