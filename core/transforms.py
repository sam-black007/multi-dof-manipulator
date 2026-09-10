import numpy as np


def rotation_x(angle_rad):
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1],
    ])


def rotation_y(angle_rad):
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1],
    ])


def rotation_z(angle_rad):
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])


def translation(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1],
    ])


def homogeneous_transform(dh_params):
    a, alpha, d, theta = dh_params
    return rotation_z(theta) @ translation(a, 0, d) @ rotation_x(alpha)


def compute_forward_kinematics(dh_table, joint_angles):
    T = np.eye(4)
    for i, (a, alpha, d, theta) in enumerate(dh_table):
        T_i = homogeneous_transform((a, alpha, d, theta + joint_angles[i]))
        T = T @ T_i
    return T


def transform_to_pose(T):
    position = T[:3, 3]
    r11, r12, r13 = T[0, 0], T[0, 1], T[0, 2]
    r21, r22, r23 = T[1, 0], T[1, 1], T[1, 2]
    r31, r32, r33 = T[2, 0], T[2, 1], T[2, 2]

    pitch = np.arcsin(-r31)
    if np.cos(pitch) > 1e-6:
        roll = np.arctan2(r32, r33)
        yaw = np.arctan2(r21, r11)
    else:
        roll = np.arctan2(-r23, r12)
        yaw = 0.0

    return {
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
    }


def pose_to_transform(pose):
    x, y, z = pose["x"], pose["y"], pose["z"]
    roll = pose.get("roll", 0.0)
    pitch = pose.get("pitch", 0.0)
    yaw = pose.get("yaw", 0.0)

    R = rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)
    T = np.eye(4)
    T[:3, :3] = R[:3, :3]
    T[:3, 3] = [x, y, z]
    return T


def pose_difference(pose_a, pose_b):
    T_a = pose_to_transform(pose_a)
    T_b = pose_to_transform(pose_b)
    T_diff = np.linalg.inv(T_b) @ T_a
    pos_diff = T_diff[:3, 3]
    R_diff = T_diff[:3, :3]
    trace = np.trace(R_diff)
    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
    return pos_diff, angle


def are_poses_equal(pose_a, pose_b, pos_tol=0.01, orient_tol=0.0175):
    pos_diff, orient_diff = pose_difference(pose_a, pose_b)
    return np.linalg.norm(pos_diff) < pos_tol and abs(orient_diff) < orient_tol