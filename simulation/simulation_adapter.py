from core.kinematics import ForwardKinematics, InverseKinematics
from core.configuration import RobotConfiguration


class SimulationAdapter:
    def __init__(self):
        self._robot_config = None
        self._current_angles = None
        self._running = False
        self._fk = None
        self._ik = None

    def initialize(self, robot_config: RobotConfiguration):
        self._robot_config = robot_config
        self._current_angles = list(robot_config.home_position)
        self._fk = ForwardKinematics(robot_config)
        self._ik = InverseKinematics(robot_config)
        self._running = True

    def set_joint_angles(self, joint_angles_deg):
        if not self._running:
            raise RuntimeError("Simulation not initialized")
        self._current_angles = list(joint_angles_deg)

    def get_joint_angles(self):
        return list(self._current_angles)

    def get_ee_position(self):
        if not self._running:
            raise RuntimeError("Simulation not initialized")
        pose = self._fk.compute(self._current_angles)
        return (pose["x"], pose["y"], pose["z"])

    def step(self, dt):
        if not self._running:
            raise RuntimeError("Simulation not initialized")
        return {"dt": dt, "status": "stepped"}

    def reset(self):
        if self._robot_config:
            self._current_angles = list(self._robot_config.home_position)

    def is_running(self):
        return self._running

    def solve_ik(self, target_pose):
        if not self._running:
            raise RuntimeError("Simulation not initialized")
        return self._ik.solve(target_pose)

    def simulate_trajectory(self, trajectory, step_size=0.1):
        if not self._running:
            raise RuntimeError("Simulation not initialized")
        for point in trajectory:
            self.set_joint_angles(point)
            self.step(step_size)
        return True


class OmniSimAdapter(SimulationAdapter):
    pass


__all__ = ["SimulationAdapter", "OmniSimAdapter"]
