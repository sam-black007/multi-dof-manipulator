class RobotError(Exception):
    pass


class KinematicsError(RobotError):
    pass


class ValidationError(RobotError):
    pass


class ConfigurationError(RobotError):
    pass


class HardwareError(RobotError):
    pass


class TrajectoryError(RobotError):
    pass