from enum import Enum, auto
from typing import Dict, List, Optional, Callable
import time


class SafetyState(Enum):
    DISABLED = auto()
    IDLE = auto()
    ARMED = auto()
    MOVING = auto()
    EMERGENCY_STOP = auto()
    FAULT = auto()


class SafetyEvent(Enum):
    SYSTEM_START = auto()
    ARM_COMMAND = auto()
    MOVE_COMMAND = auto()
    STOP_COMMAND = auto()
    EMERGENCY_PRESSED = auto()
    FAULT_DETECTED = auto()
    FAULT_CLEARED = auto()
    RESET_COMMAND = auto()
    LIMIT_EXCEEDED = auto()
    COLLISION_DETECTED = auto()


class SafetyLimits:
    def __init__(self, max_joint_velocity=180.0, max_joint_acceleration=360.0,
                 max_cartesian_velocity=0.5, max_cartesian_acceleration=1.0,
                 max_force=50.0, temperature_threshold=80.0):
        self.max_joint_velocity = max_joint_velocity
        self.max_joint_acceleration = max_joint_acceleration
        self.max_cartesian_velocity = max_cartesian_velocity
        self.max_cartesian_acceleration = max_cartesian_acceleration
        self.max_force = max_force
        self.temperature_threshold = temperature_threshold

    def check_velocity(self, velocities, current_velocities):
        violations = []
        for i, (v, cv) in enumerate(zip(velocities, current_velocities)):
            if abs(v - cv) > self.max_joint_acceleration:
                violations.append(f"Joint {i}: acceleration violation")
        return violations

    def check_force(self, forces):
        violations = []
        for i, f in enumerate(forces):
            if abs(f) > self.max_force:
                violations.append(f"Joint {i}: force {f} exceeds {self.max_force}")
        return violations

    def check_temperature(self, temperatures):
        violations = []
        for i, t in enumerate(temperatures):
            if t > self.temperature_threshold:
                violations.append(f"Joint {i}: temperature {t} exceeds {self.temperature_threshold}")
        return violations


class SafetyStateMachine:
    def __init__(self, limits: Optional[SafetyLimits] = None):
        self.state = SafetyState.DISABLED
        self.limits = limits or SafetyLimits()
        self._handlers: Dict[SafetyEvent, List[Callable]] = {}
        self._state_log: List[dict] = []
        self._fault_reason = ""

    def register_handler(self, event: SafetyEvent, handler: Callable):
        if event not in self._handlers:
            self._handlers[event] = []
        self._handlers[event].append(handler)

    def _log_transition(self, from_state, to_state, event):
        self._state_log.append({
            "timestamp": time.time(),
            "from": from_state.name,
            "to": to_state.name,
            "event": event.name,
        })

    def _transition(self, event: SafetyEvent):
        from_state = self.state
        handler_map = {
            (SafetyState.DISABLED, SafetyEvent.SYSTEM_START): SafetyState.IDLE,
            (SafetyState.IDLE, SafetyEvent.ARM_COMMAND): SafetyState.ARMED,
            (SafetyState.ARMED, SafetyEvent.MOVE_COMMAND): SafetyState.MOVING,
            (SafetyState.MOVING, SafetyEvent.STOP_COMMAND): SafetyState.IDLE,
            (SafetyState.MOVING, SafetyEvent.EMERGENCY_PRESSED): SafetyState.EMERGENCY_STOP,
            (SafetyState.ARMED, SafetyEvent.EMERGENCY_PRESSED): SafetyState.EMERGENCY_STOP,
            (SafetyState.IDLE, SafetyEvent.EMERGENCY_PRESSED): SafetyState.EMERGENCY_STOP,
            (SafetyState.EMERGENCY_STOP, SafetyEvent.RESET_COMMAND): SafetyState.IDLE,
            (SafetyState.EMERGENCY_STOP, SafetyEvent.FAULT_CLEARED): SafetyState.IDLE,
            (SafetyState.MOVING, SafetyEvent.FAULT_DETECTED): SafetyState.FAULT,
            (SafetyState.ARMED, SafetyEvent.FAULT_DETECTED): SafetyState.FAULT,
            (SafetyState.FAULT, SafetyEvent.FAULT_CLEARED): SafetyState.IDLE,
            (SafetyState.MOVING, SafetyEvent.LIMIT_EXCEEDED): SafetyState.FAULT,
            (SafetyState.MOVING, SafetyEvent.COLLISION_DETECTED): SafetyState.EMERGENCY_STOP,
        }

        new_state = handler_map.get((from_state, event))
        if new_state is None:
            raise ValueError(f"Invalid transition: {from_state.name} + {event.name}")

        self._transition(from_state, new_state, event)
        self.state = new_state

        if event in self._handlers:
            for handler in self._handlers[event]:
                handler(self.state, event)

    def arm(self):
        self._transition(SafetyEvent.ARM_COMMAND)

    def move(self):
        self._transition(SafetyEvent.MOVE_COMMAND)

    def stop(self):
        self._transition(SafetyEvent.STOP_COMMAND)

    def emergency_stop(self):
        self._transition(SafetyEvent.EMERGENCY_PRESSED)

    def reset(self):
        self._transition(SafetyEvent.RESET_COMMAND)

    def clear_fault(self):
        self._transition(SafetyEvent.FAULT_CLEARED)

    def detect_fault(self, reason=""):
        self._fault_reason = reason
        self._transition(SafetyEvent.FAULT_DETECTED)

    def validate_move(self, joint_angles, velocities, forces=None, temperatures=None):
        violations = []
        for v in velocities:
            if abs(v) > self.limits.max_joint_velocity:
                violations.append(f"Velocity {v} exceeds {self.limits.max_joint_velocity}")

        vel_violations = self.limits.check_velocity(velocities, [0]*len(velocities))
        violations.extend(vel_violations)

        if forces:
            violations.extend(self.limits.check_force(forces))
        if temperatures:
            violations.extend(self.limits.check_temperature(temperatures))

        if violations:
            self.detect_fault("; ".join(violations))
            return False, violations

        return True, []

    def get_state(self):
        return self.state

    def get_state_log(self):
        return list(self._state_log)

    def is_safe_to_move(self):
        return self.state in (SafetyState.IDLE, SafetyState.ARMED)

    def __repr__(self):
        return f"SafetyStateMachine(state={self.state.name})"
