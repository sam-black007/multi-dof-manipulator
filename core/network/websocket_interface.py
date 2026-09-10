import json
import threading
import asyncio
from typing import Dict, Callable, Optional, Any
from http.server import HTTPServer, BaseHTTPRequestHandler
import socket

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False


class WebSocketInterface:
    def __init__(self, host="0.0.0.0", port=8765):
        self.host = host
        self.port = port
        self._clients: Dict[str, Any] = {}
        self._handlers: Dict[str, Callable] = {}
        self._running = False
        self._server = None
        self._loop = None
        self._thread = None

    def register_handler(self, command: str, handler: Callable):
        self._handlers[command] = handler

    def _handle_message(self, message: str, client_id: str) -> str:
        try:
            data = json.loads(message)
            command = data.get("command")
            params = data.get("params", {})

            if command in self._handlers:
                result = self._handlers[command](**params)
                return json.dumps({"status": "success", "result": result})
            else:
                return json.dumps({"status": "error", "message": f"Unknown command: {command}"})
        except json.JSONDecodeError:
            return json.dumps({"status": "error", "message": "Invalid JSON"})
        except Exception as e:
            return json.dumps({"status": "error", "message": str(e)})

    async def _handle_client(self, websocket, path):
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self._clients[client_id] = websocket

        try:
            async for message in websocket:
                response = self._handle_message(message, client_id)
                await websocket.send(response)
        except Exception:
            pass
        finally:
            self._clients.pop(client_id, None)

    def start(self):
        if not HAS_WEBSOCKETS:
            raise ImportError("websockets package required: pip install websockets")

        self._running = True

        async def serve():
            self._server = await websockets.serve(
                self._handle_client, self.host, self.port
            )

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=lambda: self._loop.run_until_complete(serve()), daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._server:
            self._loop.call_soon_threadsafe(self._server.close)
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=5)

    def is_running(self):
        return self._running

    def send_to_client(self, client_id: str, message: str):
        if client_id in self._clients:
            asyncio.run_coroutine_threadsafe(
                self._clients[client_id].send(message), self._loop
            )

    def broadcast(self, message: str):
        for client_id, ws in list(self._clients.items()):
            asyncio.run_coroutine_threadsafe(ws.send(message), self._loop)


class RobotServer:
    def __init__(self, robot_config, host="0.0.0.0", port=8765):
        self.robot_config = robot_config
        self.ws = WebSocketInterface(host, port)
        self._setup_handlers()

    def _setup_handlers(self):
        self.ws.register_handler("get_state", self._get_state)
        self.ws.register_handler("get_joint_angles", self._get_joint_angles)
        self.ws.register_handler("set_joint_angles", self._set_joint_angles)
        self.ws.register_handler("solve_ik", self._solve_ik)
        self.ws.register_handler("get_ee_position", self._get_ee_position)
        self.ws.register_handler("get_kinematics_info", self._get_kinematics_info)

    def _get_state(self):
        return {"status": "active", "robot": self.robot_config.name, "dof": self.robot_config.dof}

    def _get_joint_angles(self):
        return self.robot_config.home_position

    def _set_joint_angles(self, angles):
        return {"status": "moving", "angles": angles}

    def _solve_ik(self, target_x, target_y, target_z, **kwargs):
        from core.kinematics import InverseKinematics
        ik = InverseKinematics(self.robot_config)
        target = {"x": target_x, "y": target_y, "z": target_z}
        target.update(kwargs)
        try:
            result = ik.solve(target)
            return {"status": "success", "joint_angles": result["joint_angles"]}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _get_ee_position(self):
        from core.kinematics import ForwardKinematics
        fk = ForwardKinematics(self.robot_config)
        pose = fk.compute(self.robot_config.home_position)
        return {"x": pose["x"], "y": pose["y"], "z": pose["z"]}

    def _get_kinematics_info(self):
        return {
            "name": self.robot_config.name,
            "dof": self.robot_config.dof,
            "home_position": self.robot_config.home_position,
            "joint_limits": [(j.min_angle, j.max_angle) for j in self.robot_config.joints],
        }

    def start(self):
        self.ws.start()

    def stop(self):
        self.ws.stop()
