#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import logging
import sys
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from dimos.vr.models import ControllerFrame
from dimos.vr.generate_cert import generate_self_signed_cert, get_local_ip

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def log_controller_data(frame: ControllerFrame):
    """Log complete controller data in readable format."""
    print(f"\n{'=' * 60}")
    print(f"Frame #{frame.timestamp:.3f}")
    print(f"{'=' * 60}")

    for side, controller in [("LEFT", frame.left), ("RIGHT", frame.right)]:
        if controller:
            print(f"\n{side} CONTROLLER:")
            print(f"  Connected: {controller.connected}")
            print(
                f"  Position (m): x={controller.position[0]:.3f}, y={controller.position[1]:.3f}, z={controller.position[2]:.3f}"
            )
            print(
                f"  Rotation (quat): x={controller.rotation[0]:.3f}, y={controller.rotation[1]:.3f}, z={controller.rotation[2]:.3f}, w={controller.rotation[3]:.3f}"
            )
            print(
                f"  Rotation (deg): x={controller.rotation_euler[0]:.1f}, y={controller.rotation_euler[1]:.1f}, z={controller.rotation_euler[2]:.1f}"
            )

            print(f"  Buttons:")
            print(
                f"    Trigger: {controller.buttons.trigger.value:.2f} {'PRESSED' if controller.buttons.trigger.pressed else 'released'}"
            )
            print(
                f"    Grip: {controller.buttons.grip.value:.2f} {'PRESSED' if controller.buttons.grip.pressed else 'released'}"
            )
            print(
                f"    Menu: {controller.buttons.menu.value:.2f} {'PRESSED' if controller.buttons.menu.pressed else 'released'}"
            )
            print(
                f"    Thumbstick: {controller.buttons.thumbstick.value:.2f} {'PRESSED' if controller.buttons.thumbstick.pressed else 'released'}"
            )
            print(
                f"    X/A: {controller.buttons.x_or_a.value:.2f} {'PRESSED' if controller.buttons.x_or_a.pressed else 'released'}"
            )
            print(
                f"    Y/B: {controller.buttons.y_or_b.value:.2f} {'PRESSED' if controller.buttons.y_or_b.pressed else 'released'}"
            )

            print(
                f"  Thumbstick: x={controller.axes.thumbstick_x:.3f}, y={controller.axes.thumbstick_y:.3f}"
            )
            print(f"  Raw button values: {[f'{v:.2f}' for v in controller.button_values]}")
        else:
            print(f"\n{side} CONTROLLER: Not connected")


class VRServer:
    def __init__(self, host="0.0.0.0", port=8881):
        self.host = host
        self.port = port
        self.app = FastAPI(title="VR Test Server")
        self.stats = {"frames_received": 0, "errors": 0, "last_frame": None}
        self._setup_routes()

    def _setup_routes(self):
        static_dir = Path(__file__).parent / "static"
        self.app.mount("/static", StaticFiles(directory=static_dir), name="static")

        @self.app.get("/")
        async def get_root():
            index_path = static_dir / "index.html"
            return HTMLResponse(content=index_path.read_text())

        @self.app.get("/health")
        async def health():
            return self.stats

        @self.app.get("/camera/list")
        async def list_cameras():
            return {"cameras": [], "count": 0}

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            client_ip = websocket.client.host if websocket.client else "unknown"
            logger.info(f"Quest connected: {client_ip}")

            try:
                while True:
                    try:
                        message = await websocket.receive()

                        if message["type"] == "websocket.disconnect":
                            break

                    except WebSocketDisconnect:
                        break

                    data_text = message.get("text")
                    if not data_text:
                        continue

                    try:
                        frame_dict = json.loads(data_text)
                        frame = ControllerFrame(**frame_dict)

                        self.stats["frames_received"] += 1
                        self.stats["last_frame"] = frame

                        log_controller_data(frame)

                    except ValidationError as e:
                        self.stats["errors"] += 1
                        logger.error(f"Validation error: {e}")
                    except Exception as e:
                        self.stats["errors"] += 1
                        logger.error(f"Error: {e}")

            except WebSocketDisconnect:
                logger.info(f"Quest disconnected: {client_ip}")
            except Exception as e:
                logger.error(f"WebSocket error: {e}")

    def run(self):
        cert_file = Path(__file__).parent / "certificates" / "cert.pem"
        key_file = Path(__file__).parent / "certificates" / "key.pem"

        if not cert_file.exists() or not key_file.exists():
            logger.info("Generating SSL certificate...")
            generate_self_signed_cert()

        local_ip = get_local_ip()
        logger.info(f"VR Server starting on https://{local_ip}:{self.port}")
        logger.info("Connect Quest 3 to this URL")

        uvicorn.run(
            self.app,
            host=self.host,
            port=self.port,
            ssl_certfile=str(cert_file),
            ssl_keyfile=str(key_file),
            log_level="info",
        )


if __name__ == "__main__":
    server = VRServer()
    server.run()
