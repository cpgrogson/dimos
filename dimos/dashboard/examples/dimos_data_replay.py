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

"""Standalone Dask actor example that replays YAML logs into Rerun."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, HTTPServer
import os
from pathlib import Path
import pickle
import sys
import threading
import time
from typing import TYPE_CHECKING, Any

from distributed import Client
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
import yaml

grpc_port = int(os.environ.get("RERUN_GRPC_PORT", "9876"))
rerun_info = {
    "logging_id": os.environ.get("RERUN_ID", "dask_actor_demo"),
    "grpc_port": grpc_port,
    "server_memory_limit": os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "25%"),
    "url": f"rerun+http://127.0.0.1:{grpc_port}/proxy",
}

from dimos.core import Module, Out, pLCMTransport, pSHMTransport
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.dashboard.module import Dashboard, RerunConnection
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage

if TYPE_CHECKING:
    from dimos.msgs.nav_msgs import Odometry


class Dashboard(Module):
    @rpc
    def start(self):
        super().start()

        rr.init(rerun_info["logging_id"], spawn=False, recording_id=rerun_info["logging_id"])
        default_blueprint = rrb.Blueprint(
            rrb.Tabs(
                rrb.Spatial3DView(
                    name="Spatial3D",
                    origin="/",
                    line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
                ),
                rrb.TextDocumentView(name="Logs", origin="/logs"),
            )
        )
        rr.send_blueprint(default_blueprint)
        rr.serve_grpc(
            grpc_port=rerun_info["grpc_port"],
            default_blueprint=default_blueprint,
            server_memory_limit=rerun_info["server_memory_limit"],
        )

        #
        # Manual control of Rerun viewer (simple html server)
        #
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(
                    f"""
                    <body>
                        <style>body {{ margin: 0; border: 0; }}\ncanvas {{ width: 100vw !important; height: 100vh !important; }}</style>
                        <script type="module">
                            import {{ WebViewer }} from "https://esm.sh/@rerun-io/web-viewer@0.27.2";
                            const viewer = new WebViewer();
                            viewer.start("{rerun_info["url"]}", document.body);
                        </script>
                    </body>
                """.encode()
                )

            def log_message(self, *args):
                return

        host = "127.0.0.1"
        port = 4000
        server = HTTPServer((host, port), Handler)
        threading.Thread(target=server.serve_forever, name="dashboard-server", daemon=True).start()


DEFAULT_REPLAY_PATHS = {
    "color_image": str(Path(__file__).with_name(f"example_data_{'color_image'}.flat.yaml")),
    "lidar": str(Path(__file__).with_name(f"example_data_{'lidar'}.flat.yaml")),
}


def iter_yaml_data_line_by_line(path):
    if not Path(path).exists():
        raise FileNotFoundError(Path(path))
    with Path(path).open("r", encoding="utf-8") as f:
        for i, line in enumerate(f):
            if not line.startswith("- "):
                continue
            try:
                yield pickle.loads(yaml.safe_load(line[2:]))
            except Exception as exc:
                print(f"[yaml_replay_read] line {i} parse error: {exc}", file=sys.stderr)
                continue


class DataReplay(Module):
    color_image: Out[Image] = None  # type: ignore[assignment]
    lidar: Out[LidarMessage] = None  # type: ignore[assignment]
    odom: Out[Odometry] = None  # type: ignore[assignment]

    @rpc
    def start(self) -> bool:
        super().start()
        for output_name, yaml_filepath in DEFAULT_REPLAY_PATHS.items():
            threading.Thread(
                target=self._publish_stream,
                args=(yaml_filepath,),
                name=f"{output_name}-replay",
                daemon=True,
            ).start()
        return True

    def _publish_stream(self, yaml_filepath):
        print("starting replay of data from " + yaml_filepath)
        stream = rr.RecordingStream(rerun_info["logging_id"], recording_id=rerun_info["logging_id"])
        stream.connect_grpc(rerun_info["url"])
        while True:  # restart if ran out of messages
            for log_path, payload in iter_yaml_data_line_by_line(yaml_filepath):
                print("logging " + yaml_filepath)
                stream.log(log_path, payload, strict=True)


# ------------------------------ Entrypoint --------------------------------- #
blueprint = (
    autoconnect(
        DataReplay.blueprint(),
        Dashboard.blueprint(),
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport("/replay/color_image"),
            ("lidar", LidarMessage): pLCMTransport("/replay/lidar"),
        }
    )
    .global_config(n_dask_workers=2)
)


def main() -> None:
    coordinator = blueprint.build()
    print("Data replay running. Press Ctrl+C to stop.")
    coordinator.loop()


if __name__ == "__main__":
    main()
