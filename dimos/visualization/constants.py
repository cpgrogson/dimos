from typing import Literal, TypeAlias

ViewerBackend: TypeAlias = Literal["rerun", "foxglove", "none"]
RerunOpenOption: TypeAlias = Literal["none", "web", "native", "both"]

RERUN_OPEN_DEFAULT: RerunOpenOption = "native"
RERUN_ENABLE_WEB = True
RERUN_GRPC_PORT = 9876
RERUN_WEB_PORT = 9090