from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray


@dataclass
class LowCmdBuilderConfig:
    robot_type: str = "g1"
    mode_pr: int = 0  # Unitree HG: 0=PR, 1=AB


class LowCmdBuilder:
    """Build SDK2 LowCmd messages (HG vs GO) from per-joint targets in motor order."""

    def __init__(self, config: LowCmdBuilderConfig) -> None:
        self.config = config

        # Lazy import SDK2 types so non-SDK2 users don't need unitree_sdk2py installed.
        if config.robot_type in ("g1", "h1_2"):
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmd_default

            self._create_lowcmd = LowCmd_default
            self._is_hg = True
        else:
            from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_ as LowCmd_default

            self._create_lowcmd = LowCmd_default
            self._is_hg = False

        # CRC is required by real robot; harmless in sim.
        try:
            from unitree_sdk2py.utils.crc import CRC

            self._crc = CRC()
        except Exception:
            self._crc = None

    def build(
        self,
        *,
        mode_machine: int,
        enabled: bool,
        q: NDArray[np.floating],
        dq: NDArray[np.floating] | None,
        kp: NDArray[np.floating] | None,
        kd: NDArray[np.floating] | None,
        tau: NDArray[np.floating] | None,
    ) -> Any:
        cmd = self._create_lowcmd()

        # HG LowCmd: must set mode_machine + mode_pr consistently; GO ignores these attrs.
        if hasattr(cmd, "mode_machine"):
            cmd.mode_machine = int(mode_machine)
        if hasattr(cmd, "mode_pr"):
            cmd.mode_pr = int(self.config.mode_pr)

        n = len(q)
        dq_arr = dq if dq is not None else np.zeros(n, dtype=np.float32)
        kp_arr = kp if kp is not None else np.zeros(n, dtype=np.float32)
        kd_arr = kd if kd is not None else np.zeros(n, dtype=np.float32)
        tau_arr = tau if tau is not None else np.zeros(n, dtype=np.float32)

        for i in range(n):
            # HG: motor_cmd[i].mode is enable(1)/disable(0). GO: mode is motor control mode.
            cmd.motor_cmd[i].mode = 1 if enabled else 0
            cmd.motor_cmd[i].q = float(q[i])
            cmd.motor_cmd[i].dq = float(dq_arr[i])
            cmd.motor_cmd[i].kp = float(kp_arr[i])
            cmd.motor_cmd[i].kd = float(kd_arr[i])
            cmd.motor_cmd[i].tau = float(tau_arr[i])

        if self._crc is not None and hasattr(cmd, "crc"):
            try:
                cmd.crc = self._crc.Crc(cmd)
            except Exception:
                # Best-effort; some sim paths may not require/allow CRC.
                pass

        return cmd


