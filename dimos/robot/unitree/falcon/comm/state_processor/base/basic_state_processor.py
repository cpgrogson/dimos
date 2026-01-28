from abc import ABC, abstractmethod

import numpy as np

from ...utils.robot import Robot


class BasicStateProcessor(ABC):
    """Vendored from FALCON/sim2real/utils/comm/state_processor/base/basic_state_processor.py.

    Dimos integration uses this to build Falcon's `robot_state_data` vector.
    """

    def __init__(self, config):
        self.config = config
        self.robot = Robot(config)
        self.num_motor = self.robot.NUM_MOTORS
        self.sdk_type = self.config.get("SDK_TYPE", "unitree")
        self.motor_type = self.config.get("MOTOR_TYPE", "serial")

        self.num_dof = self.robot.NUM_JOINTS
        self._init_q = np.zeros(3 + 4 + self.num_dof)
        self.q = self._init_q
        self.dq = np.zeros(3 + 3 + self.num_dof)
        self.ddq = np.zeros(3 + 3 + self.num_dof)
        self.tau_est = np.zeros(3 + 3 + self.num_dof)
        self.robot_state_data = None

        self._init_sdk_components()

    @abstractmethod
    def _init_sdk_components(self):
        pass

    @abstractmethod
    def prepare_low_state(self, msg):
        pass

    @abstractmethod
    def _extract_imu_data(self, imu_state):
        pass

    @abstractmethod
    def _extract_joint_data(self, robot_joint_state):
        pass

    def get_robot_state_data(self):
        return self.robot_state_data

    def _create_robot_state_data(self):
        robot_state_data = np.array(
            self.q.tolist() + self.dq.tolist() + self.tau_est.tolist() + self.ddq.tolist(), dtype=np.float64
        ).reshape(1, -1)
        return robot_state_data


