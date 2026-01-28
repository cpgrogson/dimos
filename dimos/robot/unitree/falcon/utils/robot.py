import numpy as np


class Robot:
    """Vendored from FALCON/sim2real/utils/robot.py.

    Container for robot configuration constants used by Falcon deployment code.
    """

    def __init__(self, config):
        self.ROBOT_TYPE = config["ROBOT_TYPE"]
        self.MOTOR2JOINT = config["MOTOR2JOINT"]
        self.JOINT2MOTOR = config["JOINT2MOTOR"]
        self.UNITREE_LEGGED_CONST = config.get("UNITREE_LEGGED_CONST", None)
        self.MOTOR_KP = np.array(config["MOTOR_KP"])
        self.MOTOR_KD = np.array(config["MOTOR_KD"])
        self.WeakMotorJointIndex = config.get("WeakMotorJointIndex", None)
        self.NUM_MOTORS = config["NUM_MOTORS"]
        self.NUM_JOINTS = config["NUM_JOINTS"]
        self.DEFAULT_DOF_ANGLES = config["DEFAULT_DOF_ANGLES"]
        self.DEFAULT_MOTOR_ANGLES = config["DEFAULT_MOTOR_ANGLES"]
        self.USE_SENSOR = config["USE_SENSOR"]
        self.MOTOR_EFFORT_LIMIT_LIST = config["motor_effort_limit_list"]
        self.MOTOR_VEL_LIMIT_LIST = config["motor_vel_limit_list"]
        self.MOTOR_POS_LOWER_LIMIT_LIST = config["motor_pos_lower_limit_list"]
        self.MOTOR_POS_UPPER_LIMIT_LIST = config["motor_pos_upper_limit_list"]


