from .base import BasicCommandSender
from .unitree import UnitreeCommandSender


def create_command_sender(config):
    sdk_type = config.get("SDK_TYPE", "unitree")
    if sdk_type == "unitree":
        return UnitreeCommandSender(config)
    raise ValueError(f"Unsupported SDK type: {sdk_type}")


CommandSender = create_command_sender

__all__ = ["BasicCommandSender", "UnitreeCommandSender", "create_command_sender", "CommandSender"]


