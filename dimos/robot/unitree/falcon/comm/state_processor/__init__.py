from .base import BasicStateProcessor
from .unitree import UnitreeStateProcessor


def create_state_processor(config):
    sdk_type = config.get("SDK_TYPE", "unitree")
    if sdk_type == "unitree":
        return UnitreeStateProcessor(config)
    raise ValueError(f"Unsupported SDK type: {sdk_type}")


__all__ = ["BasicStateProcessor", "UnitreeStateProcessor", "create_state_processor"]


