from omegaconf import DictConfig
from omegaconf import OmegaConf

from robot_platform.hardware import BaseHardwarePlatform


def get_hardware_platform(
    cfg_platform: DictConfig,
) -> BaseHardwarePlatform:
    """
    Set the platform based on the configuration.
    @param[in] cfg_platform: Configuration for the platform.
    """
    name = cfg_platform["name"]
    # merged to get channel names from sub and pub manager configs
    # cfg_merged = OmegaConf.merge(cfg_platform, cfg_sub_manager, cfg_pub_manager)
    if name == "base":
        return BaseHardwarePlatform(cfg_platform)
    elif name == "franka_fr3":
        from robot_platform.hardware.franka_fr3.HardwareFrankaFR3 import (
            HardwareFrankaFR3,
        )

        return HardwareFrankaFR3(cfg_platform)
    else:
        raise ValueError(f"Unknown platform name: {name}")
