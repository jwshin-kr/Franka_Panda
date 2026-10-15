from omegaconf import DictConfig

from communication.lcm.publisher.BasePubManager import BasePubManager


def get_hardware_pub_manager(cfg_pub_manager: DictConfig) -> BasePubManager:
    """Get the publisher manager based on the configuration.
    @param[in] cfg_pub_manager: Configuration for the publisher manager.
    @return: An instance of the publisher manager.
    """
    name = cfg_pub_manager["name"]

    if name == "base":
        return BasePubManager(**cfg_pub_manager)

    elif name == "franka_fr3_hw":
        from communication.lcm.publisher.pub_manager.hardware.franka_fr3.FrankaFR3HwPubManager import (
            FrankaFR3HwPubManager,
        )

        return FrankaFR3HwPubManager(**cfg_pub_manager)

    raise ValueError(f"Publisher manager {name} not found.")
