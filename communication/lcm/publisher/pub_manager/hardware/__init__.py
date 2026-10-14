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

    elif name == "franka_hand":
        from communication.lcm.publisher.pub_manager.hardware.franka_panda.FrankaHandHwPubManager import (
            FrankaHandHwPubManager,
        )

        return FrankaHandHwPubManager(**cfg_pub_manager)

    raise ValueError(f"Publisher manager {name} not found.")
