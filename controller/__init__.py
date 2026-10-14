"""Controller factory."""

from omegaconf import DictConfig

from controller.BaseController import BaseController


def get_controller(cfg_controller: DictConfig) -> BaseController:
    """Create a controller from ``cfg_controller['name']``."""
    name = cfg_controller["name"]

    if name == "base":
        return BaseController(cfg_controller)

    if name == "grav_comp":
        from controller.GravCompControl import GravCompController

        return GravCompController(cfg_controller)

    if name == "col_aware_grav_comp":
        from controller.CollisionAwareGravCompControl import (
            CollisionAwareGravCompController,
        )

        return CollisionAwareGravCompController(cfg_controller)

    if name == "ik_qp":
        from controller.IKQPControl import IKQPController

        return IKQPController(cfg_controller)

    raise ValueError(f"Unknown controller name: {name}.")
