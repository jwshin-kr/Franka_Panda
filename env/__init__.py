"""Environment factory."""

from omegaconf import DictConfig

from env.BaseEnv import BaseEnv


def get_env(cfg_env: DictConfig) -> BaseEnv:
    """Create an environment from ``cfg_env['name']``."""
    name = cfg_env["name"]

    if name == "default_mujoco":
        from env.MujocoBaseEnv import MujocoBaseEnv

        return MujocoBaseEnv(cfg_env)

    if name == "franka_fr3_col":
        from env.franka_fr3.FrankaArmHandColEnv import FrankaArmHandColEnv

        return FrankaArmHandColEnv(cfg_env)

    raise ValueError(f"Unknown environment name: {name}.")
