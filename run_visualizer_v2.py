"""run_visualizer.py for the v2 collision-aware debug overlay.

The visualizer factory is left untouched, so this runner builds
FrankaPandaVisManagerV2 directly.

    python run_visualizer_v2.py --config configs/visualizer/franka_panda/collision_v2.yaml

Works with run_controller.py (original) and run_controller_v2.py.
"""

import argparse

from omegaconf import DictConfig, OmegaConf

from utils.terminal.argparse_utils import parse_nested_args, parse_unknown_args
from visualizer.vis_manager.franka_panda.FrankaPandaVisManager_v2 import (
    FrankaPandaVisManagerV2,
)


def run_visualizer(cfg_visualizer: DictConfig):
    vis_manager = FrankaPandaVisManagerV2(cfg_visualizer)
    vis_manager.initialize()
    vis_manager.start()
    vis_manager.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True)
    args, unknown = parser.parse_known_args()

    cfg = OmegaConf.load(args.config)
    cfg = OmegaConf.merge(cfg, parse_nested_args(parse_unknown_args(unknown)))
    print(OmegaConf.to_yaml(cfg))

    run_visualizer(cfg)
