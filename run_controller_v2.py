"""run_controller.py for the v2 collision-aware controller.

The controller / pub-manager factories are left untouched, so this runner builds
the v2 classes directly (any other controller name falls back to the factories).

    python run_controller_v2.py --config configs/franka_fr3/controllers/col_aware_grav_comp_v2.yaml

Reproduce the original behaviour (each cause visible in run_visualizer_v2.py):

    python run_controller_v2.py --config configs/franka_fr3/controllers/col_aware_grav_comp_v2.yaml \
        --v2.fix_fresh_state false --v2.fix_multi_point_witness false --v2.fix_smooth_command false

(overrides are space separated ``--key value``, one fix at a time also works)

Tutorials 01-03 v2 (gravity comp, stale-LCM-data fix only), e.g. on the robot:

    python run_controller_v2.py --config configs/franka_fr3/controllers/grav_comp_hw.yaml

Tutorial 06 v2 (IK QP, stale-LCM-data fix only) reuses the original yaml:

    python run_controller_v2.py --config configs/franka_fr3/controllers/ik_qp.yaml --name ik_qp_v2
"""

import argparse

from omegaconf import DictConfig, OmegaConf

from communication.lcm.publisher.pub_manager.controller import (
    get_controller_pub_manager,
)
from communication.lcm.subscriber.sub_manager.controller import (
    get_controller_sub_manager,
)
from controller import get_controller
from utils.terminal.argparse_utils import parse_nested_args, parse_unknown_args


def build_controller(cfg: DictConfig):
    if cfg["name"] == "col_aware_grav_comp_v2":
        from controller.CollisionAwareGravCompControl_v2 import (
            CollisionAwareGravCompControllerV2,
        )

        return CollisionAwareGravCompControllerV2(cfg)
    if cfg["name"] == "grav_comp_v2":
        from controller.GravCompControl_v2 import GravCompControllerV2

        return GravCompControllerV2(cfg)
    if cfg["name"] == "ik_qp_v2":
        from controller.IKQPControl_v2 import IKQPControllerV2

        return IKQPControllerV2(cfg)
    return get_controller(cfg)


def build_pub_manager(cfg_pub: DictConfig):
    if cfg_pub["name"] == "joint_ctrl_col_debug_v2":
        from communication.lcm.publisher.pub_manager.controller.JointCtrlColDebugPubManager_v2 import (
            JointCtrlColDebugPubManagerV2,
        )

        return JointCtrlColDebugPubManagerV2(**cfg_pub)
    return get_controller_pub_manager(cfg_pub)


def run_controller(cfg_controller: DictConfig):
    controller = build_controller(cfg_controller)

    sub = get_controller_sub_manager(cfg_controller["sub_manager"])
    controller.set_sub_que_dict(sub.get_sub_que_dict())

    pub = build_pub_manager(cfg_controller["pub_manager"])
    controller.set_ctrl_pub_que(pub.get_ctrl_pub_que())
    controller.set_pub_que_dict(pub.get_pub_que_dict())

    sub.start()
    pub.start()

    # Stop the LCM threads also when initialize() fails or is interrupted
    # (e.g. Ctrl-C while waiting for collision info); otherwise the process
    # hangs on exit.
    try:
        controller.initialize()
        controller.start()
    finally:
        sub.stop()
        pub.stop()
        controller.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, required=True)
    args, unknown = parser.parse_known_args()

    cfg = OmegaConf.load(args.config)
    cfg = OmegaConf.merge(cfg, parse_nested_args(parse_unknown_args(unknown)))
    print(OmegaConf.to_yaml(cfg))

    run_controller(cfg)
