from omegaconf import DictConfig

from perception.BasePerception import BasePerception


def get_perception(cfg_perception: DictConfig) -> BasePerception:
    """
    Set the perception based on the configuration provided.
    @param[in] cfg_perception: Configuration for the perception.
    """
    name = cfg_perception["name"]
    if name == "base":
        return BasePerception(cfg_perception)
    elif name == "tapnet_click_and_track":
        from perception.tapnet.TapnetClickAndTrack import TapnetClickAndTrack

        return TapnetClickAndTrack(cfg_perception)
    elif name == "tapnet_click_and_track_calib":
        from perception.tapnet.TapnetClickAndTrackCalib import (
            TapnetClickAndTrackCalib,
        )

        return TapnetClickAndTrackCalib(cfg_perception)
    else:
        raise ValueError(f"Unknown controller name: {name}.")
