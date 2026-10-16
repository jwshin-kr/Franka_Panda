from sensor.BaseSensorManager import BaseSensorManager
from sensor.sensor.realsense.MockRealSense import MockRealSense
from sensor.sensor_info.realsense.CameraInfo import CameraInfo


class MockRealSenseManager(BaseSensorManager):
    """
    RealSenseManager with every camera replaced by a MockRealSense (MuJoCo
    render). Selected by ``mock_mode: true`` in the RealSense config; reads the
    same ``realsense`` entries (channel, size, K, depth_factor, tf_link_to_cam)
    and publishes the same RGBD + camera_info, so everything downstream runs
    unchanged. Needs no pyrealsense2 and no camera.
    """

    def __init__(self, config, *args, **kwargs):
        super().__init__(config, *args, **kwargs)
        self._config = config
        self._realsense_configs = self._config["realsense"]
        self._pub_manager_config = self._config["pub_manager"]
        self._mock_config = self._config.get("mock", {}) or {}

    def initialize_sensors_and_sensor_info(self):
        for key, cfg in self._realsense_configs.items():
            name = self._pub_manager_config["realsense"][f"{key}_channel"]
            width = cfg.get("width", 640)
            height = cfg.get("height", 480)
            frequency = cfg.get("frequency", 30)
            fx = cfg.get("fx", 0)
            fy = cfg.get("fy", 0)
            cx = cfg.get("cx", 0)
            cy = cfg.get("cy", 0)
            depth_factor = cfg.get("depth_factor", 1000)
            self._sensors[name] = MockRealSense(
                name,
                height=height,
                width=width,
                frequency=frequency,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                depth_factor=depth_factor,
                mock_cfg=self._mock_config,
                visualize=cfg.get("visualize", False),
            )
            self._sensor_info[name] = CameraInfo(
                name + "_info",
                str(cfg["serial"]),
                cfg.get("fixed", False),
                cfg.get("attached_body", None),
                height,
                width,
                frequency,
                fx,
                fy,
                cx,
                cy,
                depth_factor,
                tf_link_to_cam=cfg.get("tf_link_to_cam", None),
                channel_name=name + "_info",
            )
        print("[Sensor] MOCK mode: cameras are MuJoCo renders, not RealSense devices.")
