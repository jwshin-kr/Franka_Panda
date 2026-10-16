"""Controller subscriber manager: arm/hand joints + collision info + a 3D point
from perception (point_set_t, e.g. the tracked point of TapnetClickAndTrack)."""

from queue import Queue

from communication.lcm.subscriber.data_subscriber.Point3DSubscriber import (
    Point3DSubscriber,
)
from communication.lcm.subscriber.sub_manager.controller.ArmHandColInfoSubManager import (
    ArmHandColInfoSubManager,
)


class ArmHandColInfoPointSubManager(ArmHandColInfoSubManager):
    """``ArmHandColInfoSubManager`` plus ``Point3DData`` on
    ``extr_sub_que_dict[point_channel]``."""

    def __init__(self, *args, point_channel: str = "click_and_point", **kwargs):
        super().__init__(*args, **kwargs)
        self.data_queue_dict["extr_sub_que_dict"][point_channel] = Queue()
        Point3DSubscriber(
            self._lcm_instance,
            self.data_queue_dict["extr_sub_que_dict"][point_channel],
        ).subscribe(point_channel)
