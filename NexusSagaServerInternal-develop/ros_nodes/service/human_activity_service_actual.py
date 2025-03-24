import json
import logging
import random
import threading

import rospy
from std_msgs.msg import String

import base.heartbeat
import base.service
import human_activity.data_provider
from human_activity import dto
from utils.str_utils import (
    pretty_parse_object_or_furniture_str,
    remap_od_object_labels_to_ui_object_labels,
)
from multiprocessing import Queue
from typing import Dict, Any

logger = logging.getLogger('human_activity_service')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)


class HumanActivityDataProviderActual(human_activity.data_provider.HumanActivityDataProvider):
    # human_activity_messages = human_activity_messages
    # __current_message_index = 0
    # __advance_message_chance_per_second = 0.05

    def __init__(self, *args, **kwargs):
        super().__init__(service=kwargs["service"])

        self._current_human_activity : Dict = {}
        self._last_picked_object : Any[str] = None

        # Create multiprocessing Queue for sharing msgs
        self.har_msg_queue = Queue()

        # Define the subscriber in the other thread
        thread_subscriber = threading.Thread(target=self._ros_code)
        thread_subscriber.start()

    def _ros_code(self):
        rospy.init_node("human_activity_service",disable_signals=True)
        rospy.Subscriber(
            "human_activity_current", String, self._get_current_human_activity, queue_size=10
        )

    def _get_current_human_activity(self, data: String):
        current_human_activity_string = data.data
        try:
            current_human_activity_json = json.loads(current_human_activity_string)
            self.har_msg_queue.put(
                {
                    "msg": "current_human_activity",
                    "data": current_human_activity_json,
                }
            )
        except json.JSONDecodeError:
            print("Received message in _get_current_human_activity but it was malformed")
            print(current_human_activity_string)
            raise RuntimeError

    def get_human_activity(self) -> dto.HumanActivityData:
        har_output_str = ""
        if self._current_human_activity is not None:
            activity = self._current_human_activity.get("action", "")
            if "pick" in activity:
                object_str = remap_od_object_labels_to_ui_object_labels(
                    pretty_parse_object_or_furniture_str(self._current_human_activity.get("object", ""))
                )
                har_output_str = "Human agent picked " + object_str
                self._last_picked_object = object_str
            elif "place" in activity:
                har_output_str = "Human agent placed " + self._last_picked_object
                self._last_picked_object = ""
            else:
                print(f"Received invalid activity inside get_human_activity : '{activity}' . This is not supported yet")

        return dto.HumanActivityData(
            activity=har_output_str,
            _object="",
            receptacle="",
        )

    def tick(self, delta):
        # if random.random() < self.__advance_message_chance_per_second * delta:
        #     self.__current_message_index += 1
        #     if self.__current_message_index >= len(self.human_activity_messages):
        #         self.__current_message_index = 0
        if not self.har_msg_queue.empty():
            next_msg = self.har_msg_queue.get()
            print(next_msg)
            if next_msg["msg"] == "current_human_activity":
                self._current_human_activity = next_msg["data"]
                self.send_human_activity_state(self.get_human_activity())

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')

class HumanActivityServiceActual(base.service.Service):
    host = '0.0.0.0'
    port = 7502
    data_provider_class = HumanActivityDataProviderActual
    logger = logger


if __name__ == '__main__':
    HumanActivityServiceActual().start()
