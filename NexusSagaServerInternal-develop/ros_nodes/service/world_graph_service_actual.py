import json
import os
import logging
import threading

import rospy
from std_msgs.msg import String

import base.heartbeat
import base.service
import world_graph.dto
import world_graph.data_provider
from multiprocessing import Queue

logger = logging.getLogger("world_graph_service")
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format="%(asctime)s %(name)-25s %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S",
)


class ActualWorldGraphDataProvider(world_graph.data_provider.WorldGraphDataProvider):
    def __init__(self, *args, **kwargs):
        super().__init__(service=kwargs["service"])

        self._world_graph_state: dict = {}
        self._world_graph_edits = []
        self._wg_path = os.path.expanduser("~/received_wg.json")

        self._test: bool = True

        # Create multiprocessing Queue for sharing msgs
        self.wg_msg_queue = Queue()

        # Define the subscriber in the other thread
        thread_subscriber = threading.Thread(target=self._ros_code)
        thread_subscriber.start()

    def _ros_code(self):
        rospy.init_node("world_graph_service", disable_signals=True)
        rospy.Subscriber(
            "/world_graph_state", String, self._get_world_graph_state, queue_size=10
        )
        rospy.Subscriber(
            "/world_graph_edits", String, self._get_world_graph_edits, queue_size=10
        )

    def _get_world_graph_state(self, data: String):
        wg_string = data.data
        try:
            wg_json = json.loads(wg_string)
            self.wg_msg_queue.put(
                {
                    "msg": "world_graph_state",
                    "data": wg_json,
                }
            )
            with open(self._wg_path, "w") as f:
                json.dump(wg_json, f)
        except json.JSONDecodeError:
            print("Received message in _get_world_graph_edits but it was malformed")
            print("Msg: ", wg_string)
            raise RuntimeError
        if self._test:
            print("In _get_world_graph_state")
            print(wg_json)

    def _get_world_graph_edits(self, data: String):
        wg_edit_string = data.data
        try:
            wg_edits_json = json.loads(wg_edit_string)
            self.wg_msg_queue.put(
                {
                    "msg": "world_graph_edits",
                    "data": wg_edits_json,
                }
            )
        except json.JSONDecodeError:
            print("Received message in _get_world_graph_edits but it was malformed")
            print(wg_edit_string)
            raise RuntimeError
        if self._test:
            print("In _get_world_graph_edits")
            print(wg_edits_json)

    def tick(self, delta: float):
        # print("Len of Q", self._queue.qsize())
        # if self._world_graph_state:
        #     print(self._world_graph_state.keys())
        #     print(self._world_graph_state.get('status', None))
        if not self.wg_msg_queue.empty():
            next_msg = self.wg_msg_queue.get()
            print(next_msg)
            if next_msg["msg"] == "world_graph_state":
                self._world_graph_state = next_msg["data"]
                self.send_world_graph_state(self.get_world_graph())

            elif next_msg["msg"] == "world_graph_edits":
                self._world_graph_edits.append(next_msg["data"])
                print("WG Edits data : \n", next_msg["data"])
                self.send_world_graph_edit(
                    world_graph.dto.WorldGraphEditData.from_json(next_msg["data"])
                )

    def get_world_graph(self) -> world_graph.dto.WorldGraphData:
        if self._world_graph_state:
            print(self._world_graph_state.keys())
            return world_graph.dto.WorldGraphData.from_json(self._world_graph_state)
        else:
            with open(self._wg_path, "r") as f:
                return world_graph.dto.WorldGraphData.from_json(json.load(f))

    def get_world_graph_edits(self, timestamp):
        while len(self._world_graph_edits) > 0:
            msg = self._world_graph_edits.pop(0)
            return world_graph.dto.WorldGraphEditData.from_json(msg)

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status="available")

    def get_object_by_type(self, object_type) -> world_graph.dto.WorldGraphObjects:
        pass

    def get_object_by_name(self, object_name) -> world_graph.dto.WorldGraphObjects:
        pass


class ActualWorldGraphService(base.service.Service):
    host = "0.0.0.0"
    port = 7503
    data_provider_class = ActualWorldGraphDataProvider
    logger = logger


if __name__ == "__main__":
    ActualWorldGraphService().start()
