import json
import logging

import base.heartbeat
import base.service
import world_graph.dto
import world_graph.data_provider

logger = logging.getLogger('world_graph_service')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)

with open('stub_data/world_graph/response_world_graph.json') as f:
    response_world_graph = json.load(f)

with open('stub_data/world_graph/response_world_graph_edits.json') as f:
    response_world_graph_edits = json.load(f)

with open('stub_data/world_graph/response_object_by_type_failure.json') as f:
    response_object_by_type_failure = json.load(f)

with open('stub_data/world_graph/response_object_by_type_success.json') as f:
    response_object_by_type_success = json.load(f)

with open('stub_data/world_graph/response_one_object_by_name_failure.json') as f:
    response_one_object_by_name_failure = json.load(f)

with open('stub_data/world_graph/response_one_object_by_name_success.json') as f:
    response_one_object_by_name_success = json.load(f)

with open('stub_data/world_graph/response_one_object_by_name_success_without_receptacle.json') as f:
    response_one_object_by_name_success_without_receptacle = json.load(f)


class StubWorldGraphDataProvider(world_graph.data_provider.WorldGraphDataProvider):
    _update_frequency = 0.1
    _timer = 0
    _edit_index = 0

    # DataProvider methods
    def tick(self, delta):
        self._timer += delta
        if self._timer >= 1 / self._update_frequency:
            self._timer = 0
            self.send_world_graph_edit(self.get_world_graph_edits(None))

    # WorldGraphDataProvider methods
    def get_world_graph_edits(self, timestamp) -> world_graph.dto.WorldGraphEditData:
        result = world_graph.dto.WorldGraphEditData.from_json(response_world_graph_edits[self._edit_index])
        self._edit_index += 1
        if self._edit_index >= len(response_world_graph_edits):
            self._edit_index = 0

        return result

    def get_object_by_type(self, object_type) -> world_graph.dto.WorldGraphObjects:
        if object_type == "failure":
            return world_graph.dto.WorldGraphObjects.from_json(response_object_by_type_failure)
        else:
            return world_graph.dto.WorldGraphObjects.from_json(response_object_by_type_success)

    def get_object_by_name(self, object_name) -> world_graph.dto.WorldGraphObjects:
        if object_name == "failure":
            return world_graph.dto.WorldGraphObjects.from_json(response_one_object_by_name_failure)
        elif object_name == "success_without_receptacle":
            return world_graph.dto.WorldGraphObjects.from_json(response_one_object_by_name_success_without_receptacle)
        else:
            return world_graph.dto.WorldGraphObjects.from_json(response_one_object_by_name_success)

    def get_world_graph(self) -> world_graph.dto.WorldGraphData:
        return world_graph.dto.WorldGraphData.from_json(response_world_graph)

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')


class StubWorldGraphService(base.service.Service):
    host = '0.0.0.0'
    port = 7503
    data_provider_class = StubWorldGraphDataProvider
    logger = logger


if __name__ == '__main__':
    StubWorldGraphService().start()
