import abc
import json

from world_graph.dto import WorldGraphData, WorldGraphObjects, WorldGraphEditData

from base.data_provider import DataProvider

# use this as type in message
WORLD_GRAPH = 'world_graph'
WORLD_GRAPH_OBJECTS = 'world_graph_objects'

GET_WORLD_GRAPH = 'get_world_graph'
GET_WORLD_GRAPH_EDITS = 'get_world_graph_edits'
GET_WORLD_GRAPH_OBJECT_BY_TYPE = 'get_world_graph_object_by_type'
GET_WORLD_GRAPH_OBJECT_BY_NAME = 'get_world_graph_object_by_name'

LISTENERS = [GET_WORLD_GRAPH, GET_WORLD_GRAPH_EDITS, GET_WORLD_GRAPH_OBJECT_BY_TYPE, GET_WORLD_GRAPH_OBJECT_BY_NAME]


class WorldGraphDataProvider(DataProvider):
    service_name = 'world_graph'

    def send_world_graph_state(self, state: WorldGraphData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'world_graph',
            'data': data
        })

    def send_world_graph_objects(self, state: WorldGraphObjects):
        data = state.to_json()
        data['event'] = self.service_name
        data['type'] = 'world_graph_objects'
        self.reply(json={
            'event': self.service_name,
            'type': 'world_graph_objects',
            'data': data
        })

    def send_world_graph_edit(self, world_graph_edit: WorldGraphEditData):
        data = world_graph_edit.to_json()
        print(f"WorldGraphDataProvider.send_world_graph_edit: {data}")
        self.reply(json={
            'event': self.service_name,
            'type': 'edit',
            'data': data
        })

    @abc.abstractmethod
    def get_world_graph(self):
        raise NotImplementedError()

    @abc.abstractmethod
    def get_world_graph_edits(self, timestamp):
        raise NotImplementedError()

    @abc.abstractmethod
    def get_object_by_type(self, object_type):
        raise NotImplementedError()

    @abc.abstractmethod
    def get_object_by_name(self, object_name):
        raise NotImplementedError()

    def register_listeners(self):
        self.on(GET_WORLD_GRAPH, self.on_world_graph)
        self.on(GET_WORLD_GRAPH_EDITS, self.on_world_graph_edits)
        self.on(GET_WORLD_GRAPH_OBJECT_BY_TYPE, self.on_object_by_type)
        self.on(GET_WORLD_GRAPH_OBJECT_BY_NAME, self.on_object_by_name)

    def on_world_graph(self, data=None):
        self.send_world_graph_state(self.get_world_graph())

    def on_world_graph_edits(self, data=None):
        try:
            data = json.loads(data)
        except json.JSONDecodeError:

            return
        timestamp = data.get('timestamp')
        self.send_world_graph_objects(self.get_world_graph_edits(timestamp))

    def on_object_by_type(self, data=None):
        object_type = data.get('object_type')
        self.send_world_graph_objects(self.get_object_by_type(object_type))

    def on_object_by_name(self, data=None):
        object_name = data.get('object_name')
        self.send_world_graph_objects(self.get_object_by_name(object_name))

