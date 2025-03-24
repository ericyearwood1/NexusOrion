from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Tuple
from utils.str_utils import (
    remap_od_object_labels_to_ui_object_labels,
    pretty_parse_object_or_furniture_str,
)

# The distance between the floor to the robot dock
FLOOR_TO_DOCK = 0.24


# Changes to data objects must be pushed upstream to the main repository
class WorldGraphOperation(Enum):
    NODE_ADDED = 'add/item'
    NODE_REMOVED = 'remove/item'
    ITEM_ADDED_TO_RECEPTACLE = 'add/receptacle'
    ITEM_REMOVED_FROM_RECEPTACLE = 'remove/receptacle'
    ITEM_ROOM_CHANGE = 'room_change'
    UNKNOWN = 'unknown'


@dataclass
class WorldGraphNode:
    object_id: str
    object_name: str
    category_tag: str
    room: str
    position: Tuple[float, float, float]
    extents: Tuple[float, float, float]
    yaw: float  # in degrees

    def to_json(self):
        return {
            'object_id': self.object_id,
            'object_name': self.object_name,
            'category_tag': self.category_tag,
            'room': self.room,
            'bbox_center': self.position,
            'bbox_extent': self.extents,
            'yaw': self.yaw
        }

    @staticmethod
    def from_json(json):
        position=json.get('position', json.get('bbox_center', (0, 0, 0)))
        z_offsetted_position = (position[0], position[1], position[2]+FLOOR_TO_DOCK)
        pretty_object_name = (
            remap_od_object_labels_to_ui_object_labels(pretty_parse_object_or_furniture_str(json.get('object_name', '')))
            if json.get('category_tag', '') == "object"
            else pretty_parse_object_or_furniture_str(json.get('object_name', ''))
        )
        return WorldGraphNode(
            object_id=json.get('object_id', ''),
            object_name=pretty_object_name,
            category_tag=json.get('category_tag', ''),
            room=json.get('room', ''),
            position=z_offsetted_position,
            extents=(0, 0, 0),
            yaw=json.get('yaw', 0)
        )


@dataclass
class WorldGraphEdit(WorldGraphNode):
    operation: WorldGraphOperation
    receptacle: str

    def to_json(self):
        result = super(WorldGraphEdit, self).to_json()
        result['operation'] = self.operation.value
        result['receptacle'] = self.receptacle
        return result

    @staticmethod
    def from_json(json):
        print(f"WorldGraphEdit.from_json: {json}")
        try:
            operation = WorldGraphOperation(json.get('operation', WorldGraphOperation.UNKNOWN.value))
        except ValueError:
            operation = WorldGraphOperation.UNKNOWN
            print(f'Unknown operation: {json.get("operation")}')

        pretty_object_name = (
            remap_od_object_labels_to_ui_object_labels(pretty_parse_object_or_furniture_str(json.get('object_name', '')))
            if json.get('category_tag', '') == "object"
            else pretty_parse_object_or_furniture_str(json.get('object_name', ''))
        )
        result = WorldGraphEdit(
            object_id=json.get('object_id', ''),
            object_name=pretty_object_name,
            category_tag=json.get('category_tag', ''),
            room=json.get('room', ''),
            position=json.get('position', json.get('bbox_center', (0, 0, 0))),
            yaw=json.get('yaw', 0),
            extents=(0, 0, 0),
            operation=operation,
            receptacle=json.get('receptacle', '')
        )
        return result


@dataclass
class WorldGraphEditData:

    status: str
    timestamp: str
    object: WorldGraphEdit

    def to_json(self):
        return {
            'status': self.status,
            'timestamp': self.timestamp,
            'object': self.object.to_json()
        }

    @staticmethod
    def from_json(json):
        return WorldGraphEditData(
            status=json.get('status', ''),
            timestamp=json.get('timestamp', ''),
            object=[WorldGraphEdit.from_json(obj) for obj in json.get('object', [])][0]
        )


@dataclass
class WorldGraphEdge:
    source: str
    target: str
    relation: str

    def to_json(self):
        return {
            'source': self.source,
            'target': self.target,
            'relation': self.relation
        }

    @staticmethod
    def from_json(json):
        return WorldGraphEdge(
            source=json.get('source', ''),
            target=json.get('target', ''),
            relation=json.get('relation', '')
        )


@dataclass
class WorldGraphData:
    status: str
    timestamp: str
    nodes: List[WorldGraphNode]
    edges: List[WorldGraphEdge]

    def to_json(self):
        return {
            'status': self.status,
            'timestamp': self.timestamp,
            'graph': {
                'nodes': [node.to_json() for node in self.nodes],
                'edges': [edge.to_json() for edge in self.edges]
            }
        }

    @staticmethod
    def from_json(json):
        return WorldGraphData(
            status=json.get('status', ''),
            timestamp=json.get('timestamp', ''),
            nodes=[WorldGraphNode.from_json(node) for node in json.get('graph', {}).get('nodes', [])],
            edges=[WorldGraphEdge.from_json(edge) for edge in json.get('graph', {}).get('edges', [])]
        )


@dataclass
class WorldGraphObjects:
    status: str
    timestamp: str
    object: List[Dict]

    def to_json(self):
        return {
            'status': self.status,
            'timestamp': self.timestamp,
            'object': self.object
        }

    @staticmethod
    def from_json(json):
        return WorldGraphObjects(
            status=json.get('status', ''),
            timestamp=json.get('timestamp', ''),
            object=json.get('object', [])
        )
