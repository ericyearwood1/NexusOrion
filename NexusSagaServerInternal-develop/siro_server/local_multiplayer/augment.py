import typing
import uuid as py_uuid
from dataclasses import dataclass

from local_multiplayer import enums


@dataclass
class Augment:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    scale: float
    augment_type: enums.AugmentType
    uuid: str = None

    def __post_init__(self):
        self.uuid = str(py_uuid.uuid4())
