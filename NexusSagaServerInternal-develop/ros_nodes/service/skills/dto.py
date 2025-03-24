import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Tuple


class GripperState(IntEnum):
    NotSet = 0
    Open = 1
    Closed = 2  # close without object
    Holding = 3  # close with object


# Changes to data objects must be pushed upstream to the main repository
@dataclass
class SkillsRobotInstructionData:
    timestamp: str
    commands: List[str]

    def __init__(self, commands: List[str], timestamp: str = None):
        if timestamp is None:
            timestamp = str(time.strftime('%Y-%m-%d %H:%M:%S'))
        self.timestamp = timestamp
        self.commands = commands

    def to_json(self):
        return {
            'timestamp': self.timestamp,
            'commands': self.commands
        }

    @staticmethod
    def from_json(json):
        return SkillsRobotInstructionData(
            timestamp=json.get('timestamp'),
            commands=json.get('commands')
        )


@dataclass
class SkillsRobotPoseData:
    timestamp: str
    position: Tuple[float, float, float]
    rotation: float

    def to_json(self):
        return {
            'timestamp': self.timestamp,
            'position': [self.position[0], self.position[1], self.position[2]],
            'yaw': self.rotation
        }

    @staticmethod
    def from_json(json):
        return SkillsRobotPoseData(
            timestamp=json.get('timestamp'),
            position=(json.get('pos_x'), json.get('pos_y'), json.get('pos_z')),
            rotation=json.get('yaw')
        )


@dataclass
class SkillsGripperStateData:
    timestamp: str
    state: GripperState

    def to_json(self):
        return {
            'timestamp': self.timestamp,
            'state': self.state
        }

    @staticmethod
    def from_json(json):
        return SkillsGripperStateData(
            timestamp=json.get('timestamp'),
            state=json.get('state')
        )


@dataclass
class YawPitchRoll:
    yaw: float
    pitch: float
    roll: float

    def to_json(self):
        return {
            'yaw': self.yaw,
            'pitch': self.pitch,
            'roll': self.roll
        }

    @staticmethod
    def from_json(json):
        return YawPitchRoll(
            yaw=json.get('yaw'),
            pitch=json.get('pitch'),
            roll=json.get('roll')
        )


@dataclass
class SkillsEEPoseData:
    timestamp: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    yaw_pitch_roll: YawPitchRoll

    def to_json(self):
        return {
            'timestamp': self.timestamp,
            'position': [self.position[0], self.position[1], self.position[2]],
            'orientation': [self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
            'yaw_pitch_roll': self.yaw_pitch_roll.to_json()
        }

    @staticmethod
    def from_json(json):
        return SkillsEEPoseData(
            timestamp=json.get('timestamp'),
            position=(json.get('pos_x'), json.get('pos_y'), json.get('pos_z')),
            orientation=(json.get('rot_x'), json.get('rot_y'), json.get('rot_z'), json.get('rot_w')),
            yaw_pitch_roll=YawPitchRoll.from_json(json)
        )


@dataclass
class SkillsLocationDetectionData:
    timestamp: str
    action_id: str  # the id of the place action that this relates to
    position: Tuple[float, float, float]
    target_orientation: Tuple[float, float, float, float]  # relative to robot body - pitch and roll both, maybe yaw
    target_euler: Tuple[float, float, float]  # relative to robot body - pitch and roll both, maybe yaw

    def to_json(self):
        return {
            'timestamp': self.timestamp,
            'position': [self.position[0], self.position[1], self.position[2]],
            'action_id': self.action_id,
            'target_orientation': [self.target_orientation[0], self.target_orientation[1], self.target_orientation[2],
                                   self.target_orientation[3]],
            'target_euler': [self.target_euler[0], self.target_euler[1], self.target_euler[2]]
        }

    @staticmethod
    def from_json(json):
        return SkillsLocationDetectionData(
            timestamp=json.get('timestamp'),
            action_id=json.get('action_id'),
            position=(json.get('pos_x'), json.get('pos_y'), json.get('pos_z')),
            target_orientation=(
            json.get('orientation_x', 0.0), json.get('orientation_y', 0.0), json.get('orientation_z', 0.0),
            json.get('orientation_w', 0.0)),
            target_euler=(json.get('euler_x', 0.0), json.get('euler_y', 0.0), json.get('euler_z', 0.0))
        )


@dataclass
class SkillsNavigationHighlightData:
    action_id: str
    timestamp: str
    target: str  # eg 123_cup
    position: Tuple[float, float, float]

    def to_json(self):
        return {
            'action_id': self.action_id,
            'timestamp': self.timestamp,
            'target': self.target,
            'position': [self.position[0], self.position[1], self.position[2]]
        }

    @staticmethod
    def from_json(json):
        return SkillsNavigationHighlightData(
            action_id=json.get('action_id'),
            timestamp=json.get('timestamp'),
            target=json.get('target'),
            position=(json.get('pos_x'), json.get('pos_y'), json.get('pos_z'))
        )


@dataclass
class TargetObjectData:
    position: Tuple[float, float, float]
    normal: Tuple[float, float, float]  # default to UP
    threshold: float

    def to_json(self):
        return {
            'position': [self.position[0], self.position[1], self.position[2]],
            'normal': [self.normal[0], self.normal[1], self.normal[2]],
            'threshold': self.threshold
        }

    @staticmethod
    def from_json(json):
        return TargetObjectData(
            position=(json.get('x'), json.get('y'), json.get('z')),
            normal=(json.get('normal_x'), json.get('normal_y'), json.get('normal_z')),
            threshold=json.get('threshold')
        )


@dataclass
class SkillsTargetObjectDetectionData:
    timestamp: str
    action_id: str  # hopefully unique identifier
    label: str  # name of the object
    targets: List[TargetObjectData]

    def to_json(self):
        return {
            'timestamp': self.timestamp,
            'action_id': self.action_id,
            'label': self.label,
            'targets': [target.to_json() for target in self.targets or []]
        }

    @staticmethod
    def from_json(json):
        return SkillsTargetObjectDetectionData(
            action_id=json.get('action_id'),
            timestamp=json.get('timestamp'),
            label=json.get('label'),
            targets=[TargetObjectData.from_json(target) for target in json.get('targets')]
        )
