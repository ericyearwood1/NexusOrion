import abc

from skills.dto import *

from base.data_provider import DataProvider

GET_EE_POSE = 'get_ee_pose'
GET_ROBOT_POSE = 'get_robot_pose'

LISTENERS = [GET_EE_POSE, GET_ROBOT_POSE]


class SkillsDataProvider(DataProvider):
    service_name = 'skills'

    @abc.abstractmethod
    def get_ee_pose(self) -> SkillsEEPoseData:
        raise NotImplementedError

    @abc.abstractmethod
    def get_robot_pose(self) -> SkillsRobotPoseData:
        raise NotImplementedError

    @abc.abstractmethod
    def get_location_detection(self) -> SkillsLocationDetectionData:
        raise NotImplementedError

    @abc.abstractmethod
    def get_navigation_highlight(self) -> SkillsNavigationHighlightData:
        raise NotImplementedError

    @abc.abstractmethod
    def get_detecting_objects(self) -> SkillsTargetObjectDetectionData:
        raise NotImplementedError

    @abc.abstractmethod
    def get_found_object(self) -> SkillsTargetObjectDetectionData:
        raise NotImplementedError

    @abc.abstractmethod
    def get_gripper_state(self) -> SkillsGripperStateData:
        raise NotImplementedError

    def register_listeners(self):
        self.on('get_ee_pose', self.on_get_ee_pose)
        self.on('get_robot_pose', self.on_get_robot_pose)
        self.on('get_navigation_highlight', self.on_get_navigation_highlight)
        self.on('get_location_detection', self.on_get_location_detection)
        self.on('get_detecting_objects', self.on_get_detecting_objects)
        self.on('get_found_object', self.on_get_found_object)
        self.on('get_gripper_state', self.on_get_gripper_state)

    def on_get_ee_pose(self, data=None):
        self.send_ee_pose(self.get_ee_pose())

    def on_get_robot_pose(self, data=None):
        self.send_robot_pose(self.get_robot_pose())

    def on_get_navigation_highlight(self, data=None):
        self.send_navigation_highlight(self.get_navigation_highlight())

    def on_get_location_detection(self, data=None):
        self.send_location_detection(self.get_location_detection())

    def on_get_detecting_objects(self, data=None):
        self.send_detecting_objects(self.get_detecting_objects())

    def on_get_found_object(self, data=None):
        self.send_found_object(self.get_found_object())

    def on_get_gripper_state(self, data=None):
        self.send_gripper_state(self.get_gripper_state())

    def send_ee_pose(self, state: SkillsEEPoseData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'ee_pose',
            'data': data
        })

    def send_robot_pose(self, state: SkillsRobotPoseData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'robot_pose',
            'data': data
        })

    def send_location_detection(self, state: SkillsLocationDetectionData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'location_detection',
            'data': data
        })

    def send_navigation_highlight(self, state: SkillsNavigationHighlightData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'navigation_highlight',
            'data': data
        })

    def send_gripper_state(self, state: SkillsGripperStateData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'gripper_state',
            'data': data
        })

    def send_detecting_objects(self, state: SkillsTargetObjectDetectionData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'detecting_objects',
            'data': data
        })

    def send_found_object(self, state: SkillsTargetObjectDetectionData):
        data = state.to_json()
        self.reply(json={
            'event': self.service_name,
            'type': 'found_object',
            'data': data
        })
