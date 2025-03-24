import json
import logging
import time
from datetime import datetime

import base.heartbeat
import base.service
from skills.data_provider import SkillsDataProvider
from skills.dto import SkillsRobotPoseData, SkillsEEPoseData, SkillsNavigationHighlightData, \
    SkillsTargetObjectDetectionData, SkillsLocationDetectionData, SkillsGripperStateData, GripperState

logger = logging.getLogger('skills_service')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)

with open('stub_data/skill/ee_pose.json') as f:
    ee_pose = json.load(f)

# with open('stub_data/skill/robot_pose.json') as f:
#     robot_pose = json.load(f)

with open('stub_data/skill/navigation_highlight.json') as f:
    navigation_highlight = json.load(f)

with open('stub_data/skill/place_location_detection.json') as f:
    place_location_detection = json.load(f)

with open('stub_data/skill/found_object.json') as f:
    found_object = json.load(f)

with open('stub_data/skill/detecting_objects.json') as f:
    detecting_objects = json.load(f)


class StubSkillsDataProvider(SkillsDataProvider):
    _deg2Rad = 0.017453292
    _rad2Deg = 57.29578
    _ee_pose_index = 0
    _robot_position = (0, 0, 0)
    _robot_rotation = 0
    _robot_start_position = (0, 0, 0)
    _robot_end_position = (10, 0, 0)
    _robot_move_speed = 1
    _robot_returning = False
    _report_hertz = 30
    _report_timer = 0
    _highlight_hertz = 0.1
    _highlight_timer = 0
    _detect_objects_hertz = 0.1
    _detect_objects_timer = 0
    _detecting_objects = False
    _detecting_objects_duration = 1
    _detecting_objects_timer = 0

    # DataProvider methods
    def tick(self, delta):
        if self._robot_returning:
            move_speed = -self._robot_move_speed
        else:
            move_speed = self._robot_move_speed
        self._robot_position = (
            self._robot_position[0] + move_speed * delta,
            self._robot_position[1],
            self._robot_position[2],
        )
        if self._robot_position[0] >= self._robot_end_position[0]:
            self._robot_returning = True
        elif self._robot_position[0] <= self._robot_start_position[0]:
            self._robot_returning = False

        self._report_timer += delta
        if self._report_timer >= 1 / self._report_hertz:
            self._report_timer -= 1 / self._report_hertz
            self.on_get_robot_pose()

        # Or when you have highlight data to send
        self._highlight_timer += delta
        if self._highlight_timer >= 1 / self._highlight_hertz:
            self._highlight_timer -= 1 / self._highlight_hertz
            self.send_navigation_highlight(self.get_navigation_highlight())

        if not self._detecting_objects:
            self._detect_objects_timer += delta
            if self._detect_objects_timer >= 1 / self._detect_objects_hertz:
                self._detect_objects_timer -= 1 / self._detect_objects_hertz
                self._detecting_objects = True
                self._detecting_objects_timer = 0
                self.send_detecting_objects(self.get_detecting_objects())
        else:
            self._detecting_objects_timer += delta
            if self._detecting_objects_timer >= self._detecting_objects_duration:
                self._detecting_objects = False
                self.send_found_object(self.get_found_object())

    # SkillsDataProvider methods
    def get_ee_pose(self) -> SkillsEEPoseData:
        self._ee_pose_index += 1
        if self._ee_pose_index >= len(ee_pose):
            self._ee_pose_index = 0
        return SkillsEEPoseData.from_json(ee_pose[self._ee_pose_index])

    def get_robot_pose(self) -> SkillsRobotPoseData:
        return SkillsRobotPoseData(
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            position=self._robot_position,
            rotation=self._robot_rotation * self._deg2Rad,
        )

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')

    def get_navigation_highlight(self) -> SkillsNavigationHighlightData:
        # Build the data object from json
        return SkillsNavigationHighlightData.from_json({
            "action_id": "the_id",
            "timestamp": datetime.now().isoformat(),
            "target": "the_target",
            "pos_x": 0,
            "pos_y": 0,
            "pos_z": 0,

        })
        # Or directly instantiate the data class
        # return SkillsNavigationHighlightData(
        #     id='the_id',
        #     timestamp=datetime.now().isoformat(),
        #     target='the_target',
        #     position=(0, 0, 0),
        # )

    def get_location_detection(self) -> SkillsLocationDetectionData:
        return SkillsLocationDetectionData.from_json(place_location_detection)

    def get_detecting_objects(self) -> SkillsTargetObjectDetectionData:
        return SkillsTargetObjectDetectionData.from_json(detecting_objects)

    def get_found_object(self) -> SkillsTargetObjectDetectionData:
        return SkillsTargetObjectDetectionData.from_json(found_object)

    def get_gripper_state(self) -> SkillsGripperStateData:
        return SkillsGripperStateData(
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            state=GripperState.NotSet
        )


class StubSkillsService(base.service.Service):
    host = '0.0.0.0'
    port = 7504
    data_provider_class = StubSkillsDataProvider
    logger = logger


if __name__ == '__main__':
    StubSkillsService().start()
