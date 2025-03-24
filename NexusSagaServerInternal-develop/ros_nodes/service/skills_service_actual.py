import json
import logging
import time
from datetime import datetime
from multiprocessing import Queue
import numpy as np
import base.heartbeat
import base.service
from skills.data_provider import SkillsDataProvider
from skills.dto import SkillsRobotPoseData, SkillsEEPoseData, SkillsNavigationHighlightData, \
    SkillsTargetObjectDetectionData, SkillsLocationDetectionData, TargetObjectData, \
    SkillsGripperStateData, GripperState, YawPitchRoll
from utils.str_utils import (
    pretty_parse_object_or_furniture_str,
    remap_od_object_labels_to_ui_object_labels,
)

# spot-sim2real stuff
import sys
from spot_wrapper.spot import Spot
from spot_rl.utils.utils import get_skill_name_and_input_from_ros
from spot_rl.utils.utils import ros_topics as rt
from bosdyn.client.frame_helpers import get_vision_tform_body
from std_msgs.msg import Float32MultiArray, String
import rospy
import threading

thismodule = sys.modules[__name__]
thismodule.x = 0

logger = logging.getLogger('skills_service')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)
# The distance between the floor to the robot dock
FLOOR_TO_DOCK = 0.24


class ActualSkillsDataProvider(SkillsDataProvider):
    _ee_pose_index = 0
    _robot_position = (0, 0, 0)
    _robot_rotation = 0
    _robot_ee_pose_position = (0, 0, 0)
    _robot_ee_pose_rotation = (0, 0, 0, 0)
    _action_id = "None[none]"
    _pre_action_id = "None[none]"
    _has_sent_navigation_highlight = False
    _has_sent_location_detection = False
    _has_sent_detecting_objects = False
    _has_sent_found_object = False
    _navigation_highlight_target = "None"
    _navigation_highlight_position = (0, 0, 0)
    _location_detection_position = (0, 0, 0)
    _target_euler = (0, 0, 0)
    _target_object_detection_label = []
    _target_object_detection_object_info = []
    _detecting_objects_label = ""
    _detecting_objects_targets = []
    _gripper_state = GripperState.NotSet
    _last_gripper_state = GripperState.NotSet
    _report_hertz = 100
    _report_timer = 0

    _enable_target_object_circles = False
    _enable_target_object_markers = True
    _enable_nav_highlight_markers = True
    _enable_semantic_place_highlights = False

    def __init__(self, *args, **kwargs):
        super().__init__(service=kwargs["service"])

        # Need lock for pick skill data
        self.lock = threading.Lock() 

        # Create spot object
        self.spot = Spot("SpotSkillsDataProvider")
  
        # Define the subscriber in the other thread
        thread_subscriber = threading.Thread(target=self.subscriber)
        thread_subscriber.start()

        # This is the function that reads from Spot object
        robot_info_subscriber = threading.Thread(target=self.robot_info_from_spot_object)
        robot_info_subscriber.start()

        self._queue_skill_state = Queue()
        thread_subscriber_skill_state = threading.Thread(target=self.subscriber_skill_state)
        thread_subscriber_skill_state.start()

    def subscriber(self):
        """A function to init the ros node and subscribe things"""
        rospy.init_node("nexus_robot", disable_signals=True)
        # Disable method for using ros to fetch robot info 
        # rospy.Subscriber(
        #     rt.ROBOT_STATE,
        #     Float32MultiArray,
        #     self.robot_info_sending_callback_manager,
        #     queue_size=1,
        # )
        rospy.Subscriber(
            rt.OPEN_VOC_OBJECT_DETECTOR_TOPIC,
            String,
            self.target_object_detection_callback,
            queue_size=1,
        )

    def target_object_detection_callback(self, msg):
        """Call back function for the target object detection"""
        with self.lock:
            detections = msg.data
            self._target_object_detection_label = []
            self._target_object_detection_object_info = []
            if detections != "":
                detections = detections.split(";")
                # glass bottle,0.7982363700866699,0.04438495635986328, 0.28107717633247375;
                # cup,1.1067100763320923,-0.12386636435985565,0.2462521493434906
                for detection in detections:
                    # In dock (global) frame
                    dets = detection.split(",")
                    if len(dets) == 4: # Temp solution to make it compatible with spot-sim2real
                        score = 0.5
                        label, x, y, z = dets
                    elif len(dets) == 5:
                        label, x, y, z, score = dets
                    else:
                        label, x, y, z, score, ltx, lty, rbx, rby = dets
                    self._target_object_detection_label.append(label)
                    info = {}
                    info["position"] = (float(x), float(y), float(z)+FLOOR_TO_DOCK)
                    info["normal"] = (0, 0, 1)
                    info["threshold"] = float(score)
                    self._target_object_detection_object_info.append(info)

    def robot_info_sending_callback_manager(self, msg):
        """Call back function for sending robot info from ros"""
        x, y, yaw = msg.data[:3]
        _x, _y, _yaw = self.spot.xy_yaw_global_to_home(x, y, yaw)

        # The method to fetch ee pose using ros
        #_ee_position = msg.data[-7:-4]
        #_ee_orientation = msg.data[-4:]

        # The method to fetch ee pose using spot
        _ee_position, _ee_orientation = self.spot.get_ee_pos_in_body_frame()

        # Offset the height
        # robot_state = self.spot.robot_state_client.get_robot_state()
        # robot_state_kin = robot_state.kinematic_state
        # robot_tform = get_vision_tform_body(robot_state_kin.transforms_snapshot)
        dock_to_base = 0.0 # This value is usually close to zero

        _ee_position[2] += dock_to_base + FLOOR_TO_DOCK
        _ee_orientation = [np.rad2deg(v) for v in _ee_orientation]

        _z = dock_to_base + FLOOR_TO_DOCK
        
        _info = {"x": _x, "y": _y, "z": _z, "yaw": _yaw, "position": _ee_position, "orientation": _ee_orientation}
        
        # Now we send ee pose all the time
        _info["send_ee_pose"] = True
        _info["timestamp"] = time.time()

        self._queue.put(_info)

    def robot_info_from_spot_object(self):
        """Call back function for sending robot info from Spot object"""
        while True:
            # Sometimes cannot reach the robot in first few frames
            # Use try catch here to try more time
            try:
                _x, _y, _yaw = self.spot.get_xy_yaw()

                # The method to fetch ee pose using spot
                _ee_position, _ee_orientation = self.spot.get_ee_pos_in_body_frame()
                dock_to_base = 0.0 # This value is usually close to zero

                _ee_position[2] += dock_to_base + FLOOR_TO_DOCK
                _ee_orientation = [np.rad2deg(v) for v in _ee_orientation]

                _z = dock_to_base + FLOOR_TO_DOCK
                
                _info = {"x": _x, "y": _y, "z": _z, "yaw": _yaw, "position": _ee_position, "orientation": _ee_orientation}
                
                # Now we send ee pose all the time
                _info["send_ee_pose"] = True
                _info["timestamp"] = time.time()
                self._queue.put(_info)
            except Exception:
                pass

    def subscriber_skill_state(self):
        while True:
            _info = {}
            _info = self.check_sending_condition(_info)
            self._queue_skill_state.put(_info)

    def check_sending_condition(self, info):
        """Determine if we want to send the certain info"""
        skill_name, skill_input = get_skill_name_and_input_from_ros()
        info["action_id"] = f"{skill_name}[{skill_input}]"

        # We send location_detection when the skill is place
        info["send_location_detection"] = False
        if self._enable_semantic_place_highlights and skill_name in ["place"]:
            # Relative to the robot base
            place_target_xyz = rospy.get_param("place_target_xyz", "None,None,None|")
            place_target_xyz = place_target_xyz.split("|")[-2] # only get the last one
            place_target_xyz = place_target_xyz.split(",")
            robot_target_ee_rpy = rospy.get_param("robot_target_ee_rpy", "None,None,None")
            robot_target_ee_rpy = robot_target_ee_rpy.split(",")
            if "None" not in place_target_xyz and "None" not in robot_target_ee_rpy: # ready to show the location detection
                robot_target_ee_rpy = [float(v) for v in robot_target_ee_rpy]
                info["location_detection_position"] = [float(place_target_xyz[0]), float(place_target_xyz[1]), float(place_target_xyz[2])+FLOOR_TO_DOCK]
                info["target_euler"] = robot_target_ee_rpy
                info["send_location_detection"] = True

        # We send navigation highlight when the skill is nav
        info["send_navigation_highlight"] = False
        if self._enable_nav_highlight_markers and skill_name in ["nav", "nav_path_planning_with_view_poses", "explore"]:
            # nav_target_xyz = rospy.get_param("nav_target_xyz", "None,None,None|")
            nav_target_xyz_name = rospy.get_param("/nexus_nav_highlight", "None;None;None;None")
            nav_target_xyz_name = nav_target_xyz_name.split(";")
            if "None" not in nav_target_xyz_name: # ready to show the navigation highlight
                # nav_target_str = skill_input.split(";")[-1]
                info["navigation_highlight_target"] = pretty_parse_object_or_furniture_str(nav_target_xyz_name[-1])
                info["navigation_highlight_position"] = [float(nav_target_xyz_name[0]), float(nav_target_xyz_name[1]), 0.35] # anchor nav-highlight on ground
                info["send_navigation_highlight"] = True
                rospy.set_param("/nexus_nav_highlight", "None;None;None;None")

        # Copy target object data & target object into data within a lock
        target_object_detection_label_list = []
        target_object_detection_object_info_list = []
        with self.lock:
            target_object_detection_label_list = self._target_object_detection_label.copy()
            target_object_detection_object_info_list = self._target_object_detection_object_info.copy()

        # We send target object detection circles when the skill is pick
        info["send_target_object_detection"] = False
        if self._enable_target_object_circles and skill_name in ["pick"]:
            if target_object_detection_label_list != [] and target_object_detection_object_info_list != []:

                matching_idx = -1
                max_score = -1.0
                for i in range(len(target_object_detection_label_list)):
                    # Check for matching label between current detection and skill input
                    if skill_input == target_object_detection_label_list[i]:
                        # Check for score and only override if found max score
                        if target_object_detection_object_info_list[i].get("threshold",-1.0) > max_score:
                            max_score = target_object_detection_object_info_list[i].get("threshold",-1.0)
                            matching_idx = i

                if matching_idx == -1 or (matching_idx >= len(target_object_detection_label_list) and matching_idx >= len(target_object_detection_object_info_list)):
                    info["target_object_detection_label"] = "" # only the detection that matches the skill input target
                    info["target_object_detection_targets"] = []
                    info["send_target_object_detection"] = False
                else:
                    target_object_detection_label = target_object_detection_label_list[matching_idx] # only the detection that matches the skill input target
                    target_object_ui_label = remap_od_object_labels_to_ui_object_labels(target_object_detection_label) # Parse this label through name remapping function
                    info["target_object_detection_label"] = target_object_ui_label
                    info["target_object_detection_targets"] = [target_object_detection_object_info_list[matching_idx]]
                    info["send_target_object_detection"] = True

        # We send target object detection when the skill is pick
        info["send_found_object"] = False
        # Get the momment when we lock on the object
        pick_lock_on = rospy.get_param("/pick_lock_on", False)
        if self._enable_target_object_markers and skill_name in ["pick", "Open"] and pick_lock_on:
            if target_object_detection_label_list != [] and target_object_detection_object_info_list != []:

                matching_idx = -1
                max_score = -1.0
                for i in range(len(target_object_detection_label_list)):
                    # Check for matching label between current detection and skill input
                    if skill_input == target_object_detection_label_list[i]:
                        # Check for score and only override if found max score
                        if target_object_detection_object_info_list[i].get("threshold",-1.0) > max_score:
                            max_score = target_object_detection_object_info_list[i].get("threshold",-1.0)
                            matching_idx = i

                if matching_idx == -1 or (matching_idx>= len(target_object_detection_label_list) and matching_idx >= len(target_object_detection_object_info_list)):
                    info["target_object_detection_label"] = "" # only the detection that matches the skill input target
                    info["target_object_detection_targets"] = []
                    info["send_target_object_detection"] = False
                else:
                    target_object_detection_label = target_object_detection_label_list[matching_idx] # only the detection that matches the skill input target
                    target_object_ui_label = remap_od_object_labels_to_ui_object_labels(target_object_detection_label) # Parse this label through name remapping function
                    info["target_object_detection_label"] = target_object_ui_label
                    info["target_object_detection_targets"] = [target_object_detection_object_info_list[matching_idx]]
                    info["send_found_object"] = True

        # We only send gripper state if there is a change
        info["gripper_state"] = GripperState.NotSet
        info["send_gripper_state"] = False
        # This value is between 0 and 100
        try:
            _gripper_open_percentage = self.spot.robot_state_client.get_robot_state().manipulator_state.gripper_open_percentage
            _is_gripper_holding_item = self.spot.robot_state_client.get_robot_state().manipulator_state.is_gripper_holding_item
        except Exception:
            _gripper_open_percentage = 100
            _is_gripper_holding_item = False
            print("Cannot fetch robot state for gripper")
        if _is_gripper_holding_item:
            info["gripper_state"] = GripperState.Holding
        elif _gripper_open_percentage > 95:
            info["gripper_state"] = GripperState.Open
        else:
            info["gripper_state"] = GripperState.Holding # GripperState.Closed

        # Check the change
        if self._last_gripper_state != info["gripper_state"]:
            self._last_gripper_state = info["gripper_state"]
            info["send_gripper_state"] = True

        info["timestamp"] = time.time()
        return info

    # DataProvider methods
    def tick(self, delta):
        # Sending the ee and robot pose
        send_robot_info = False
        if not self._queue.empty():
            robot_info = self._queue.get()
            send_robot_info = True

        send_skill_state_info = False
        if not self._queue_skill_state.empty():
            skill_state_info = self._queue_skill_state.get()
            send_skill_state_info = True

        if send_robot_info:
            # get robot pose
            self._robot_position = (robot_info["x"], robot_info["y"],  robot_info["z"])
            self._robot_rotation = robot_info["yaw"]        
            self.on_get_robot_pose()

            # get ee pose during the duration of the pick/place skills
            _send_ee_pose = robot_info["send_ee_pose"]
            if _send_ee_pose:
                self._robot_ee_pose_position = robot_info["position"]
                self._robot_ee_pose_rotation = robot_info["orientation"]
                self.on_get_ee_pose()

        if send_skill_state_info:        
            self._action_id = skill_state_info["action_id"]

            # get navigation highlight only once right after we call nav skills
            _send_navigation_highlight = skill_state_info["send_navigation_highlight"]
            if _send_navigation_highlight and not self._has_sent_navigation_highlight:
                self._navigation_highlight_target = skill_state_info["navigation_highlight_target"]
                self._navigation_highlight_position = skill_state_info["navigation_highlight_position"]
                print(f"_send_navigation_highlight {self._navigation_highlight_target} {self._navigation_highlight_position}")
                self.on_get_navigation_highlight()
                self._has_sent_navigation_highlight = True

            # get location detection only once right after we call place skills
            _send_location_detection = skill_state_info["send_location_detection"]
            if _send_location_detection and not self._has_sent_location_detection:
                self._location_detection_position = skill_state_info["location_detection_position"]
                print(f"_send_location_detection {self._location_detection_position}")
                self._target_euler = skill_state_info["target_euler"]
                print(f"_target_euler {self._target_euler}")
                self.on_get_location_detection()
                self._has_sent_location_detection = True

            # get target object detection only once right after we call pick skills
            _send_target_object_detection = skill_state_info["send_target_object_detection"]
            if _send_target_object_detection and not self._has_sent_detecting_objects:
                self._detecting_objects_label = skill_state_info["target_object_detection_label"]
                self._detecting_objects_targets = skill_state_info["target_object_detection_targets"]
                print(f"_send_target_object_detection {self._detecting_objects_label} {self._detecting_objects_targets}")
                self.on_get_detecting_objects()
                self._has_sent_detecting_objects = True

            # get target object detection only once right after we find one object
            _send_found_object = skill_state_info["send_found_object"]
            if _send_found_object and not self._has_sent_found_object:
                self._detecting_objects_label = skill_state_info["target_object_detection_label"]
                self._detecting_objects_targets = skill_state_info["target_object_detection_targets"]
                print(f"_send_found_object {self._detecting_objects_label} {self._detecting_objects_targets}")
                self.on_get_found_object()
                self._has_sent_found_object = True

            # get the gripper state
            _send_gripper_state = skill_state_info["send_gripper_state"]
            if _send_gripper_state:
                self._gripper_state = skill_state_info["gripper_state"]
                print(f"_send_gripper_state: {self._gripper_state}")
                self.on_get_gripper_state()

            # Reset the flag that controls sending of the instruction
            if not _send_navigation_highlight:
                self._has_sent_navigation_highlight = False
            if not _send_location_detection:
                self._has_sent_location_detection = False
            if not _send_target_object_detection:
                self._has_sent_detecting_objects = False
            if not _send_found_object:
                self._has_sent_found_object = False

            self._pre_action_id = self._action_id

    def get_ee_pose(self) -> SkillsEEPoseData:
        # ee pose is relative to the body (robot) frame
        return SkillsEEPoseData(
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            position=self._robot_ee_pose_position,
            orientation=[0.0,0.0,0.0,1.0],
            yaw_pitch_roll=YawPitchRoll(
                yaw=self._robot_ee_pose_rotation[2],
                pitch=self._robot_ee_pose_rotation[1],
                roll=self._robot_ee_pose_rotation[0],
            )
        )
    
    def get_robot_pose(self) -> SkillsRobotPoseData:
        return SkillsRobotPoseData(
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            position=self._robot_position,
            rotation=self._robot_rotation,
        )

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')

    def get_navigation_highlight(self) -> SkillsNavigationHighlightData:
        # Build the data object from json
        return SkillsNavigationHighlightData(
            action_id=self._action_id,
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            target=self._navigation_highlight_target,
            position=self._navigation_highlight_position,
        )

    def get_location_detection(self) -> SkillsLocationDetectionData:
        return SkillsLocationDetectionData(
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            position=self._location_detection_position,
            action_id=self._action_id,
            target_orientation=(0,0,0,0),
            target_euler=self._target_euler
        )

    def get_detecting_objects(self) -> SkillsTargetObjectDetectionData:
        infos = []
        for _info in self._detecting_objects_targets:
            info = TargetObjectData(
                position=_info["position"],
                normal=_info["normal"],
                threshold=_info["threshold"],
            )
            infos.append(info)
        return SkillsTargetObjectDetectionData(
            action_id=self._action_id,
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            label=self._detecting_objects_label,
            targets=infos
        )

    def get_found_object(self) -> SkillsTargetObjectDetectionData:
        result = self.get_detecting_objects()
        return result

    def get_gripper_state(self) -> SkillsGripperStateData:
        return SkillsGripperStateData(
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            state=self._gripper_state,
        )
    
class AcutalSkillsService(base.service.Service):
    host = '0.0.0.0'
    port = 7504
    data_provider_class = ActualSkillsDataProvider
    logger = logger


if __name__ == '__main__':
    AcutalSkillsService().start()
