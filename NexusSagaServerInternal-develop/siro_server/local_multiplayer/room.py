import logging
from dataclasses import asdict

from flask import request

import local_multiplayer
from local_multiplayer import User
from local_multiplayer.spatial_anchor import SpatialAnchor

logger = logging.getLogger('siro_ws_server')


class Room:

    space_sync_anchor: SpatialAnchor or None
    robot_home_anchor: SpatialAnchor or None

    def __init__(self, room_id):
        self.id = room_id
        self.users = []
        self.active_features = []
        self.world_graph_anchors = []
        self.space_sync_anchor = None
        self.robot_home_anchor = None
        self.user_sids = set()
        self.is_robot_feature_available = True
        self.is_augment_feature_available = True
        self.augments = {}

    def contains_user(self, sid):
        return sid in self.user_sids

    def get_user_by_sid(self, sid) -> User or None:
        for user in self.users:
            if user.sid == sid:
                return user
        return None

    def add_active_feature(self, feature_id):
        for feature in self.active_features:
            if feature == feature_id:
                return
        self.active_features.append(feature_id)

    def remove_active_feature(self, feature_id):
        if feature_id in self.active_features:
            self.active_features.remove(feature_id)

    def add_user(self, user: User):
        self.users.append(user)
        self.user_sids.add(user.sid)

    def remove_user(self, user_id):
        user = self.get_user_by_id(user_id)
        if user is not None:
            if user.sid in self.user_sids:
                self.user_sids.remove(user.sid)
            if user in self.users:
                self.users.remove(user)  # TODO add lock?

    def add_anchor(self, anchor_data):
        print(f"add_anchor {anchor_data}")
        if anchor_data is None:
            return
        print(f"add_anchor2 {anchor_data.anchor_type} ")
        if anchor_data.anchor_type == "CoLocation":
            self.space_sync_anchor = anchor_data
            print(f"add_anchor3 {self.space_sync_anchor.anchor_type} | {anchor_data}")
        elif anchor_data.anchor_type == "RobotHome":
            self.robot_home_anchor = anchor_data
        elif anchor_data.anchor_type == "WorldGraph":
            is_override = False
            for idx, anchor in enumerate(self.world_graph_anchors):
                anchor = self.world_graph_anchors[idx]
                if anchor.name == anchor_data.name:
                    self.world_graph_anchors[idx] = anchor_data
                    is_override = True
                    break
            if not is_override:
                self.world_graph_anchors.append(anchor_data)

    def clear_world_graph_anchors(self):
        self.world_graph_anchors = []

    def reset_room(self):
        self.users = []
        self.user_sids = set()
        self.active_features = []
        # self.main_anchor_id = None  # should we let users reuse spatial anchors?
        # self.dock_anchor_id = None

    def get_user_by_id(self, user_id, logger_additional_info='') -> User or None:
        for user in self.users:
            if user.user_id == user_id:
                return user
        if logger_additional_info:
            logger_additional_info = f' {logger_additional_info}'
        logger.error(f'sid: {request.sid} - {logger_additional_info} could not find user {user_id}')
        return None

    def add_augment(
        self,
        augment_type: local_multiplayer.enums.AugmentType,
        x,
        y,
        z,
        rx,
        ry,
        rz,
        scale
    ):
        augment = local_multiplayer.augment.Augment(
            augment_type=augment_type,
            x=x,
            y=y,
            z=z,
            rx=rx,
            ry=ry,
            rz=rz,
            scale=scale
        )
        self.augments[augment.uuid] = augment
        return augment

    def delete_augment(self, uuid):
        self.augments.pop(uuid)

    def update_augment(
        self,
        uid,
        x,
        y,
        z,
        rx,
        ry,
        rz,
        scale
    ):
        augment = self.augments[uid]
        augment.x = x
        augment.y = y
        augment.z = z
        augment.rx = rx
        augment.ry = ry
        augment.rz = rz
        augment.scale = scale

    def serialize(self):
        serialized_users = []
        for user in self.users:
            serialized_users.append(asdict(user))
        serialized_world_graph_anchors = []
        for anchor in self.world_graph_anchors:
            serialized_world_graph_anchors.append(anchor.serialize())
        return {
            'id': self.id,
            'space_sync_anchor': None if self.space_sync_anchor is None else self.space_sync_anchor.serialize(),
            'robot_home_anchor': None if self.robot_home_anchor is None else self.robot_home_anchor.serialize(),
            'connected_users': serialized_users,
            'active_features': self.active_features,
            'is_robot_feature_available': self.is_robot_feature_available,
            'is_augment_feature_available': self.is_augment_feature_available,
            'world_graph_anchors': serialized_world_graph_anchors
        }
