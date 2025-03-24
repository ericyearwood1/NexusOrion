import json
import logging
from dataclasses import asdict
import singletons
from flask import request
from flask_socketio import join_room

import local_multiplayer

# test quirks, sigh
try:
    from __main__ import parse_message, UNITY_EVENT_TYPE, sio, MessageInQueue
except ImportError:
    from main import parse_message, UNITY_EVENT_TYPE, sio, MessageInQueue

from local_multiplayer.user import User
from local_multiplayer.spatial_anchor import SpatialAnchor

logger = logging.getLogger('siro_ws_server')


def on_relay_message(message):
    message = MessageInQueue(data=json.dumps(message), service='multiplayer')
    singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)


def get_room(data, logger_additional_info=''):
    room_id = data['room']
    room = local_multiplayer.state.rooms.get(room_id)
    if room is None:
        if logger_additional_info:
            logger_additional_info = f' {logger_additional_info}'
        logger.error(f'sid: {request.sid} -{logger_additional_info} could not find room: {data}')
        return None
    return room


@sio.on('join_room')
def on_join(message):
    data = parse_message(message)
    room = get_room(data, logger_additional_info='on_join')
    join_room(room.id)
    user = User.deserialize(data['user'])
    room.add_user(user)

    logger.debug(f'sid: {request.sid} - user joined room: {user.user_id}')

    # room_message = {
    #     'event': local_multiplayer.consts.MESSAGE_TYPE,
    #     'type': local_multiplayer.consts.ROOM_STATE,
    #     'data': {
    #         'room': json.dumps(room.serialize())
    #     }
    # }
    room_state_message = json.dumps({
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.ROOM_STATE,
        'room': room.serialize()
    })
    logger.debug(f'sid: {request.sid} - room state message: {room_state_message}')

    sio.emit(
        event=UNITY_EVENT_TYPE,
        data=room_state_message,
        to=request.sid
    )

    relay_json = {
            'event': local_multiplayer.consts.MESSAGE_TYPE,
            'type': local_multiplayer.consts.USER_JOIN,
            'data': {
                'user': data['user']
            }
    }

    # sio.emit(
    #     event=UNITY_EVENT_TYPE,
    #     data=message,
    #     to=room.id,
    #     # why skip self?
    #     skip_sid=request.sid
    # )
    on_relay_message(relay_json)


@sio.on('synced_entity')
def on_synced_data_update(message):
    data = parse_message(message)
    room_id = data['room']
    room = get_room(data, logger_additional_info='on_synced_data_update')
    # sio.emit(UNITY_EVENT_TYPE, message, to=room_id, skip_sid=request.sid)
    relay_json = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.SYNCED_ENTITY,
        'data': {
            'room': room_id,
            'entity_id': data['entity_id'],
            'position': data['position'],
            'rotation': data['rotation'],
            'scale': data['scale']
        }
    }
    on_relay_message(relay_json)


@sio.on('activate_feature')
def on_activate_feature(message):
    logger.debug(f'sid: {request.sid} - on_activate_feature: {message}')
    data = parse_message(message)
    feature_id = data['feature']
    room_id = data['room']
    room = get_room(data, logger_additional_info='on_activate_feature')
    if room is None:
        return
    room.add_active_feature(feature_id)

    relay_json = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.ACTIVATE_FEATURE,
        'data': {
            'room': room_id,
            'feature': feature_id
        }
    }
    # sio.emit(UNITY_EVENT_TYPE, message, to=room_id)
    on_relay_message(relay_json)


@sio.on('deactivate_feature')
def on_deactivate_feature(message):
    logger.debug(f'sid: {request.sid} - on_deactivate_feature: {message}')
    data = parse_message(message)
    feature_id = data['feature']
    room_id = data['room']
    room = get_room(data, logger_additional_info='on_deactivate_feature')
    if room is None:
        return
    room.remove_active_feature(feature_id)
    relay_json = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.DEACTIVATE_FEATURE,
        'data': {
            'room': room_id,
            'feature': feature_id
        }
    }
    # sio.emit(UNITY_EVENT_TYPE, message, to=room_id)
    on_relay_message(relay_json)


@sio.on('spatial_anchor_update')
def on_anchor_update(message):
    data = parse_message(message)
    anchor_data = SpatialAnchor.deserialize(data['anchor_data'])
    room = get_room(data, logger_additional_info='on_anchor_update')
    if room is None:
        return
    room.add_anchor(anchor_data)
    logger.debug(f'sid: {request.sid} - received anchor update: {data} | {room.id} | {message}')
    # sio.emit(UNITY_EVENT_TYPE, message, to=room.id, skip_sid=request.sid)
    relay_json = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.SPATIAL_ANCHOR_UPDATE,
        'data': {
            'room': data["room"],
            'anchor_data': data['anchor_data']
        }
    }
    on_relay_message(relay_json)
    local_multiplayer.state.save()


@sio.on('clear_world_graph_anchors')
def on_clear_world_graph_anchors(message):
    data = parse_message(message)
    room = get_room(data, logger_additional_info='on_clear_world_graph_anchors')
    room.clear_world_graph_anchors()


@sio.on('user_details')
def on_user_edit(message):
    data = parse_message(message)
    room = get_room(data, logger_additional_info='on_user_edit')
    if room is None:
        sio.emit(
            "error",
            json.dumps({
                "event": "error",
                "type": local_multiplayer.consts.USER_DETAILS_UPDATE,
                "message": f"could not find room id {room.id}"
            }),
            to=request.sid
        )
        return
    user_data = data['user']
    user_id = user_data['user_id']
    user = room.get_user_by_id(user_id, logger_additional_info='on_user_edit')
    if user is None:
        data = {
            "event": "error",
            "type": local_multiplayer.consts.USER_DETAILS_UPDATE,
            "message": f"could not find user id {user_id}"
        }
        sio.emit(
            "error",
            json.dumps(data),
            to=request.sid
        )
        return

    user.display_name = user_data['display_name']
    user.color = user_data['color']
    logger.debug(f'sid: {request.sid} - on_user_edit: color {user.color} | display name : {user.display_name}')
    data = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.USER_DETAILS_UPDATE,
        'user': asdict(user)
    }

    # why skip self?
    sio.emit(
        "message",
        json.dumps(data),
        to=room,
        skip_sid=request.sid
    )


@sio.on('create_augment')
def on_create_augment(message):
    data = parse_message(message)
    room = get_room(data, logger_additional_info='on_create_augment')
    if room is None:
        sio.emit(
            "error",
            json.dumps({
                "event": "error",
                "type": local_multiplayer.consts.CREATE_AUGMENT,
                "message": f"could not find room id {room.id}"
            }),
            to=request.sid
        )
        return
    augment_data = data['augment']
    room.add_augment(
        augment_type=augment_data['augment_type'],
        x=augment_data['x'],
        y=augment_data['y'],
        z=augment_data['z'],
        rx=augment_data['rx'],
        ry=augment_data['ry'],
        rz=augment_data['rz'],
        scale=augment_data['scale'],
    )
    data = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.CREATE_AUGMENT,
        'room': room.serialize()
    }
    # skip self?
    sio.emit(
        "message",
        json.dumps(data),
        to=room,
        skip_sid=request.sid
    )


@sio.on('delete_augment')
def on_delete_augment(message):
    data = parse_message(message)
    room = get_room(data, logger_additional_info='on_delete_augment')
    if room is None:
        sio.emit(
            "error",
            json.dumps({
                "event": "error",
                "type": local_multiplayer.consts.DELETE_AUGMENT,
                "message": f"could not find room id {room.id}"
            }),
            to=request.sid
        )
        return
    augment_data = data['augment']
    room.delete_augment(uuid=augment_data['uuid'])
    data = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.DELETE_AUGMENT,
        'room': room.serialize()
    }
    sio.emit(
        "message",
        json.dumps(data),
        to=room,
        skip_sid=request.sid
    )


@sio.on('update_augment')
def on_update_augment(message):
    data = parse_message(message)
    room = get_room(data, logger_additional_info='on_update_augment')
    if room is None:
        sio.emit(
            "error",
            json.dumps({
                "event": "error",
                "type": local_multiplayer.consts.UPDATE_AUGMENT,
                "message": f"could not find room id {room.id}"
            }),
            to=request.sid
        )
        return
    augment_data = data['augment']
    room.update_augment(
        uid=augment_data['uuid'],
        x=augment_data['x'],
        y=augment_data['y'],
        z=augment_data['z'],
        rx=augment_data['rx'],
        ry=augment_data['ry'],
        rz=augment_data['rz'],
        scale=augment_data['scale']
    )
    data = {
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.UPDATE_AUGMENT,
        'room': room.serialize()
    }
    sio.emit(
        "message",
        json.dumps(data),
        to=room,
        skip_sid=request.sid
    )
