import json
import logging
import multiprocessing
import sys
from dataclasses import asdict, dataclass

from flask import Flask, request, render_template
from flask_socketio import SocketIO, leave_room
import socketio as socketio_client

import config
import service_discovery.server
import singletons
import local_multiplayer

UNITY_EVENT_TYPE = 'message'

logger = logging.getLogger('siro_ws_server')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)

app = Flask(__name__, template_folder='templates', static_folder='static')
app.config['SECRET_KEY'] = 'secret!'
sio = SocketIO(app)
app.logger.setLevel(logging.DEBUG)


try:
    with open('config/suppressed_logs.json') as f:
        suppressed_logs = json.load(f)
except FileNotFoundError:
    suppressed_logs = []


try:
    with open('config/feature_config.json') as f:
        feature_config = json.load(f)
except FileNotFoundError:
    feature_config = {}


def is_suppressed(event, type) -> bool:
    suppressed_types = suppressed_logs.get(event, [])
    return type in suppressed_types

def parse_message(message):
    if isinstance(message, str):
        try:
            request_data = json.loads(message)
        except Exception as e:
            logger.warning(e)
            send_bad_request_response()
            return
    else:
        request_data = message
    return request_data


def send_response(event, data=None, to=None):
    sio.emit(
        event=event,
        data=json.dumps(data),
        to=to
    )


def send_ok_response(code, details=None, data=None, event='ack', to=None):
    sio.emit(
        event=event,
        data=json.dumps({
            'status': 'ok',
            'code': code,
            'details': details,
            'data': data
        }),
        to=to
    )


def send_error_response(code, details, event='ack', to=None):
    sio.emit(
        event=event,
        data=json.dumps({
            'status': 'error',
            'code': code,
            'details': details
        }),
        to=to
    )


def send_bad_request_response():
    send_error_response(
        code='bad_request',
        details='Check request data and consult to documentation.'
    )


def send_ack_response(data, code=None):
    send_ok_response(
        event='ack',
        code=code,
        details=None,
        data=data
    )


def broadcast_to_all_connected_clients(func, *args, **kwargs):
    for sid in singletons.STATE["CONNECTED_CLIENTS"]:
        kwargs['to'] = sid
        func(
            *args,
            **kwargs
        )


@dataclass
class MessageInQueue:
    data: str  # a JSON string, forwarded directly from ROS nodes
    service: str


@app.route('/')
def index():
    return render_template('index.html')


@sio.on('connect')
def on_connect():
    logger.debug(f'sid: {request.sid} - Client connected')
    singletons.STATE['CONNECTED_CLIENTS'].add(request.sid)
    for service_name, status in singletons.STATE['ROS_NODES_STATUS'].items():
        send_ok_response(
            event='service_status',
            code=f'{service_name}_{status}'.lower(),
            to=request.sid
        )
    room_keys = list(local_multiplayer.state.rooms.keys())
    on_connect_message = json.dumps({
        'event': local_multiplayer.consts.MESSAGE_TYPE,
        'type': local_multiplayer.consts.INITIAL_STATE,
        'sid': request.sid,
        'rooms': room_keys,
        'config': feature_config
    })
    sio.emit(
        event=UNITY_EVENT_TYPE,
        data=on_connect_message,
        to=request.sid
    )


@sio.on('disconnect')
def on_disconnect():
    sid = request.sid
    logger.debug(f'sid: {sid} - Client disconnected')
    singletons.STATE['CONNECTED_CLIENTS'].remove(sid)

    user = local_multiplayer.state.find_user_with_sid(sid)
    logger.debug(f'sid: {sid} - found user to remove from room: {user}')

    if user is not None:
        room_id = user.room
        leave_room(room_id)
        user_id = user.user_id
        room = local_multiplayer.state.rooms.get(room_id)
        if room is not None:
            room.remove_user(user_id)
            if len(room.users) == 0:
                logger.debug(f'sid: {sid} - all users have left. resetting room: {room_id}')
                room.reset_room()
        else:
            logger.debug(f'sid: {sid} - could not remove user from room/user not in the room: {user_id} | {room_id}')

        # user_left_message = json.dumps({
        #     'event': local_multiplayer.consts.MESSAGE_TYPE,
        #     'type': local_multiplayer.consts.USER_LEFT,
        #     'user': asdict(user)
        # })
        relay_json = {
            'event': local_multiplayer.consts.MESSAGE_TYPE,
            'type': local_multiplayer.consts.USER_LEFT,
            'data': {
                'user_id': user_id
            }
        }
        logger.debug(f'sid: {sid} - user_left_message: {room_id} | {relay_json}')
        message = MessageInQueue(data=json.dumps(relay_json), service='multiplayer')
        singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)
        # sio.emit(
        #     event=UNITY_EVENT_TYPE,
        #     data=user_left_message,
        #     to=room_id,
        #     skip_sid=request.sid
        # )
        # sio.emit(
        #     event=UNITY_EVENT_TYPE,
        #     data=relay_json
        # )


@sio.on('subscribe')
def on_subscribe(message):
    data = parse_message(message)
    service = data['service']
    singletons.ROS_NODES_SUBSCRIPTIONS[service].add(request.sid)
    sio.emit(
        event=UNITY_EVENT_TYPE,
        data=json.dumps({
            'event': 'subscription',
            'type': f'successfully_subscribed_to_{service}',
        }),
        to=request.sid
    )


@sio.on('unsubscribe')
def on_unsubscribe(message):
    data = parse_message(message)
    service = data['service']
    singletons.ROS_NODES_SUBSCRIPTIONS[service].remove(request.sid)
    sio.emit(
        event=UNITY_EVENT_TYPE,
        data=json.dumps({
            'event': 'subscription',
            'type': f'successfully_unsubscribed_to_{service}',
        }),
        to=request.sid
    )


def instantiate_ros_node_connection(
        name: str,
        host: str,
        _port: int,
        ssl: bool,
        _reconnection=True
):
    sio = socketio_client.Client(reconnection=_reconnection)
    schema = 'wss' if ssl else 'ws'
    sio.connection_url = f'{schema}://{host}:{_port}'
    try:
        sio.connect(sio.connection_url)
    except Exception as e:
        sio._initially_connected = False
        if _reconnection is False:
            logger.error(
                f'Failed to connect to {name}. Reconnection flag is set to False, not retrying.'
            )
            return sio
        logger.error(e)
        logger.error(
            f'Failed to initially connect to {name}, retrying in the healthchecker...'
        )
        return sio
    else:
        sio._initially_connected = True
        singletons.STATE['ROS_NODES_STATUS'][name.upper()] = 'available'
        logger.debug(f'Connected to {name} service.')
        return sio


def ros_node_healthchecker(
        name,
        ros_node_sio
):
    """
    Reconnection is actually handled by the sio.client,
    we want just use logger here.
    """

    # if the client did not managed to initially connect, we need to do
    # the initial connect until it's successful, because reconnection
    # flag (the one in the socketio.Client()) only works after the initial
    # connect is successful
    if ros_node_sio._initially_connected is False:
        while True:
            try:
                ros_node_sio.connect(ros_node_sio.connection_url)
            except Exception as e:
                logger.debug(f'healthchecker: {name} - reconnecting...')
            else:
                logger.debug(f'healthchecker: {name} - reconnect successful')
                break
            sio.sleep(2)

    while True:
        # =====================================================================
        sio.sleep(2)
        if ros_node_sio.connected is False:
            logger.debug(f'healthchecker: {name} - reconnecting...')

        if ros_node_sio.connected is True and ros_node_sio.connected is False:
            singletons.STATE['ROS_NODES_STATUS'][name.upper()] = 'unavailable'
            logger.debug(
                f'healthchecker: {name} - is unavailable'
            )
            broadcast_to_all_connected_clients(
                send_error_response,
                event=name,
                code=f'{name}_unavailable',
                details=f'{name} is unavailable. Trying to reconnect.'
            )
        elif ros_node_sio.connected is False and ros_node_sio.connected is True:
            logger.debug(f'healthchecker: {name} - reconnect successful')
            broadcast_to_all_connected_clients(
                send_error_response,
                event=name,
                code=f'{name}_available',
                details=f'{name} is available.'
            )
            singletons.STATE['ROS_NODES_STATUS'][name.upper()] = 'available'


def messages_broadcaster():
    while True:
        while singletons.MESSAGES_TO_BROADCAST_QUEUE.empty() is False:
            message = singletons.MESSAGES_TO_BROADCAST_QUEUE.get()
            if message.data is None:
                logger.debug(f"Attempt to broadcast null message: from service={message.service} {message.data}")
                continue
            if message.service != "multiplayer" and not is_suppressed(message.service, json.loads(message.data).get('type')):
                logger.debug(f"BROADCASTING: from service={message.service} {message.data}")
            if args.enable_ros_nodes_subscriptions:
                sids = singletons.ROS_NODES_SUBSCRIPTIONS[message.service]
                for sid in sids:
                    sio.emit(
                        event=UNITY_EVENT_TYPE,
                        data=message.data,
                        sid=sid
                    )
            else:
                # broadcast to all
                sio.emit(
                    event=UNITY_EVENT_TYPE,
                    data=message.data
                )
        # must use server event loop to free thread
        sio.sleep(0.01)


def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description='Start the server')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='The host to bind to')
    parser.add_argument('--port', type=int, default=7500, help='The port to bind to')

    parser.add_argument(
        '--enable-ros-nodes-subscriptions',
        action='store_true',
        help='When enabled, you need to send a subscribe event to specific '
             'ROS node service to start receiving their messages.'
    )

    parser.add_argument('--service-discovery-port', type=int, default=7400)
    parser.add_argument('--disable-service-discovery', action='store_true')

    parser.add_argument(
        '--disable-ros-nodes-reconnection',
        action='store_true',
        help='Will never try to reconnect if ros nodes services are down. '
             'Only useful in debugging mode when we only care about siro_server e.g. '
             'testing Unity.'
    )
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()

    ros_nodes = [{
        'name': 'planner',
        'host': config.ROS_NODE_PLANNER_HOST,
        'port': config.ROS_NODE_PLANNER_PORT,
        'ssl': config.ROS_NODE_PLANNER_USE_SSL
    }, {
        'name': 'world_graph',
        'host': config.ROS_NODE_WORLD_GRAPH_HOST,
        'port': config.ROS_NODE_WORLD_GRAPH_PORT,
        'ssl': config.ROS_NODE_WORLD_GRAPH_USE_SSL
    }, {
        'name': 'human_activity',
        'host': config.ROS_NODE_HUMAN_ACTIVITY_HOST,
        'port': config.ROS_NODE_HUMAN_ACTIVITY_PORT,
        'ssl': config.ROS_NODE_HUMAN_ACTIVITY_USE_SSL
    }, {
        'name': 'skills',
        'host': config.ROS_NODE_SKILLS_HOST,
        'port': config.ROS_NODE_SKILLS_PORT,
        'ssl': config.ROS_NODE_SKILLS_USE_SSL
    }, {
        'name': 'speech',
        'host': config.ROS_NODE_SPEECH_HOST,
        'port': config.ROS_NODE_SPEECH_PORT,
        'ssl': config.ROS_NODE_SPEECH_USE_SSL
    }]

    reconnection = not args.disable_ros_nodes_reconnection

    for ros_node in ros_nodes:
        attribute_name = ros_node['name'] + '_sio'
        setattr(
            sys.modules[__name__],
            attribute_name,
            instantiate_ros_node_connection(
                name=ros_node['name'],
                host=ros_node['host'],
                _port=ros_node['port'],
                ssl=ros_node['ssl'],
                _reconnection=reconnection
            )  # even though the service is offline, we can still use it to register events
        )
        if reconnection:
            sio.start_background_task(
                ros_node_healthchecker,
                name=ros_node['name'],
                ros_node_sio=getattr(
                    sys.modules[__name__],
                    attribute_name
                )
            )

    sio.start_background_task(messages_broadcaster)

    # import the handlers, do not remove
    import handlers.human_activity
    import handlers.planner
    import handlers.skills
    import handlers.world_graph
    import handlers.speech
    import handlers.multiplayer_state
    import handlers.info

    if args.disable_service_discovery is False:
        # Important, as of right now, service discovery only works
        # in native Python, doesn't work in Docker and WSL
        multiprocessing.Process(
            target=service_discovery.server.main,
            args=(args.service_discovery_port,)
        ).start()

    logger.info(f'Running socket.io server on ws://{args.host}:{args.port}')
    sio.run(
        app,
        host=args.host,
        port=args.port
    )
