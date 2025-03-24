import json
import logging
import multiprocessing
import sys
from dataclasses import dataclass

import eventlet

import singletons.states
from flask import Flask, request
from flask_socketio import SocketIO
from flask_socketio.namespace import Namespace

logger = logging.getLogger('ws_server')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

received_messages_queue = multiprocessing.Queue()
messages_to_sent_queue = multiprocessing.Queue()


@dataclass
class MessageInQueue:
    event: str
    type: str
    data: dict

    def __str__(self):
        return f'{self.event=} {self.type=} {self.data=}'

    def __repr__(self):
        return self.__str__()


@socketio.on('connect')
def on_connect():
    logger.debug(f'sid: {request.sid} - Siro server connected')
    singletons.states.SIRO_SERVER_SID = request.sid


@socketio.on('disconnect')
def on_disconnect():
    logger.debug(f'sid: {request.sid} - Siro server disconnected')
    singletons.states.SIRO_SERVER_SID = None


def parse_message(message):
    if isinstance(message, str):
        try:
            request_data = json.loads(message)
        except Exception as e:
            logger.error(message)
            logger.exception(e)
            raise
    elif isinstance(message, dict):
        return message
    else:
        raise ValueError(f'Not supported data type {message}')
    return request_data


class HandleAllEventsNamespace(Namespace):
    def trigger_event(self, event, *args):
        """
        All events will be catch all here and added to the queue
        """
        sid = args[0]
        if len(args) > 1:
            message = args[1]
        else:
            message = None
        logger.debug(f'Incoming message {sid=} {event=} {message=}')
        if message:
            try:
                data = parse_message(message)
            except Exception as e:
                logger.error(f'Incoming message must be a JSON or completely empty! dropping {message}')
                return
        else:
            data = {}
        received_messages_queue.put(
            MessageInQueue(
                event=data.get('event', event),
                data=data,  # eg. {"instruction": "test"}
                type=event  # eg. next_action
            )
        )


socketio.server.register_namespace(HandleAllEventsNamespace())
