import singletons
import json
# test quirks, sigh
try:
    from __main__ import parse_message, UNITY_EVENT_TYPE, sio, MessageInQueue
except ImportError:
    from main import parse_message, UNITY_EVENT_TYPE, sio, MessageInQueue

try:
    with open('config/default_instructions.json') as f:
        default_instructions = json.load(f)
except FileNotFoundError:
    default_instructions = []


@sio.on('get_default_instructions')
def get_default_instructions():
    data = {'event': 'siro',
            'type': 'default_instructions',
            'data': default_instructions}
    message = MessageInQueue(service='siro', data=json.dumps(data))

    singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)
