import singletons
import json
from __main__ import sio, planner_sio, MessageInQueue


# Passthrough from siro to planner
@sio.on('instruction')
def instruction(data=None):
    planner_sio.emit('instruction', data)
    print(f'Relaying Instruction: {data}')
    if data is not None:
        instruction_data = json.loads(data)
        print(f'Instruction sent: {data}')
        relay_message = {
            'event': 'planner',
            'type': 'instruction_received',
            'data': instruction_data.get('data', {'data': None})
        }
        json_message = json.dumps(relay_message)
        print(f'Relaying Instruction: {json_message}')
        message = MessageInQueue(data=json_message, service='planner')
        singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)


@sio.on('get_next_action')
def get_next_action(data=None):
    planner_sio.emit('get_next_action', '')
    print(f'Get next action sent')


@sio.on('get_skill_feedback')
def get_skill_feedback(data=None):
    planner_sio.emit('get_skill_feedback', '')
    print(f'Get skill feedback sent')

@sio.on('cancel')
def cancel(data=None):
    planner_sio.emit('cancel', '')
    print(f'Cancel sent')


# Passthrough from planner to siro
@planner_sio.on('message')
def on_planner_message(message):
    message = MessageInQueue(data=message, service='planner')
    singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)


print('planner.py loaded')
