import singletons

from __main__ import sio, human_activity_sio, MessageInQueue


# Passthrough from siro to human_activity
@sio.on('get_human_activity')
def get_human_activity(message=None):
    human_activity_sio.emit('get_human_activity', message)


# Passthrough from human_activity to siro
@human_activity_sio.on('message')
def on_human_activity_message(message):
    message = MessageInQueue(data=message, service='human_activity')
    singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)


print('human_activity.py loaded')
