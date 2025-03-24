import singletons
from __main__ import sio, skills_sio, MessageInQueue


# Passthrough from siro to skills
@sio.on('get_ee_pose')
def get_ee_pose(data=None):
    skills_sio.sleep(1)
    skills_sio.emit('get_ee_pose', data)
    print(f'get_ee_pose sent: {data}')


@sio.on('get_robot_pose')
def get_robot_pose(data=None):
    skills_sio.emit('get_robot_pose', data)
    print(f'get_robot_pose sent: {data}')


# Passthrough from skills to siro

@skills_sio.on('message')
def on_skills_message(message):
    message = MessageInQueue(data=message, service='skills')
    singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)


print('skills.py loaded')