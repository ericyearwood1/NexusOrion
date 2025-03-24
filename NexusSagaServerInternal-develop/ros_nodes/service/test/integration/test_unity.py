import socketio

sio = socketio.Client()

sio.logger.setLevel('DEBUG')


@sio.event
def connect():
    print('connection established')


@sio.event
def disconnect():
    print('disconnected from server')


# Human activity
@sio.on('human_activity')
def human_activity(data=None):
    print('Human activity:', data)


# Planner
@sio.on('next_action')
def next_action(data=None):
    print('Next action:', data)


@sio.on('skill_feedback')
def skill_feedback(data=None):
    print('Skill feedback:', data)


# World graph
@sio.on('world_graph')
def world_graph(data=None):

    print('World graph:', data)


@sio.on('world_graph_objects')
def world_graph_objects(data=None):
    print('World graph objects:', data)


# Skills
@sio.on('ee_pose')
def ee_pose(data=None):
    print('End effector pose:', data)


@sio.on('robot_pose')
def robot_pose(data=None):
    print('Robot pose:', data)


def main():
    print('Starting test')
    _port = 7500
    print(f'Connecting to server on port {_port}')
    sio.connect(f'http://localhost:{_port}')
    print('Connected to server')
    sio.emit('test', '123')
    sio.sleep(5)
    sio.emit('get_ee_pose', 'test')
    print(f'Sent get_ee_pose test')
    sio.sleep(5)
    sio.emit('get_robot_pose', 'test')
    print(f'Sent get_robot_pose test')
    sio.sleep(5)
    sio.emit('get_world_graph', 'test')
    print(f'Sent get_world_graph test')
    sio.sleep(5)
    sio.emit('get_world_graph_edits', {'timestamp': 0})
    print(f'Sent get_world_graph_edits test')
    sio.sleep(5)
    sio.emit('get_world_graph_object_by_type', {'object_type': 'test'})
    print(f'Sent get_world_graph_object_by_type test')
    sio.sleep(5)
    sio.emit('get_world_graph_object_by_name', {'object_name': 'test'})
    print(f'Sent get_world_graph_object_by_name test')
    sio.sleep(5)
    sio.emit('get_human_activity', 'test')
    print(f'Sent get_human_activity test')
    sio.sleep(5)
    sio.emit('instruction', 'test')
    print(f'Sent instruction test')

    # listen for events
    sio.wait()


if __name__ == '__main__':
    main()

