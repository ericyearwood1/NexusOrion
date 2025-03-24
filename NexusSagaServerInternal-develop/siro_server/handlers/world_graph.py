import singletons
from __main__ import sio, world_graph_sio, MessageInQueue


# Passthrough from siro to world_graph
@sio.on('get_world_graph')
def world_graph(data=None):
    world_graph_sio.emit('get_world_graph', data)
    print(f'world_graph.py: get_world_graph: {data}')


@sio.on('get_world_graph_edits')
def world_graph_edits(data=None):
    world_graph_sio.emit('get_world_graph_edits', data)
    print(f'world_graph.py: get_world_graph_edits: {data}')


@sio.on('get_world_graph_object_by_type')
def world_graph_object_by_type(data=None):
    world_graph_sio.emit('get_world_graph_object_by_type', data)
    print(f'world_graph.py: get_world_graph_object_by_type: {data}')


@sio.on('get_world_graph_object_by_name')
def world_graph_object_by_name(data=None):
    world_graph_sio.emit('get_world_graph_object_by_name', data)
    print(f'world_graph.py: get_world_graph_object_by_name: {data}')

# Passthrough from world_graph to siro
@world_graph_sio.on('message')
def on_world_graph_message(message):
    message = MessageInQueue(data=message, service='world_graph')
    singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)
    print(f'world_graph.py: world_graph: {message}')



print('world_graph.py loaded')