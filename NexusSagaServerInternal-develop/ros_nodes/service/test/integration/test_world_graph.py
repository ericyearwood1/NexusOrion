import json

import socketio

sio = socketio.Client()


GET_WORLD_GRAPH = 'get_world_graph'
GET_WORLD_GRAPH_EDITS = 'get_world_graph_edits'
GET_WORLD_GRAPH_OBJECT_BY_TYPE = 'get_world_graph_object_by_type'
GET_WORLD_GRAPH_OBJECT_BY_NAME = 'get_world_graph_object_by_name'


@sio.event
def connect():
    print('connection established')


@sio.event
def disconnect():
    print('disconnected from server')


@sio.event
def message(data):
    data = json.loads(data)
    if data.get('event') != 'world_graph':
        return
    if data.get('type') == 'world_graph':
        print(f'World graph: {data}')
    if data.get('type') == 'world_graph_objects':
        print(f'World graph objects: {data}')





def main():
    print('Starting test_world_graph')
    for _port in [7503, 7500]:
        print(f'Connecting to server on port {_port}')
        sio.connect(f'http://localhost:{_port}')
        print('Connected to server')

        sio.emit(GET_WORLD_GRAPH, '')
        print(f'Sent {GET_WORLD_GRAPH}')
        sio.sleep(5)
        sio.emit(GET_WORLD_GRAPH_EDITS, {'timestamp': 0})
        print(f'Sent {GET_WORLD_GRAPH_EDITS}')
        sio.sleep(5)
        sio.emit(GET_WORLD_GRAPH_OBJECT_BY_TYPE, {'data': {'object_type': 'test'}})
        print(f'Sent {GET_WORLD_GRAPH_OBJECT_BY_TYPE} test')
        sio.sleep(5)
        sio.emit(GET_WORLD_GRAPH_OBJECT_BY_TYPE, {'data': {'object_type': 'failure'}})
        print(f'Sent {GET_WORLD_GRAPH_OBJECT_BY_TYPE} failure')
        sio.sleep(5)
        sio.emit(GET_WORLD_GRAPH_OBJECT_BY_NAME, {'data': {'object_name': 'test'}})
        print(f'Sent {GET_WORLD_GRAPH_OBJECT_BY_NAME} test')
        sio.sleep(5)
        sio.emit(GET_WORLD_GRAPH_OBJECT_BY_NAME, {'data': {'object_name': 'failure'}})
        print(f'Sent {GET_WORLD_GRAPH_OBJECT_BY_NAME} failure')
        sio.sleep(5)
        sio.emit(GET_WORLD_GRAPH_OBJECT_BY_NAME, {'data': {'object_name': 'success_without_receptacle'}})
        print(f'Sent {GET_WORLD_GRAPH_OBJECT_BY_NAME} success_without_receptacle')
        sio.sleep(5)

        sio.disconnect()
        print('Disconnected from server')


if __name__ == '__main__':
    main()



