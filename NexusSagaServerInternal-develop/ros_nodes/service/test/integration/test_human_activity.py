import json

import socketio

GET_HUMAN_ACTIVITY = 'get_human_activity'

sio = socketio.Client()


@sio.event
def connect():
    print('connection established')


@sio.event
def disconnect():
    print('disconnected from server')


@sio.event
def message(data):
    data = json.loads(data)
    if data.get('event') != 'human_activity':
        return
    if data.get('type') == 'human_activity':
        print(f'Human activity: {data}')


def main():
    print('Starting test_human_activity')
    for _port in [7502, 7500]:
        print(f'Connecting to server on port {_port}')
        sio.connect(f'http://localhost:{_port}')
        print('Connected to server')
        sio.emit(event=GET_HUMAN_ACTIVITY, data={})
        print(f'Sent {GET_HUMAN_ACTIVITY} test')
        sio.sleep(5)
        sio.emit(event=GET_HUMAN_ACTIVITY, data={})
        print(f'Sent {GET_HUMAN_ACTIVITY} test')
        sio.emit(event=GET_HUMAN_ACTIVITY, data={})
        print(f'Sent {GET_HUMAN_ACTIVITY} test')
        sio.sleep(5)

        sio.disconnect()
        print('Disconnected from server')


if __name__ == '__main__':
    main()
