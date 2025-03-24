import json

import socketio

# from skills.service import GET_EE_POSE, GET_ROBOT_POSE
# from skills.data_provider import EE_POSE, ROBOT_POSE


GET_EE_POSE = 'get_ee_pose'
GET_ROBOT_POSE = 'get_robot_pose'

sio = socketio.Client()


@sio.event
def connect():
    print('connection established')


@sio.event
def disconnect():
    print('disconnected from server')


@sio.on('message')
def on_message(data):
    data = json.loads(data)
    if data.get('event') != 'skills':
        return
    if data.get('type') == 'ee_pose':
        print(f'EE pose: {data}')

    if data.get('type') == 'robot_pose':
        print(f'Robot pose: {data}')


def main():
    print('Starting test_skills')
    for _port in [7504, 7500]:
        print(f'Connecting to server on port {_port}')
        sio.connect(f'http://localhost:{_port}')
        print('Connected to server')

        sio.emit(GET_EE_POSE)
        print(f'Sent {GET_EE_POSE}')
        sio.sleep(5)
        sio.emit(GET_ROBOT_POSE)
        print(f'Sent {GET_ROBOT_POSE}')
        sio.sleep(5)

        sio.disconnect()
        print('Disconnected from server')


if __name__ == '__main__':
    main()
