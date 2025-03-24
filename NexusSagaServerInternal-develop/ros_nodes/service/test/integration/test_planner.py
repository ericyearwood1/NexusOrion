import json

import socketio

INSTRUCTION = 'instruction'
GET_NEXT_ACTION = 'get_next_action'
GET_SKILL_FEEDBACK = 'get_skill_feedback'



# instruction
# get_next_action
# get_skill_feedback

# send an instruction message
# listen for next action messages
# listen for skill feedback messages
# ask for the next action
# ask for the skill feedback

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
    if data.get('event') != 'planner':
        return
    if data.get('type') == 'next_action':
        print(f'Next action: {data.get("action")}')

    if data.get('type') == 'action_feedback':
        print(f'Action feedback: {data.get("action")}: {data.get("feedback")}')

    if data.get('type') == 'instruction_not_understood':
        print(f'Instruction not understood: {data.get("instruction")}')

    if data.get('type') == 'instruction_complete':
        print(f'Instruction complete')




def main():
    print('Starting test_planner')
    for _port in [7501, 7500]:
        print(f'Connecting to server on port {_port}')
        sio.connect(f'http://localhost:{_port}')
        print('Connected to server')
        sio.emit(INSTRUCTION, {'data': {'data': 'test_bad'}})
        print(f'Sent {INSTRUCTION} test_bad')
        sio.sleep(5)
        sio.emit(GET_NEXT_ACTION)
        print(f'Sent {GET_NEXT_ACTION}')
        sio.sleep(5)
        sio.emit(GET_SKILL_FEEDBACK)
        print(f'Sent {GET_SKILL_FEEDBACK}')
        sio.sleep(5)
        sio.emit(INSTRUCTION)
        print(f'Sent {INSTRUCTION}')
        sio.sleep(30)
        sio.emit(INSTRUCTION)
        print(f'Sent {INSTRUCTION}')
        sio.sleep(5)

        sio.disconnect()



if __name__ == '__main__':
    main()
