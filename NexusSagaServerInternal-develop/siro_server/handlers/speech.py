import singletons
from __main__ import sio, speech_sio, MessageInQueue


@sio.on('stt')
def handle_asr(data=None):
    speech_sio.emit('asr', data)

@sio.on('asr')
def handle_asr(data=None):
    speech_sio.emit('asr', data)


@sio.on('tts')
def handle_tts(data=None):
    speech_sio.emit('tts', data)


@speech_sio.on('message')
def on_speech_message(data):
    message = MessageInQueue(data=data, service='speech')
    singletons.MESSAGES_TO_BROADCAST_QUEUE.put_nowait(message)

@speech_sio.on('asr_result')
def on_asr_result(data):
    print(f'ASR result: {data}')


print('speech.py loaded')
