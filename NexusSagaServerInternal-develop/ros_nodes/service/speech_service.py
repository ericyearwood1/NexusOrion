from datetime import datetime
import json
import logging
import time
import base64
from typing import Union

import base.heartbeat
import base.service

from speech.data_provider import SpeechDataProvider
from speech.dto import SpeechToTextResultData, SpeechToTextRequestData, TextToSpeechRequestData, TextToSpeechResultData
from speech.seamless import SeamlessMT

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)


class SeamlessSpeechDataProvider(SpeechDataProvider):

    def tick(self, delta: float):
        if self.seamless is None:
            self.seamless = SeamlessMT()
            self.seamless.init()
        # consume the queue
        while not self._queue.empty():
            action, data = self._queue.get()
            if action == 'asr':
                self.do_asr(data)
            elif action == 'tts':
                self.do_tts(data)

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')

    def __init__(self, service: base.service.Service):
        super().__init__(service)
        self.seamless = None

    def do_asr(self, data: SpeechToTextRequestData):
        stt_result = self.seamless.stt(data.audio, data.source_language, data.target_language)
        speech_to_text_result_data = SpeechToTextResultData(
            user_id=data.user_id,
            id=data.id,
            timestamp=datetime.now().isoformat(),
            text=stt_result,
            source_language=data.source_language,
            target_language=data.target_language
        )
        logger.debug(f'ASR result: {speech_to_text_result_data}')

        self.send_asr_result(speech_to_text_result_data)
        return speech_to_text_result_data

    def do_tts(self, data: TextToSpeechRequestData):
        # print(f'TTS data: {data}')
        audio = self.seamless.tts(data.text, data.source_language, data.target_language)

        text_to_speech_result_data = TextToSpeechResultData(
            user_id=data.user_id,
            id=data.id,
            timestamp=datetime.now().isoformat(),
            audio=audio,
            text=data.text,
            source_language=data.source_language,
            target_language=data.target_language
        )
        logger.debug(f'TTS result: {text_to_speech_result_data}')

        self.send_tts_result(text_to_speech_result_data)
        return text_to_speech_result_data

    def asr(self, data=None) -> Union[None, SpeechToTextResultData]:
        # logger.debug(f'ASR data: {data}')
        stt_request_data = SpeechToTextRequestData.from_json(data.get('data', {}))
        logger.debug(f'ASR request data: {stt_request_data}')
        self._queue.put(('asr', stt_request_data))
        return None

    def tts(self, data=None) -> Union[None, TextToSpeechResultData]:

        tts_request_data = TextToSpeechRequestData.from_json(data.get('data', {}))
        logger.debug(f'TTS request data: {tts_request_data}')
        self._queue.put(('tts', tts_request_data))
        return None


class SeamlessSpeechService(base.service.Service):
    host = '0.0.0.0'
    port = 7505
    data_provider_class = SeamlessSpeechDataProvider
    logger = logger


if __name__ == '__main__':
    SeamlessSpeechService().start()
