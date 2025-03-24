import abc
from speech.dto import *

from base.data_provider import DataProvider
from typing import Union

ASR = 'asr'
TTS = 'tts'
ASR_RESULT = 'asr_result'
TTS_RESULT = 'tts_result'

LISTENERS = [ASR, TTS, ASR_RESULT, TTS_RESULT]


class SpeechDataProvider(DataProvider):
    service_name = 'speech'

    @abc.abstractmethod
    def asr(self, data=None) -> Union[None, SpeechToTextResultData]:
        raise NotImplementedError()

    @abc.abstractmethod
    def tts(self, data=None) -> TextToSpeechResultData:
        raise NotImplementedError()

    def register_listeners(self):
        self.on(ASR, self.on_asr)
        self.on(TTS, self.on_tts)

    def on_asr(self, data=None):
        print(f'ASR data: {data}')
        asr_result = self.asr(data)
        if asr_result is None:
            return

        self.send_asr_result(asr_result)

    def on_tts(self, data=None):
        print(f'TTS data: {data}')
        tts_result = self.tts(data)
        if tts_result is None:
            return
        self.send_tts_result(tts_result)

    def send_asr_result(self, data: Union[None,SpeechToTextResultData]):
        print(f'Sending ASR result: {data}')
        if data is None:
            return
        self.reply(json={
            'event': self.service_name,
            'type': 'asr_result',
            'data': data.to_json()
        })

    def send_tts_result(self, data: TextToSpeechResultData):
        print(f'Sending TTS result: {data}')
        if data is None:
            return

        self.reply(json={
            'event': self.service_name,
            'type': 'tts_result',
            'data': data.to_json()
        })

