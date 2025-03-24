import base64
from dataclasses import dataclass
from datetime import datetime


@dataclass
class TextToSpeechResultData:
    user_id: str
    id: str
    timestamp: str
    audio: bytes
    text: str
    source_language: str
    target_language: str

    def to_json(self):
        return {
            'user_id': self.user_id,
            'id': self.id,
            'timestamp': self.timestamp,
            'audio': base64.b64encode(self.audio).decode('utf-8'),
            'text': self.text,
            'source_language': self.source_language,
            'target_language': self.target_language
        }

    @staticmethod
    def from_json(json):
        return TextToSpeechResultData(
            user_id=json.get('user_id', None),
            id=json.get('id', None),
            timestamp=json.get('timestamp', datetime.now().isoformat()),
            audio=base64.b64decode(json.get('audio', "")),
            text=json.get('text', ""),
            source_language=json.get('source_language', 'eng'),
            target_language=json.get('target_language', 'eng')
        )

    def __str__(self):
        return f'{self.id} [{self.timestamp}] ({self.source_language} -> {self.target_language}) {len(self.audio)} bytes'

    def __repr__(self):
        return self.__str__()


@dataclass
class TextToSpeechRequestData:
    user_id: str
    id: str
    timestamp: str
    text: str
    source_language: str
    target_language: str

    def to_json(self):
        return {
            'user_id': self.user_id,
            'id': self.id,
            'timestamp': self.timestamp,
            'text': self.text,
            'source_language': self.source_language,
            'target_language': self.target_language
        }

    @staticmethod
    def from_json(json):
        return TextToSpeechRequestData(
            user_id=json.get('user_id', None),
            id=json.get('id', None),
            timestamp=json.get('timestamp', datetime.now().isoformat()),
            text=json.get('text', ""),
            source_language=json.get('source_language', 'eng'),
            target_language=json.get('target_language', 'eng')
        )

    def __str__(self):
        return f'{self.id} [{self.timestamp}] ({self.source_language} -> {self.target_language}) "{self.text}"'

    def __repr__(self):
        return self.__str__()


@dataclass
class SpeechToTextResultData:
    user_id: str
    id: str
    timestamp: str
    text: str
    source_language: str
    target_language: str

    def to_json(self):
        return {
            'user_id': self.user_id,
            'id': self.id,
            'timestamp': self.timestamp,
            'text': self.text,
            'source_language': self.source_language,
            'target_language': self.target_language
        }

    @staticmethod
    def from_json(json):
        return SpeechToTextResultData(
            user_id=json.get('user_id', None),
            id=json.get('id', None),
            timestamp=json.get('timestamp', datetime.now().isoformat()),
            text=json.get('text', ""),
            source_language=json.get('source_language', 'eng'),
            target_language=json.get('target_language', 'eng')
        )

    def __str__(self):
        return f'{self.id} [{self.timestamp}] ({self.source_language} -> {self.target_language}) {self.text}'

    def __repr__(self):
        return self.__str__()


@dataclass
class SpeechToTextRequestData:
    user_id: str
    id: str
    timestamp: str
    audio: bytes
    source_language: str
    target_language: str

    def to_json(self):
        return {
            'user_id': self.user_id,
            'id': self.id,
            'timestamp': self.timestamp,
            'audio': base64.b64encode(self.audio).decode('utf-8'),
            'source_language': self.source_language,
            'target_language': self.target_language
        }

    @staticmethod
    def from_json(json):
        return SpeechToTextRequestData(
            user_id=json.get('user_id', None),
            id=json.get('id', None),
            timestamp=json.get('timestamp', datetime.now().isoformat()),
            audio=base64.b64decode(json.get('audio', "")),
            source_language=json.get('source_language', 'eng'),
            target_language=json.get('target_language', 'eng')
        )

    def __str__(self):
        return f'{self.id} [{self.timestamp}] ({self.source_language} -> {self.target_language}) {len(self.audio)} bytes'

    def __repr__(self):
        return self.__str__()

