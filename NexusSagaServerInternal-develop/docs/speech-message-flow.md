# Expected Speech Message Flow

## Speech-to-Text
**Request**
> {"event": "speech", "type": "asr", "data": {
            'user_id': "3565364",
            'id': 123456-1234-1234-1234-123456789012,
            'timestamp': 1234567890,
            'audio': 'base64 encoded audio, WAV PCM 16kHz mono',
            'source_language': 'eng'
            'target_language': 'eng'
        }}

**Response**
> {"event": "speech", "type": "asr_result", "data": {
            'user_id': "3565364",
            'id': 123456-1234-1234-1234-123456789012,
            'timestamp': 1234567890,
            'audio': 'base64 encoded audio, WAV PCM 16kHz mono',
            'text': 'Hello, how are you?',
            'source_language': 'eng'
            'target_language': 'eng'
        }}
 
## Text-to-Speech
**Request**
> {"event": "speech", "type": "tts", "data": {
            'user_id': "3565364",
            'id': 123456-1234-1234-1234-123456789012,
            'timestamp': 1234567890,
            'text': 'Hello, how are you?',
            'source_language': 'eng'
            'target_language': 'eng'
        }}

**Response**
> {"event": "speech", "type": "tts_result", "data": {
            'user_id': "3565364",
            'id': 123456-1234-1234-1234-123456789012,
            'timestamp': 1234567890,
            'audio': 'base64 encoded audio, WAV PCM 16kHz mono',
            'source_language': 'eng'
            'target_language': 'eng'
        }}




