const socket = io();
console.log("Connected to server at: " + socket.io.uri);

document.getElementById('ttsButton').addEventListener('click', function () {
    console.log('Button clicked');  // Log to verify button click
    const text = document.getElementById('textInput').value;
    const language = document.getElementById('languageTTS').value;
    const stream = document.getElementById('streamTTS').checked;

    console.log('Preparing to emit tts event:', text, language);  // Log to verify data before emission
    socket.emit('tts', {
        service: 'speech',
        type: 'tts',
        data: {
            id: 'tts',
            text: text,
            source_language: language,
            target_language: language,
            },
    });
});




// Speech-to-Text button click handler
document.getElementById('asrButton').addEventListener('click', function () {
    const audioFile = document.getElementById('audioInput').files[0];
    const language = document.getElementById('languageASR').value;

    console.log('asrButton clicked: audioInput:', audioFile, 'language:', language); // Debugging log

    // Ensure audio file and required fields are provided
    if (!audioFile ||  !language) {
        console.error('Audio file, or language is missing for ASR.');
        document.getElementById('asrStatus').textContent = 'Error: Audio file, or language is missing.';
        return;
    }

    const logElement = document.getElementById('asrLog');
    logElement.textContent = 'Audio Speech Recognition: ' + audioFile.name;
    // Read the audio file and emit the 'asr' event to the server
    const reader = new FileReader();
    reader.onload = function (event) {
        const arrayBuffer = event.target.result;
        const base64Audio = btoa(String.fromCharCode.apply(null, new Uint8Array(arrayBuffer)));
        socket.emit('asr', {
            id: 'asr',
            service: 'speech',
            type: 'asr',
            data: {
                audio: base64Audio,
                source_language: language,
                target_language: language,
            },
        });
    };

    reader.readAsArrayBuffer(audioFile);
});

socket.on('message', function (data) {
    console.log('Received message:', data);  // Log to verify data received from server
    data = JSON.parse(data);
    var event = data.event;
    var type = data.type;

    console.log('event:', event);

    if (event != 'speech') {
        console.log('event:', event);
        return;
    }

    if (type == "asr_result") {
        handleAsrResult(data.id, data.text, data.timestamp, data.source_language, data.target_language);
    }else if (type == "tts_result") {
        handleTtsResult(data.id, data.audio, data.timestamp, data.source_language, data.target_language);
    }


});

// asr handler function
function handleAsrResult(id, text, timestamp, source_language, target_language) {
    console.log('handleAsrResult:', id, text, timestamp, source_language, target_language);
    const statusElement = document.getElementById('asrStatus');
    const transcriptionElement = document.getElementById('transcriptionResult');

    statusElement.textContent = 'Success';
    transcriptionElement.textContent = 'Transcription: ' + text;
}

// tts handler function
function handleTtsResult(id, audio, timestamp, source_language, target_language) {
    console.log('handleTtsResult:', id, audio, timestamp, source_language, target_language);
    const statusElement = document.getElementById('ttsStatus');
    if (audio == null) {
        statusElement.textContent = 'Error: No audio data';
        return;
    }

    if (id == null) {
        id = 'tts';
    }
    statusElement.textContent = 'Success';
    // Decode the base64 string
    const binaryString = atob(audio);

    // Convert the binary string to an array of 8-bit unsigned integers
    const len = binaryString.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);

    }

    // Create a Blob from the bytes array
    const blob = new Blob([bytes], { type: 'audio/wav' });
    const url = URL.createObjectURL(blob);

    // Create a link element to trigger the download
    const downloadLink = document.createElement('a');
    downloadLink.href = url;
    downloadLink.download = id + '.wav';

    document.body.appendChild(downloadLink);
    downloadLink.click();

    document.body.removeChild(downloadLink);  // Clean up the DOM

    URL.revokeObjectURL(url);  // Clean up the URL object


}

// Listener for the 'asr_result' event from the server
//socket.on('asr_result', function (data) {
//    console.log('Received asr_result:', data);  // Log to verify data received from server
//    const statusElement = document.getElementById('asrStatus');
//    const transcriptionElement = document.getElementById('transcriptionResult');
//
//    if (data.success) {
//        statusElement.textContent = data.message;
//        transcriptionElement.textContent = 'Transcription: ' + data.text;
//    } else {
//        statusElement.textContent = 'Error: ' + data.message;
//    }
//});
//
//// Listener for the 'tts_result' event from the server
//socket.on('tts_result', function (data) {
//    console.log('Received tts_result:', data);  // Log to verify data received from server
//    const statusElement = document.getElementById('ttsStatus');
//    const streamLinkElement = document.getElementById('streamLink');
//
//
//    if (data.success) {
//        statusElement.textContent = data.message;
//        // create a safe name from the message
//        const safeName = data.text.replace(/[^a-z0-9]/gi, '_').toLowerCase();
//
//        // we get eaither speech or speech_url
//        if (data.speech_url) {
//            streamLinkElement.href = data.speech_url;
//            streamLinkElement.textContent = 'Stream: ' + safeName + '.wav';
//            streamLinkElement.style.display = 'block';
//            return;
//        } else {
//            streamLinkElement.style.display = 'none';
//        }
//
//        // Decode the base64 string
//        const binaryString = atob(data.audio);
//
//        // Convert the binary string to an array of 8-bit unsigned integers
//        const len = binaryString.length;
//        const bytes = new Uint8Array(len);
//        for (let i = 0; i < len; i++) {
//            bytes[i] = binaryString.charCodeAt(i);
//        }
//
//        // Create a Blob from the bytes array
//        const blob = new Blob([bytes], { type: 'audio/wav' });
//        const url = URL.createObjectURL(blob);
//
//        // Create a link element to trigger the download
//        const downloadLink = document.createElement('a');
//        downloadLink.href = url;
//        downloadLink.download = safeName + '.wav';
//        document.body.appendChild(downloadLink);
//        downloadLink.click();
//        document.body.removeChild(downloadLink);  // Clean up the DOM
//        URL.revokeObjectURL(url);  // Clean up the URL object
//
//    } else {
//        statusElement.textContent = 'Error: ' + data.message;
//    }
//});

// Listener for 'connect' event to confirm connection to the server
socket.on('connect', function () {
    console.log('Connected to server');
});

// Listener for 'error' event to handle any errors
socket.on('error', function (error) {
    console.error('Error:', error);
    document.getElementById('ttsStatus').textContent = 'Error: ' + error.error;
    document.getElementById('asrStatus').textContent = 'Error: ' + error.error;
});
