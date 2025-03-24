# Meta Fair Siro Phase H1 - Speech Unity Client

## Scripts
### SocketIOClientHelper.cs
I wrote this on the previous project as a simple wrapper around the **SocketIOClient** class from SocketIOSharp.
It doesn't really add much, but I found it a bit easier to wrap my head around things with this.

It should be added as a property on a MonoBehaviour or a ScriptableObject, and then you can use it to connect to a server and send messages.

### SpeechClient.cs
This is the main class that handles the connection to the server and the speech recognition.
It uses the `SocketIOClientHelper` to connect to the server and send messages.

Call *SocketIOClientHelper.On("message", callback)* to register a callback for when the server sends a message with the given name.
Call *SocketIOClientHelper.Emit("message", data)* to send a message to the server with the given name and data.

This class is setup to communicate with the server and respond to results.

### SpeechClient.TtsResult
Struct to hold the result of the TextToSpeech request.

### SpeechClient.AsrResult
Struct to hold the result of the SpeechToText request.

### WavUtility.cs
Third party script from https://github.com/deadlyfingers/UnityWav
This script is used to convert the audio data from the microphone into a WAV file that can be sent to the server.

### SpeechToText.cs
Simple script to handle recording from the microphone and sending the audio data to the server through the SpeechClient.

### TextToSpeech.cs
Simple script to handle receiving text from the server and playing it through the speakers.



## SampleScene
### Play Mode
This is a simple example of the classes in use.
Enter text into the input field and hit enter to send it to the server.
Press the ASR button to start recording audio. It will record for a few seconds and then send the audio to the server.

### Edit Mode
#### SpeechClient
If you're running the server locally, you can set the host to localhost and the port to whatever you're running the server on.
Don't change the Policy Port from 843, it's just exposed because it's there.
Same with the Path, leave it as /socket.io.
Caveat:
You might find jumping in and out of play mode, or if there's an error, that the connection will remain open in Edit Mode which means it will keep trying to connect.


#### SpeechToText
You can select your recording device from the dropdown.

#### TextToSpeech
Nothing to set here, but you find the TTS result AudioClip here if you want to have a look at it.

