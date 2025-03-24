using System;
using EngineIOSharp.Common.Enum;
using Newtonsoft.Json.Linq;
using SocketIOSharp.Client;
using SocketIOSharp.Common;
using SocketIOSharp.Common.Packet;
using UnityEngine;

namespace SocketIOClientHelper
{
    [Serializable]
    public class SocketIOClientHelper 
    {
        [Header("SocketIO Settings")] public string host = "localhost";
        public ushort port = 5000;
        public ushort policyPort = 843;
        public string path = "/socket.io";
        public bool reconnection = true;
    
        private SocketIOClient _client;

        public void Initialise()
        {
            Debug.Log("Creating new SocketIOClient...");
            _client = new SocketIOClient(new SocketIOClientOption(EngineIOScheme.http, host, port, policyPort, path, reconnection));
            Debug.Log("SocketIOClient created.");
        }
    
    
        [ContextMenu("Connect to Server")]
        public void ConnectToServer()
        {
            if (_client == null)
            {
                Debug.LogError("SocketIOClient not initialised. Please call Initialise() first.");
                return;
            }
        
            Debug.Log($@"Attempting to connect to SocketIO server at {host}:{port}...");

        
            _client.On(SocketIOEvent.CONNECTION, OnConnect);
            _client.On(SocketIOEvent.DISCONNECT, OnDisconnect);
            _client.On(SocketIOEvent.ERROR, OnError);
            // _client.On("tts_result", (JToken[] data) =>
            // {
            //     Debug.Log($"TTS Result: {data}");
            // });
            Debug.Log("Connecting to server...");
            _client.Connect();
            Debug.Log($"Connection status: {_client.ReadyState}");

        }
    
        public void On(string eventName, Action action)
        {
            _client.On(eventName, action);
        }
    
        public void On(JToken eventName, Action action)
        {
            _client.On(eventName, action);
        }
    
        public void On(JToken eventName, Action<JToken[]> action)
        {
            _client.On(eventName, action);
        }
    
        public void On(JToken eventName, Action<SocketIOAckEvent> action)
        {
            _client.On(eventName, action);
        }
    
        public void Emit(string eventName, JToken message, Action<JToken[]> callback = null)
        {
            _client.Emit(eventName, message, callback);
            Debug.Log($"Emitting {eventName} with message: {message}");
        }
    
        private void OnConnect()
        {
            Debug.Log("Connected to server.");
        }

        private void OnDisconnect()
        {
            Debug.Log(
                $"Disconnected from server. {_client.Option.Host}:{_client.Option.Port} {_client.ReadyState}, {_client.Option.Reconnection}");
        }

        private void OnError()
        {
            Debug.LogError("Error occurred.");
        }

        private void OnDestroy()
        {
            _client?.Close();
        }

    }
}
