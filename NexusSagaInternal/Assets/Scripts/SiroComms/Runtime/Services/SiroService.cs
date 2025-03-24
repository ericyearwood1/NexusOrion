using System;
using EngineIOSharp.Common.Enum;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using SiroComms.Runtime.Data;
using SiroComms.Runtime.Messages.Client;
using SocketIOSharp.Client;
using SocketIOSharp.Common;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    
    public class SiroService : BaseSiroService
    {
        private const string MESSAGE = "message";
        
        private bool _receivedSocketClosed;
        private bool _receivedSocketOpen;
        private SocketIOClient _client;
        
        public ushort policyPort = 843;
        public string path = "/socket.io";
        public bool reconnection = true;

        public override void Update()
        {
            base.Update();
            if (_receivedSocketOpen)
            {
                Debug.Log("RobotInitialiseState::received connection open");
                _receivedSocketOpen = false;
                HandleSocketOpened();
            }

            if (_receivedSocketClosed)
            {
                Debug.LogError("RobotConnectionService::Socket closed");
                _receivedSocketClosed = false;
                RemoveSocketListeners();
                _data.ServerConnectionState = ConnectionStatus.NotConnected;
                _data.OnStateChange();
                TriggerServiceDisconnectedEvent();
            }
        }

        public override void Start(SiroServiceData data)
        {
            if (_data != null && (_data.ServerConnectionState != ConnectionStatus.None && _data.ServerConnectionState != ConnectionStatus.NotConnected))
            {
                Debug.LogError($"Service not in correct state to Start(). Current state: {_data.ServerConnectionState}");
                return;
            }

            base.Start(data);
            InitWebSocketClient();
        }

        private void InitWebSocketClient()
        {
            try
            {
                Debug.Log($"InitWebSocket:{_data.ServerUrl}");
                _client = new SocketIOClient(new SocketIOClientOption(EngineIOScheme.http, _data.Host, _data.Port, policyPort, path, reconnection));
                Debug.Log("SocketIOClient created.");
                AddListeners();
                Debug.Log("Connecting to server...");
                _client.Connect();
            }
            catch (ArgumentException e)
            {
                Debug.LogError(e);
                RemoveSocketListeners();
                _data.ServerConnectionState = ConnectionStatus.ConnectionError;
                _data.ConnectionError = e.Message;
                _data.OnStateChange();
                TriggerServiceConnectionErrorEvent();
            }
        }

        private void AddListeners()
        {
            _client.On(SocketIOEvent.CONNECTION, OnSocketOpened);
            _client.On(SocketIOEvent.DISCONNECT, OnSocketClosed);
            _client.On(SocketIOEvent.ERROR, OnSocketError);
            _client.On(MESSAGE, OnMessage);
        }
        
        private void OnMessage(JToken[] message)
        {
            if (message == null || message.Length <= 0 || message[0] == null) return;
            var messageString = message[0].ToString();
            if (messageString == null || messageString == string.Empty)
            {
                Debug.LogError($"SiroService:: received null message {(messageString == null)}");
                return;
            }
            lock (_queueLock)
            {
                _messages.Enqueue(messageString);
            }
        }
        
        public override void Stop()
        {
            Debug.Log($"Stop: {_data?.ServerConnectionState}");
            if (_data != null && (_data.ServerConnectionState == ConnectionStatus.Disconnecting ||
                                  _data.ServerConnectionState == ConnectionStatus.NotConnected))
            {
                Debug.LogError($"Service already disconnected, or in process of: {_data.ServerConnectionState}");
                return;
            }
            
            base.Stop();
            _client.Close();
        }

        public override void Dispose()
        {
            base.Dispose();
            if (_client == null) return;
            if (_data.ServerConnectionState == ConnectionStatus.Connected ||
                _data.ServerConnectionState == ConnectionStatus.Connecting)
            {
                _client.Close();
            }

            _client.Dispose();
        }

        public override void SendMessage(string messageType, IClientMessage message)
        {
            var json = JsonConvert.SerializeObject(message);//, SiroServiceJsonSettings.Settings);
            _client.Emit(messageType, json);
        }

        private void RemoveSocketListeners()
        {
            if (_client == null) return;
            _client.Off(SocketIOEvent.CONNECTION, OnSocketOpened);
            _client.Off(SocketIOEvent.DISCONNECT, OnSocketClosed);
            _client.Off(SocketIOEvent.ERROR, OnSocketError);
            _client.Off(MESSAGE, OnMessage);
        }

        private void HandleSocketOpened()
        {
            _data.ServerConnectionState = ConnectionStatus.Connected;
            _data.OnStateChange();
            TriggerServiceConnectedEvent();
        }

        private void OnSocketClosed()
        {
            RemoveSocketListeners();
            Debug.Log($"OnSocketClosed");
            _receivedSocketClosed = true;
        }

        private void OnSocketOpened()
        {
            Debug.Log($"OnSocketOpened");
            _receivedSocketOpen = true;
        }

        private void OnSocketError()
        {
            Debug.LogError($"OnSocketError");
            _data.ServerConnectionState = ConnectionStatus.ConnectionError;
            _data.ConnectionError = "Received Socket Error";
            _data.OnStateChange();
            TriggerServiceConnectionErrorEvent();
        }

        public override void SendRequest(string eventName)
        {
            Debug.Log($"WorldGraph::SiroService:SendRequest {eventName}");
            _client.Emit(eventName);
        }
    }
}