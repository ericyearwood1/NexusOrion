using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using SiroComms.Runtime.Data;
using SiroComms.Runtime.Messages.Client;
using SiroComms.Runtime.Messages.Server;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public abstract class BaseSiroService : ISiroService
    {
        protected object _queueLock = new object();
        public delegate void SiroServiceEvent();

        // ConnectionEvents
        public event SiroServiceEvent OnServiceConnected;
        public event SiroServiceEvent OnServiceDisconnected;
        public event SiroServiceEvent OnServiceConnectionError;

        protected readonly Queue<string> _messages = new(1000);
        private readonly Dictionary<string, IMessageHandler> _messageHandlers = new();

        protected SiroServiceData _data;

        public virtual void Start(SiroServiceData data)
        {
            _data = data;
            _data.ConnectionError = null;
            _data.ServerConnectionState = ConnectionStatus.Connecting;
            Debug.Log($"BaseSiroService::Start {this}");
        }

        public virtual void Update()
        {
            while (_messages.Count > 0)
            {
                var message = string.Empty;
                lock (_queueLock)
                {
                    message = _messages.Dequeue();
                }
                if(string.IsNullOrEmpty(message)) return;
                try
                {
                    var serverMessage = JsonConvert.DeserializeObject<ServerMessage>(message);

                    if (string.IsNullOrWhiteSpace(serverMessage.EventName))
                    {
                        Debug.LogError($"Invalid message received : {message}");
                        return;
                    }

                    if (_messageHandlers.TryGetValue(serverMessage.EventName, out var handler))
                    {
                        handler.HandleMessage(serverMessage.EventType, message);
                    }
                    else
                    {
                        Debug.LogError($"Could not find handler for message {serverMessage.EventName}");
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error handling message {message} : {e}");
                }
            }
        }

        public virtual void Stop()
        {
            _data.ServerConnectionState = ConnectionStatus.Disconnecting;
        }

        public virtual void Dispose()
        {
            OnServiceConnected = null;
            OnServiceDisconnected = null;
            OnServiceConnectionError = null;
        }

        public void AddHandler(string type, IMessageHandler handler)
        {
            if (_messageHandlers.ContainsKey(type)) return;
            _messageHandlers.Add(type, handler);
        }

        public abstract void SendMessage(string messageType, IClientMessage message);
        public abstract void SendRequest(string eventName);

        #region Events

        protected void TriggerServiceConnectedEvent()
        {
            OnServiceConnected?.Invoke();
        }

        protected void TriggerServiceDisconnectedEvent()
        {
            OnServiceDisconnected?.Invoke();
        }

        protected void TriggerServiceConnectionErrorEvent()
        {
            OnServiceConnectionError?.Invoke();
        }

        #endregion

    }
}