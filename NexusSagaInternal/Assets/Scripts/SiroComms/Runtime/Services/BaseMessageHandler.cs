using System;
using Newtonsoft.Json;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public abstract class BaseMessageHandler : IMessageHandler
    {
        public abstract void HandleMessage(string messageType, string message);
        
        protected void ProcessMessage<T>(string message, Action<T> callback)
        {
            var siroMessage = JsonConvert.DeserializeObject<T>(message);
            if (siroMessage == null)
            {
                Debug.LogError($"Could not process message {message}");
                return;
            }
            callback?.Invoke(siroMessage);
        }
    }
}