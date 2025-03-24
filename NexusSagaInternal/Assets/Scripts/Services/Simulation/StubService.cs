using System.Collections.Generic;
using Multiplayer.Runtime.Services;
using Newtonsoft.Json;
using Robot.Runtime.Data;
using Robot.Runtime.ServiceHandlers;
using SiroComms.Runtime.Messages.Server;
using UnityEngine;

namespace SiroComms.Runtime.Services
{
    public class StubService
    {
        private readonly Queue<string> _messages;

        protected StubService(Queue<string> messageQueue)
        {
            _messages = messageQueue;
        }

        public virtual void Tick()
        {
            
        }
        
        protected void SendMessage<T>(string serviceName, string messageType, T data)
        {
            var serverMessage = new ServerMessage<T>
            {
                EventName = serviceName,
                EventType = messageType,
                Data = data
            };
            var serializedMessage = JsonConvert.SerializeObject(serverMessage);
            if (serviceName != MultiplayerMessageHandler.Type && messageType != SkillsMessageHandler.EE_POSE &&
                messageType != SkillsMessageHandler.ROBOT_POSE)
            {
                Debug.Log(serializedMessage);
            }
            _messages.Enqueue(serializedMessage);
        }
    }
}