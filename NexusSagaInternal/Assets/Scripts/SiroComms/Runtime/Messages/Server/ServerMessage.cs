using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using UnityEngine;

namespace SiroComms.Runtime.Messages.Server
{
    [Serializable]
    public class ServerMessage
    {
        [JsonProperty("event")]
        public string EventName;
        
        [JsonProperty("type")]
        public string EventType;
        
        [JsonProperty("data")]
        public Dictionary<object, object> Data;
    }
    
    [Serializable]
    public class ServerMessage<T> : ServerMessage
    {
        [JsonProperty("data")]
        public T Data;
        
        public static ServerMessage<T> Deserialize(string json)
        {
            var serverMessage = JsonConvert.DeserializeObject<ServerMessage>(json);
            var data = JsonConvert.DeserializeObject<T>(JsonConvert.SerializeObject(serverMessage.Data));
            return new ServerMessage<T>
            {
                EventName = serverMessage.EventName,
                EventType = serverMessage.EventType,
                Data = data
            };
        }
    }
}