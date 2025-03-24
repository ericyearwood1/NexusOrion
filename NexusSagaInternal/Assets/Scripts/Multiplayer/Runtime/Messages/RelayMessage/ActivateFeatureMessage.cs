using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Client
{
    [Serializable]
    public class ActivateFeatureMessage : ServerMessage, IClientMessage
    {
        public const string Type = "activate_feature";
        [JsonProperty("event")]
        public string EventName = "multiplayer";
        [JsonProperty("type")]
        public string EventType = Type;
        [JsonProperty("room")]
        public string Room;
        [JsonProperty("feature")]
        public string Feature;
    }
}