using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using SiroComms.Runtime.Messages.Server;

namespace Multiplayer.Runtime.Messages.RelayMessage
{
    [Serializable]
    public class DeactivateFeatureMessage : ServerMessage, IClientMessage
    {
        public const string Type = "deactivate_feature";
        [JsonProperty("room")]
        public string Room;
        [JsonProperty("event")]
        public string EventName = "multiplayer";
        [JsonProperty("type")]
        public string EventType = Type;
        [JsonProperty("feature")]
        public string Feature;
    }
}