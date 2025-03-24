using System;
using Multiplayer.Runtime.Data;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Multiplayer.Runtime.Messages.Server
{
    [Serializable]
    public class InitialStateMessage : ServerMessage
    {
        public const string Type = "initial_state";
        
        [JsonProperty("sid")]
        public string SID;
        
        [JsonProperty("rooms")]
        public string[] list;
        
        [JsonProperty("config")]
        public FeatureConfiguration Config;
    }
}