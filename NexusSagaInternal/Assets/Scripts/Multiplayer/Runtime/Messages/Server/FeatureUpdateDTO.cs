using System;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Messages.Server
{
    [Serializable]
    public class FeatureUpdateDTO
    {
        [JsonProperty("room")]
        public string Room;
        [JsonProperty("feature")]
        public string Feature;
    }
}