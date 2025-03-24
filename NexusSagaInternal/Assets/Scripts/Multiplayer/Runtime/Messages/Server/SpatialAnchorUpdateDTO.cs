using System;
using Multiplayer.Runtime.Data;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Messages.Server
{
    [Serializable]
    public class SpatialAnchorUpdateDTO
    {
        [JsonProperty("room")]
        public string Room;
        [JsonProperty("anchor_data")]
        public SpatialAnchorData Data;
    }
}