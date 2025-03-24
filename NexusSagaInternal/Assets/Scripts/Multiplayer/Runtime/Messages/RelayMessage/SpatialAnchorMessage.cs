using Multiplayer.Runtime.Data;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using SiroComms.Runtime.Messages.Server;

namespace Multiplayer.Runtime.Messages.Client
{
    public class SpatialAnchorMessage : ServerMessage, IClientMessage
    {
        public const string Type = "spatial_anchor_update";
        [JsonProperty("room")]
        public string Room;
        [JsonProperty("anchor_data")]
        public SpatialAnchorData Data;
    }
}