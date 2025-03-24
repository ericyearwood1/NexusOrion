using Multiplayer.Runtime.Data;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Multiplayer.Runtime.Messages.Server
{
    public class RoomStateMessage : ServerMessage
    {
        public const string Type = "room_state";
        
        [JsonProperty("room")]
        public Room Room;
    }
}