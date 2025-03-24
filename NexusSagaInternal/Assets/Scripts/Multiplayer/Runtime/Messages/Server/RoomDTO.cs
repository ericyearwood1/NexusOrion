using System;
using Multiplayer.Runtime.Data;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Messages.Server
{
    [Serializable]
    public class RoomDTO
    {
        [JsonProperty("room")]
        public Room Room;
    }
}