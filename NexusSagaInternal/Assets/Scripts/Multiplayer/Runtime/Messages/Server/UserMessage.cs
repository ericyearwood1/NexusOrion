using System;
using Multiplayer.Runtime.Data;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Multiplayer.Runtime.Messages.Server
{
    [Serializable]
    public class UserMessage : ServerMessage
    {
        [JsonProperty("user")]
        public User User;
    }
}