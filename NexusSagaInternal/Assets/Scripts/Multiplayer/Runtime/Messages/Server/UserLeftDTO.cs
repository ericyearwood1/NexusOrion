using System;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Messages.Server
{
    [Serializable]
    public class UserLeftDTO
    {
        [JsonProperty("user_id")]
        public ulong Id;
    }
}