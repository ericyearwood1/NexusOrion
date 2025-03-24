using System;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Data
{
    [Serializable]
    public class User
    {
        [JsonProperty("user_id")]
        public ulong Id;
        [JsonProperty("sid")]
        public string ServerId;// i.e. oculus id
        [JsonProperty("display_name")]
        public string DisplayName;
        [JsonProperty("room")]
        public string Room;
        [JsonProperty("color")]
        public int Color;
        [JsonProperty("head_entity_id")]
        public string HeadEntityId;
        [JsonProperty("user_type")]
        public UserType UserType;

        public bool IsUserEntity(string entityId)
        {
            return HeadEntityId == entityId;
        }
        
        public bool IsHost()
        {
            return UserType == UserType.Host;
        }
    }
}