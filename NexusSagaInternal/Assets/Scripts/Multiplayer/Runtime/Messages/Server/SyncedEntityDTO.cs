using System;
using Newtonsoft.Json;
using UnityEngine;

namespace Multiplayer.Runtime.Messages.Server
{
    [Serializable]
    public class SyncedEntityDTO
    {
        [JsonProperty("room")]
        public string Room;
        [JsonProperty("entity_id")]
        public string EntityId;
        [JsonProperty("position")]
        public Vector3 Position;
        [JsonProperty("rotation")]
        public Quaternion Rotation;
        [JsonProperty("scale")]
        public Vector3 Scale;
    }
}