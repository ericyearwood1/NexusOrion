using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Client;
using SiroComms.Runtime.Messages.Server;
using UnityEngine;

namespace Multiplayer.Runtime.Messages.Client
{
    [Serializable]
    public class SyncedEntityMessage : ServerMessage, IClientMessage
    {
        public const string Type = "synced_entity";
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