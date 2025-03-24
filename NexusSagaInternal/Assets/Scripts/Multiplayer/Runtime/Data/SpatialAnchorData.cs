using System;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Data
{
    [Serializable]
    public class SpatialAnchorData
    {
        [JsonProperty("name")]
        public string Name;
        
        [JsonProperty("room")]
        public string Room;
        
        [JsonProperty("details")]
        public string Details;
        
        [JsonProperty("uuid")]
        public Guid UUID;
        
        [JsonProperty("state")]
        public SpatialAnchorState State;
        
        [JsonProperty("anchor_type")]
        public SpatialAnchorType AnchorType;

        // for oculus anchors, this is the oculus id of user that created it
        [JsonProperty("created_by")]
        public ulong CreatedBy;

        [JsonProperty("shared_with")]
        public List<ulong> SharedWith = new();

        [NonSerialized] public string ShareError;
        [NonSerialized] public string SaveError;
        [NonSerialized] public int LoadAttempts;
    }
}