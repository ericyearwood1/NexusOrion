using System;
using Newtonsoft.Json;
using UnityEngine;

namespace Robot.Runtime.Data.WorldGraph
{
    [Serializable]
    public class WorldGraphNode
    {
        [JsonProperty("object_id")]
        public string Id;
        
        [JsonProperty("object_name")]
        public string ObjectName;

        [JsonProperty("category_tag")]
        public string Category;

        [JsonProperty("room")]
        public string Room;

        [JsonProperty("bbox_extent")]
        public Vector3 Extents;

        [JsonProperty("bbox_center")]
        public Vector3 Position;
        
        [JsonProperty("yaw")]
        public float Yaw;
    }
}