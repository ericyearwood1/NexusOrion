using System;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Data
{
    [Serializable]
    public class Room
    {
        [JsonProperty("id")]
        public string Id;
        
        [JsonProperty("connected_users")]
        public List<User> Users = new();
        
        [JsonProperty("space_sync_anchor")]
        public SpatialAnchorData ColocationAnchor;
        
        [JsonProperty("robot_home_anchor")]
        public SpatialAnchorData RobotHomeAnchor;
        
        [JsonProperty("is_robot_feature_available")]
        public bool IsRobotFeatureAvailable;
        
        [JsonProperty("is_augment_feature_available")]
        public bool IsAugmentFeatureAvailable;
        
        [JsonProperty("active_features")]
        public List<string> ActiveFeatures = new();

        [JsonProperty("world_graph_anchors")]
        public List<SpatialAnchorData> WorldGraphAnchors = new();
    }
}