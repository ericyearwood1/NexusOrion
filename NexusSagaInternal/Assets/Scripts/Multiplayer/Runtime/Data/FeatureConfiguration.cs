using System;
using Newtonsoft.Json;

namespace Multiplayer.Runtime.Data
{
    /**
    * HAR CONFIG:
    * 0/Default = Enabled
    * 1 = only show in world visualisation
    * 2 = Disable
    */
    [Serializable]
    public class FeatureConfiguration
    {
        [JsonProperty("har")]
        public int HARConfig;

        [JsonProperty("is_notifications_active")]
        public bool IsNotificationsActive;

        [JsonProperty("is_show_place_object_highlight")]
        public bool IsShowPlaceObjectHighlight;

        [JsonProperty("is_show_semantic_place")]
        public bool IsShowSemanticPlace;
        
        [JsonProperty("is_show_world_object_highlights")]
        public bool IsShowWorldObjectHighlights;
        
        [JsonProperty("is_show_target_object_highlight")]
        public bool IsShowTargetObjectHighlight;

        [JsonProperty("is_enable_world_graph_2d_surfaces")]
        public bool IsWorldGraph2DSurfacesEnabled;

        [JsonProperty("saga_ip")]
        public string SagaIP;
        
    }
}