using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;
using UnityEngine;

namespace Robot.Runtime.Messages.Server.Skills
{
    [Serializable]
    public class NavigationHighlightMessage 
    {
        [JsonProperty("target")]
        public string Target;

        [JsonProperty("position")]
        public float[] Position;
        
        [JsonProperty("action_id")]
        public string ActionId;
    }
}