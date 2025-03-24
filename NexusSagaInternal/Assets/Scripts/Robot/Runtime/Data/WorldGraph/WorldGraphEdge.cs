using System;
using Newtonsoft.Json;

namespace Robot.Runtime.Data.WorldGraph
{
    [Serializable]
    public class WorldGraphEdge
    {
        [JsonProperty("source")]
        public string SourceId;
        [JsonProperty("target")]
        public string TargetId;
        [JsonProperty("relation")]
        public string Relationship;
    }
}