using System;
using System.Collections.Generic;
using Newtonsoft.Json;

namespace Robot.Runtime.Data.WorldGraph
{
    [Serializable]
    public class WorldGraph
    {
        [JsonProperty("status")]
        public string Status;

        [JsonProperty("timestamp")]
        public string Timestamp;
        
        [JsonProperty("nodes")]
        public List<WorldGraphNode> Nodes;
        
        [JsonProperty("edges")]
        public List<WorldGraphEdge> Edges;
    }
}