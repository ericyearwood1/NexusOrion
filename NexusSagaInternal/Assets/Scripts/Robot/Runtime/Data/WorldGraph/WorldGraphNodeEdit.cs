using System;
using Newtonsoft.Json;

namespace Robot.Runtime.Data.WorldGraph
{
    [Serializable]
    public class WorldGraphNodeEdit : WorldGraphNode
    {
        [JsonProperty("receptacle")]
        public string Receptacle;

        [JsonProperty("operation")]
        public string Operation;
    }
}