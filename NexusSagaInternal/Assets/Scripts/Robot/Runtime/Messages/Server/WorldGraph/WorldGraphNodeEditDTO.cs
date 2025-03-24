using System;
using Newtonsoft.Json;
using Robot.Runtime.Data.WorldGraph;

namespace Robot.Runtime.Messages.Server
{
    [Serializable]
    public class WorldGraphNodeEditDTO
    {
        [JsonProperty("object")]
        public WorldGraphNodeEdit Node;
    }
}