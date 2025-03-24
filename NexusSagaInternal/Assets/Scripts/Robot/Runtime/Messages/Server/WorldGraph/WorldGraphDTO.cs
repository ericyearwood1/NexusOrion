using System;
using Newtonsoft.Json;
using Robot.Runtime.Data.WorldGraph;

namespace Robot.Runtime.Messages.Server
{
    [Serializable]
    public class WorldGraphDTO 
    {
        [JsonProperty("graph")]
        public WorldGraph Graph;
    }
}