using System;
using Newtonsoft.Json;

namespace Robot.Runtime.Messages.Server.Skills
{
    [Serializable]
    public class TargetObject
    {
        [JsonProperty("threshold")]
        public float Threshold;

        [JsonProperty("position")]
        public float[] Position;
        
        [JsonProperty("normal")]
        public float[] Normal;
    }
}