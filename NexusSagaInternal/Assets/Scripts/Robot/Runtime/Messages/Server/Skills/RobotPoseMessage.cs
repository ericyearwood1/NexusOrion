using System;
using Newtonsoft.Json;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.Skills
{
    [Serializable]
    public class RobotPoseMessage 
    {
        [JsonProperty("timestamp")]
        public string Timestamp;
        [JsonProperty("position")]
        public float[] Position;
        [JsonProperty("yaw")]
        public float Yaw;
    
    }
}