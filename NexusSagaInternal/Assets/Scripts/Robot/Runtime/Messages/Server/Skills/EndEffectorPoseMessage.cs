

using System;
using Newtonsoft.Json;
using Robot.Runtime.Data.Robot;
using SiroComms.Runtime.Messages.Server;

namespace Robot.Runtime.Messages.Server.Skills
{
    [Serializable]
    public class EndEffectorPoseMessage 
    {
        [JsonProperty("timestamp")]
        public string Timestamp;
        [JsonProperty("position")]
        public float[] Position;
        [JsonProperty("orientation")]
        public float[] Orientation;
        [JsonProperty("yaw_pitch_roll")]
        public YawPitchRoll PRY;
    
    }
}