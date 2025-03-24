using System;
using Newtonsoft.Json;
using UnityEngine;

namespace Robot.Runtime.Data.Robot
{
    [Serializable]
    public class RobotState
    {
        [JsonProperty("status")]
        public string Status;  
        [JsonProperty("position")]
        public Vector2 Position;
        [JsonProperty("yaw")]
        public float Yaw;
    }
}