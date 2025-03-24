using System;
using Newtonsoft.Json;
using UnityEngine;

namespace Robot.Runtime.Data.Robot
{
    [Serializable]
    public struct YawPitchRoll
    {
        [JsonProperty("pitch")] 
        public float Pitch;
        
        [JsonProperty("roll")] 
        public float Roll;
        
        [JsonProperty("yaw")] 
        public float Yaw;
        
    }
}