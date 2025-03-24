using System;
using Newtonsoft.Json;
using Robot.Runtime.Data.Robot;
using UnityEngine;

namespace Robot.Runtime.Messages.Server.Skills
{
    [Serializable]
    public class PlaceLocationMessage 
    {
        [JsonProperty("action_id")]
        public string ActionId;
        
        [JsonProperty("position")]
        public float[] Position;
                
        [JsonProperty("target_orientation")]
        public float[] TargetOrientation;
        
        [JsonProperty("target_euler")]
        public Vector3 TargetEuler;
    }
}