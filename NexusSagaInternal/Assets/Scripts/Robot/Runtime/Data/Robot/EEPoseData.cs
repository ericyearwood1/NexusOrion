using System;
using Newtonsoft.Json;
using UnityEngine;

namespace Robot.Runtime.Data.Robot
{
    [Serializable]
    public class EEPoseData
    {
        [JsonProperty("position")]
        public Vector3 Position;
        [JsonProperty("orientation")]
        public Quaternion Orientation;
        [JsonProperty("yaw_pitch_roll")]
        public YawPitchRoll PRY;
    }
}