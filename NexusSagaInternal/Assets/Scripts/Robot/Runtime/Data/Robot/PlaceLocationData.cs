using UnityEngine;

namespace Robot.Runtime.Data.Robot
{
    public class PlaceLocationData : HighlightData
    {
        public string ActionId;
        public Quaternion TargetOrientation;
        public Vector3 TargetEuler;
    }
}