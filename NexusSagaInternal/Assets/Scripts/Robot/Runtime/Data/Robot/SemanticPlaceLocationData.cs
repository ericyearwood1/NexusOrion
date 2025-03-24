using UnityEngine;

namespace Robot.Runtime.Data.Robot
{
    public class SemanticPlaceLocationData : PlaceLocationData
    {
        public string ActionId;
        public Quaternion UpOrientation;
        public Vector3 TargetEuler;
    }
}