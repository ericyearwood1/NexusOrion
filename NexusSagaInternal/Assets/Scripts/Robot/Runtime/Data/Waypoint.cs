using System;
using UnityEngine;

namespace Robot.Runtime.Data
{
    [Serializable]
    public class Waypoint
    {
        public string Name;
        public Vector2 Position;
        public float Yaw;

        public Vector3 Get3DPosition()
        {
            return new Vector3(Position.x, 0, Position.y);
        }
    }
}