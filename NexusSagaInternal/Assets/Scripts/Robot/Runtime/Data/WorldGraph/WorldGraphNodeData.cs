using System.Collections.Generic;
using UnityEngine;

namespace Robot.Runtime.Data.WorldGraph
{
    public class WorldGraphNodeData
    {
        public string Id;
        public string Name;
        public string Room;
        public string Receptacle;
        public Vector3 Position;
        public Vector3 Extents;
        public List<WorldGraphNodeData> Items = new ();
        public string Category;
        public bool IsRemoved;
        public float Yaw;

        public bool IsHeldByRobot()
        {
            return Receptacle.Contains("robot");
        }

        public bool IsHeldByPerson()
        {
            return Receptacle.Contains("human");
        }
    }
}