using System.Collections.Generic;
using Robot.Runtime.Data;
using UnityEngine;

namespace Robot.Runtime.Config
{
    [CreateAssetMenu(menuName = "Nexus/Stub waypoint Config", fileName = "Stub waypoint Config")]
    public class WaypointConfig : ScriptableObject
    {
        [SerializeField] private List<Waypoint> _waypoints;
        
        public List<Waypoint> Waypoints => _waypoints;
    }
}