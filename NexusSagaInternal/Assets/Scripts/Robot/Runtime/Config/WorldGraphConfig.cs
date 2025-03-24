using Robot.Runtime.Data.WorldGraph;
using UnityEngine;

namespace Robot.Runtime.Config
{
    [CreateAssetMenu(menuName = "Nexus/Stub World Graph Config", fileName = "Stub World Graph Config")]
    public class WorldGraphConfig : ScriptableObject
    {
        [SerializeField] private WorldGraph _graph;
        public WorldGraph Graph => _graph;
    }
}