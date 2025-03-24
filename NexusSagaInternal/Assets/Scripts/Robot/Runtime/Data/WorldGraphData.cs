using System.Collections.Generic;
using Robot.Runtime.Config;
using Robot.Runtime.Data.WorldGraph;
using UnityEngine;

namespace Robot.Runtime.Data
{
    public class WorldGraphData
    {
        public readonly Dictionary<string, WorldGraphNodeData> Nodes = new ();
        public GameObject FurnitureHighlightPrefab;
        public GameObject ObjectHighlightPrefab;
        public GameObject IconPrefab;
        public GameObject PanelPrefab;
        public WorldGraphObjectConfig IconConfig;
        public Transform ViewContainer;
    }
}