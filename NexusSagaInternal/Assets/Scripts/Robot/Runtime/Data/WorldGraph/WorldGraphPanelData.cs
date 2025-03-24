using System;
using Multiplayer.Runtime.Data;

namespace Robot.Runtime.Data.WorldGraph
{
    [Serializable]
    public class WorldGraphPanelData
    {
        public string RoomName;
        public SpatialAnchorData AnchorView;
    }
}