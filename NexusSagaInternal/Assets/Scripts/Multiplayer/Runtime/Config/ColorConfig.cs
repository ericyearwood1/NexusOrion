using Multiplayer.Runtime.Data;
using UnityEngine;

namespace Multiplayer.Runtime.Config
{
    [CreateAssetMenu(menuName = "Nexus/Create color config", fileName = "MultiplayerColorOptions")]
    public class ColorConfig : ScriptableObject
    {
        public ColourOption[] Options;
    }
}