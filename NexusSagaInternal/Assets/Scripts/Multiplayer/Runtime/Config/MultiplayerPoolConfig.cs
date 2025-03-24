using UnityEngine;

namespace Multiplayer.Runtime.Config
{
    [CreateAssetMenu(menuName = "Nexus/Create multiplayer pool config", fileName = "MultiplayerPoolConfig")]
    public class MultiplayerPoolConfig : ScriptableObject
    {
        public int PoolSize = 20;
        public GameObject HeadPrefab;
        public GameObject LeftHandPrefab;
        public GameObject RightHandPrefab;
        public GameObject UserNameDisplayPrefab;
    }
}