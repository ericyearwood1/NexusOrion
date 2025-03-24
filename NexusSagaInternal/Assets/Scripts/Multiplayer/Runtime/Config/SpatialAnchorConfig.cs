using UnityEngine;

namespace Multiplayer.Runtime.Config
{
    [CreateAssetMenu(menuName = "Nexus/Create spatial anchor config", fileName = "SpatialAnchorConfig")]
    public class SpatialAnchorConfig : ScriptableObject
    {
        [SerializeField] private bool _isOVRTesting = true;
        [SerializeField] private GameObject _anchorPrefab;

        public bool IsOVRTesting => _isOVRTesting;
        public GameObject AnchorPrefab => _anchorPrefab;
    }
}