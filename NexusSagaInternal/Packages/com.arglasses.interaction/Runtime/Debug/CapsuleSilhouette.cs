using UnityEngine;

namespace ARGlasses.Interaction
{
    [ExecuteAlways]
    public class CapsuleSilhouette : MonoBehaviour
    {
        [SerializeField] private CapsuleCollider _capsule;
        private Plane _plane;

        private void Update()
        {
            _plane = new Plane(-transform.forward, transform.position);
            Line.Intersect(_plane, _capsule);
        }
    }
}
