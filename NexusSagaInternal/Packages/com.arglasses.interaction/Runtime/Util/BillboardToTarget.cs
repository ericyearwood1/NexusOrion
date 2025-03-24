using UnityEngine;

namespace ARGlasses.Interaction
{
    public class BillboardToTarget : MonoBehaviour
    {
        [SerializeField, Tooltip("Camera.main by default")] private Transform _target;
        [SerializeField] private bool _flipZ = true;
        [SerializeField] private bool _allowPitchRotation = true;

        void LateUpdate()
        {
            if (!_target) _target = Camera.main.transform;

            var toTarget = _target.position - transform.position;
            toTarget *= _flipZ ? -1 : 1;
            if (!_allowPitchRotation) toTarget = toTarget.WithY(0);

            if (toTarget == default) return;
            transform.rotation = Quaternion.LookRotation(toTarget);
        }
    }
}
