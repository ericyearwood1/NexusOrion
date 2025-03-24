using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CTRLIndicatorOrientation : MonoBehaviour
    {
        [SerializeField] private Transform _absoluteOrientationAxis;
        [SerializeField] private Transform _userOrientationAxis;
        [SerializeField] private Transform _accelerationAxis;

        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;

        private void Awake()
        {
            this.Scene(ref _wristband);
            this.Descendant(ref _absoluteOrientationAxis, nameof(_absoluteOrientationAxis), optional: true);
            this.Descendant(ref _userOrientationAxis, nameof(_userOrientationAxis), optional: true);
            this.Descendant(ref _accelerationAxis, nameof(_accelerationAxis), optional: true);
        }

        private void LateUpdate()
        {
            if(_absoluteOrientationAxis) _absoluteOrientationAxis.localRotation = _wristband.AbsoluteOrientation;
            if(_userOrientationAxis) _userOrientationAxis.localRotation = _wristband.UserOrientation;

            if (_accelerationAxis)
            {
                var pitchedAcceleration = _wristband.PitchedAcceleration;
                if(!pitchedAcceleration.IsNearZero()) _accelerationAxis.localRotation = Quaternion.LookRotation(pitchedAcceleration);
                var scale = pitchedAcceleration.magnitude.Map(0, 12, 0.1f, 1.5f, clamp: true);
                _accelerationAxis.localScale = Vector3.one * scale;
            }
        }
    }
}
