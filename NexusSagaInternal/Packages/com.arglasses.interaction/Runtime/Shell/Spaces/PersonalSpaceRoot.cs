// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public class PersonalSpaceRoot : MonoBehaviour
    {
        [SerializeField] private float _distance = 0.8f;
        public float Distance
        {
            get => _distance;
            set => _distance = value;
        }
        
        [SerializeField] private float _distanceMin = 0.5f;
        public float DistanceMin
        {
            get => _distanceMin;
            set => _distanceMin = value;
        }

        [SerializeField] private float _distanceMax = 1.1f;
        public float DistanceMax
        {
            get => _distanceMax;
            set => _distanceMax = value;
        }

        [SerializeField] private float _heightOffset = 0;

        [SerializeField] private float _maxHorizontalAngleLeash = 35.0f;
        [SerializeField] private float _positionLerpSpeed = 8.0f;
        [SerializeField] private float _rotationLerpSpeed = 16.0f;

        [SerializeField] private bool _applyPosition = true;
        [SerializeField] private bool _billboard = true;
        [SerializeField] private bool _billboardIncludesPitch = true;

        private Transform _head;
        private Transform _target;

        protected void Start()
        {
            _head = Camera.main.transform;
            _target = transform;
            LerpToPositionRotation(1, 1, true);
        }

        protected void Update()
        {
            if (!_applyPosition) return;

            LerpToPositionRotation(_positionLerpSpeed * Time.deltaTime, _rotationLerpSpeed * Time.deltaTime);
        }

        public void LerpToPositionRotation(float lerp, float rotLerp, bool forceCenter = false)
        {

            var pos = GetTargetPosition(forceCenter);
            _target.position = Vector3.Lerp(_target.position, pos, lerp);
            Billboard(rotLerp);
        }

        private void Billboard(float rotLerp)
        {
            if (!_billboard) return;
            Vector3 lookDirection = _target.position - _head.transform.position;
            if (!_billboardIncludesPitch) lookDirection.y = 0;
            if (lookDirection.sqrMagnitude < 0.0001f) return;
            var rotationTarget = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
            _target.rotation = Quaternion.Lerp(_target.rotation, rotationTarget, rotLerp);
        }

        private Vector3 GetTargetPosition(bool forceCenter)
        {
            var currentRotOffset = GetCurrentOffsetFromCameraForward(forceCenter);
            var targetDir = (currentRotOffset * _head.forward.WithY(0).normalized);

            var camToThisDistance = (_target.position - _head.position).WithY(0).magnitude;
            camToThisDistance = Mathf.Clamp(camToThisDistance, _distanceMin, _distanceMax);

            var pos = (targetDir * camToThisDistance) + _head.position;
            pos.y = _head.position.y + _heightOffset;
            return pos;
        }

        private Quaternion GetCurrentOffsetFromCameraForward(bool forceCenter)
        {
            var camToThisXZDir = (_target.position - _head.position).WithY(0).normalized;
            var offsetQuaternion = Quaternion.FromToRotation(_head.forward.WithY(0).normalized, camToThisXZDir);

            var offsetDegrees = offsetQuaternion.eulerAngles;
            var maxAngle = forceCenter ? 1 : _maxHorizontalAngleLeash;
            if (offsetDegrees.y < 180 && offsetDegrees.y > maxAngle)
            {
                offsetQuaternion.eulerAngles = new Vector3(offsetDegrees.x, maxAngle, offsetDegrees.z);
            }
            else if (offsetDegrees.y > 180 && offsetDegrees.y < 360 - maxAngle)
            {
                offsetQuaternion.eulerAngles = new Vector3(offsetDegrees.x, 360 - maxAngle, offsetDegrees.z);
            }

            return offsetQuaternion;
        }
    }
}
