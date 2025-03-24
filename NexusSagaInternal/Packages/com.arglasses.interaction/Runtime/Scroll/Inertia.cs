// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public class Inertia : MonoBehaviour
    {
        public interface IBoundaries
        {
            public bool HasOvershoot();
            Vector3 Overshoot { get; }
            Vector3 OvershootOrigin { get; }
        }

        public class Boundless : IBoundaries
        {
            public static readonly Boundless Instance = new Boundless();
            private Boundless() { }
            public bool HasOvershoot() => false;
            public Vector3 Overshoot => Vector3.zero;
            public Vector3 OvershootOrigin => Vector3.zero;
        }

        [Serializable]
        public class InertiaConfig
        {
            public static InertiaConfig GetDefaultUnique() => new InertiaConfig();

            [SerializeField] [Range(0.0f, 3.0f)]  private float _accelerationScale = 1.15f;

            public float AccelerationScale
            {
                get => _accelerationScale;
                set => _accelerationScale = value;
            }

            [SerializeField] [Range(0.01f, 0.5f)] private float _minimumVelocityToBeginAccelerating = 0.15f;

            public float MinimumVelocityToBeginAccelerating
            {
                get => _minimumVelocityToBeginAccelerating;
                set => _minimumVelocityToBeginAccelerating = value;
            }

            [SerializeField] [Range(1f, 4f)] private float _maximumAccelerationFactor = 2.5f;

            public float MaximumAccelerationFactor
            {
                get => _maximumAccelerationFactor;
                set => _maximumAccelerationFactor = value;
            }

            [SerializeField] [Range(0.5f, 32f)] private float _inertiaUpRateRate = 16;

            public float InertiaUpRate
            {
                get => _inertiaUpRateRate;
                set => _inertiaUpRateRate = value;
            }

            [SerializeField] [Range(0.5f, 12f)] private float _inertiaDownRateRate = 8f;

            public float InertiaDownRate
            {
                get => _inertiaDownRateRate;
                set => _inertiaDownRateRate = value;
            }
        }

        [Serializable]
        public struct Event
        {
            public Vector3 Delta;
            public Vector3 Absolute;
            public Vector3 Acceleration;

            public Event(Vector3 delta, Vector3 absolute, Vector3 acceleration)
            {
                Delta = delta;
                Absolute = absolute;
                Acceleration = acceleration;
            }

            public bool HasInertia => HasDelta || HasAbsolute;
            public bool HasDelta => Delta != default;
            public bool HasAbsolute => Absolute != default;
        }

        public event Action<Event> WhenDelta = delegate { };
        public event Action WhenClear = delegate { };
        public const float MinInteriaMagnitude = 0.0002f;

        private bool _allowAcceleration = true;

        public bool AllowAcceleration
        {
            get => _allowAcceleration;
            set => _allowAcceleration = value;
        }

        [SerializeField, ReadOnly] private bool _isGrabbed;
        public bool IsGrabbed => _isGrabbed;

        private Vector3 _pendingDelta;

        [SerializeField] private InertiaConfig _config;

        public InertiaConfig Config
        {
            get => _config == null ? _config = InertiaConfig.GetDefaultUnique() : _config;
            set => _config = value;
        }

        public IBoundaries Boundaries
        {
            get => _boundaries == null ? Boundless.Instance : _boundaries;
            set => _boundaries = value;
        }

        [SerializeField] private float _overshootResistanceScale = 6f;
        [SerializeField] private float _overshootReboundScale = 3f;
        [SerializeField, ReadOnly] private Vector3 _grabbedOvershoot;
        private IBoundaries _boundaries;

        private void OnEnable()
        {
            ClearInertia();
        }

        private void OnDisable()
        {
            ClearInertia();
        }

        // NOT A DependBehavior!
        private void Start() { }

        public void ClearInertia()
        {
            _value = Vector3.zero;
            WhenClear();
        }

        public void AddDelta(Vector3 delta, SelectionPhase phase = SelectionPhase.None)
        {
            if (phase.IsBegin()) ClearInertia();
            _isGrabbed = phase.IsBegin() || phase.IsUpdate();
            _pendingDelta += delta;
        }

        [FormerlySerializedAs("_inertia")] [SerializeField, ReadOnly] private Vector3 _value = Vector3.zero;
        public Vector3 Value => _value;
        [SerializeField] private float _maxOvershootInertia = 0.001f;

        private void Update()
        {
            Vector3 acceleration = default;
            if (_allowAcceleration && _isGrabbed && Config.AccelerationScale > 0 && Config.MinimumVelocityToBeginAccelerating != 0)
            {
                var frameRateScale = Time.deltaTime * 60;
                var accelMagnitude = frameRateScale * _pendingDelta.magnitude / Config.MinimumVelocityToBeginAccelerating;
                accelMagnitude = Mathf.Min(accelMagnitude, _pendingDelta.magnitude * Config.MaximumAccelerationFactor);
                acceleration = _pendingDelta.normalized * Mathf.Pow(accelMagnitude, 2) * Config.AccelerationScale;
                _pendingDelta += acceleration;
            }

            Vector3 delta;
            if (_isGrabbed || _pendingDelta != Vector3.zero) // Build up inertia, retain the pendingDelta
            {
                _value = Vector3.Lerp(_value, _pendingDelta, Time.deltaTime * Config.InertiaUpRate);
                delta = _pendingDelta;
            }
            else // Decay inertia, discard pendingDelta
            {
                _value = Vector3.Lerp(_value, Vector3.zero, Time.deltaTime * Config.InertiaDownRate);
                if (!HasInertia) _value = Vector3.zero;
                delta = _value;
            }

            ContainerOvershoot(ref delta, out var absolute);
            var e = new Event(delta, absolute, acceleration);
            if (e.HasInertia) WhenDelta(e);
            _pendingDelta = Vector3.zero;
        }

        public bool HasInertia => _value.magnitude > MinInteriaMagnitude;

        [ReadOnly, SerializeField] private bool _isRebounding;
        private void ContainerOvershoot(ref Vector3 delta, out Vector3 absolute)
        {
            absolute = default;

            if (Boundaries.HasOvershoot())
            {
                if (_isGrabbed) OvershootFromGrab(ref delta, out absolute);
                else OvershootFromInertia(ref delta);
            }
            else
            {
                _isRebounding = false;
                _grabbedOvershoot = default;
                delta -= Boundaries.Overshoot;
            }
        }

        private void OvershootFromGrab(ref Vector3 delta, out Vector3 absolute)
        {
            ClearInertia();
            if (_grabbedOvershoot == default) _grabbedOvershoot = Boundaries.Overshoot;
            _grabbedOvershoot += delta;

            float ElasticPosition(float overshootAxis)
            {
                var signedStrength = Mathf.Sign(overshootAxis) * 0.2f;
                return signedStrength * (overshootAxis / (overshootAxis + signedStrength));
            }

            var elasticPosition = new Vector3(
                ElasticPosition(_grabbedOvershoot.x),
                ElasticPosition(_grabbedOvershoot.y),
                ElasticPosition(_grabbedOvershoot.z)
            );

            var elasticOvershootCalibration = 0.6f;
            absolute = Boundaries.OvershootOrigin + elasticOvershootCalibration * elasticPosition;
            delta = Vector3.zero;
        }

        private void OvershootFromInertia(ref Vector3 delta)
        {
            _grabbedOvershoot = Vector3.zero;

            _isRebounding = _isRebounding || 0 > Vector3.Dot(-1 * Boundaries.Overshoot.normalized, _value.normalized);
            if (_isRebounding)
            {
                delta += Vector3.Lerp(Vector3.zero, -Boundaries.Overshoot, _overshootReboundScale * Time.deltaTime);
                ClearInertia();
            }
            else
            {
                delta += Boundaries.Overshoot * -_overshootResistanceScale * Time.deltaTime;
                _value = Vector3.ClampMagnitude(_value, _maxOvershootInertia);
            }
        }
    }
}
