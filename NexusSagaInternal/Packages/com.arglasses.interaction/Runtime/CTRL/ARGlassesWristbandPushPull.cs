using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.ARGlassesWristbandConsumers)]
    public class ARGlassesWristbandPushPull : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;
        private Vector2 ProjectedRadians => _wristband.ProjectedRadians;

        private readonly IOneEuroFilter<Vector3> _accelerationFilter = OneEuroFilter.CreateVector3();

        [SerializeField, ReadOnly] private Vector3 _accelerationWithPitch;
        public Vector3 AccelerationWithPitch => _accelerationWithPitch;

        public OneEuroFilterPropertyBlock _accelFilterProperties = new(0.8f, 0.1f, 0.05f);

        [SerializeField] private float _accelerationDeadzone = 0.2f;

        public float AccelerationDeadzone
        {
            get => _accelerationDeadzone;
            set => _accelerationDeadzone = value;
        }

        [SerializeField] private float _toWorldSpaceScale = 0.3f;

        [SerializeField] private Vector3 _accelerationScale = new(0.6f, 0.6f, 0.6f);

        public Vector3 AccelerationScale
        {
            get => _accelerationScale;
            set => _accelerationScale = value;
        }

        public float ToWorldSpaceScale
        {
            get => _toWorldSpaceScale;
            set => _toWorldSpaceScale = value;
        }

        [SerializeField] private PushPullDetector _detector = new();
        public PushPullDetector Detector => _detector;

        [SerializeField, ReadOnly] private Vector3 _accelerationSlope;

        private void Awake()
        {
            this.Ancestor(ref _wristband);
            _wristband.WhenPinch += HandlePinch;
        }

        private void HandlePinch(HandFinger finger, bool isPressed)
        {
            if (!isPressed) return;
            _detector.Reset(ProjectedRadians);
        }

        private void OnEnable() => _wristband.Streams.WhenImuAccelerationSample += HandleAcceleration;
        private void OnDisable() => _wristband.Streams.WhenImuAccelerationSample -= HandleAcceleration;

        public event Action<Vector3, float, Vector2> WhenAccelerationSample = delegate { };
        private void HandleAcceleration(ARGlassesWristbandStreams.ContinuousSample<Vector3> continuousSample)
        {
            var sampleDeltaTime = continuousSample.DeltaTime;
            if (sampleDeltaTime <= 0)
            {
                Debug.LogWarning($"Zero delta time: {continuousSample.Data.x}, {continuousSample.Data.y}, {continuousSample.Data.z}");
                return;
            }

            _accelerationFilter.SetProperties(_accelFilterProperties);
            _accelerationWithPitch = _accelerationFilter.Step(-1 * _wristband.PitchedAcceleration, sampleDeltaTime);
            _accelerationWithPitch.Scale(AccelerationScale);

            _accelerationWithPitch.x = _accelerationWithPitch.x.WithDeadzone(_accelerationDeadzone);
            _accelerationWithPitch.y = _accelerationWithPitch.y.WithDeadzone(_accelerationDeadzone);
            _accelerationWithPitch.z = _accelerationWithPitch.z.WithDeadzone(_accelerationDeadzone);

            _detector.AddSample(_accelerationWithPitch, sampleDeltaTime);
            WhenAccelerationSample(_accelerationWithPitch, sampleDeltaTime, ProjectedRadians);
        }

        public event Action<DiscreteHandMotion> WhenDiscreteHandMotion = delegate { };
        private void Update()
        {
            var motion = _detector.AnalyzeAccelerationBuffer(ProjectedRadians);
            if (!motion.IsNone()) WhenDiscreteHandMotion(motion);
        }
    }
}
