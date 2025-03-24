using UnityEngine;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.ARGlassesWristbandConsumers)]
    public class ARGlassesWristbandTwist : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;

        [SerializeField] private float _quickTwistTimeout = 0.75f;
        [SerializeField] private float _quickTwistThreshold = 300f;
        [SerializeField] private Filters.LowPass _twistVelocityLowPass = new();

        [SerializeField, ReadOnly] private float _twistDegrees;
        private float _lastTwistDegrees;
        public float TwistDegrees => _twistDegrees;

        [SerializeField, ReadOnly] private float _twistVelocityDegrees;
        public float TwistVelocityDegrees => _twistVelocityDegrees;

        [SerializeField, ReadOnly] private float _quickTwistSign;
        [SerializeField, ReadOnly] private int _quickTwistCount;
        private float _lastQuickTwistTime;

        private void Awake()
        {
            this.Ancestor(ref _wristband);
            _wristband.Streams.WhenImuTwistRadiansSample += HandleTwistRadians;
        }

        private void HandleTwistRadians(ARGlassesWristbandStreams.ContinuousSample<float> e)
        {
            _twistDegrees = e.Data * Mathf.Rad2Deg;
            var deltaTime = e.DeltaTime;

            _twistVelocityDegrees = _twistVelocityLowPass.Filter((_twistDegrees - _lastTwistDegrees) / deltaTime);
            _lastTwistDegrees = _twistDegrees;

            if (Time.time - _lastQuickTwistTime > _quickTwistTimeout)
            {
                _quickTwistSign = 0;
                _quickTwistCount = 0;
            }

            var velocitySign = Mathf.Sign(_twistVelocityDegrees);
            if (Mathf.Abs(_twistVelocityDegrees) < _quickTwistThreshold || velocitySign == _quickTwistSign) return;

            _lastQuickTwistTime = Time.time;
            _quickTwistSign = velocitySign;
            _quickTwistCount++;
        }
    }
}
