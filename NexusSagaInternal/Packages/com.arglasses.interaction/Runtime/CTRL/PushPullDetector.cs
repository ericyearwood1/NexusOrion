using System;
using System.Collections.Generic;
using UnityEngine;
using static ARGlasses.Interaction.DiscreteHandMotion;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class PushPullDetector
    {
        [SerializeField] private float _gestureThrottle = 0.6f;

        public float GestureThrottle
        {
            get => _gestureThrottle;
            set => _gestureThrottle = value;
        }

        [SerializeField] private float _accelerationPeaksZDeadzone = 0.1f;

        public float AccelerationPeaksZDeadzone
        {
            get => _accelerationPeaksZDeadzone;
            set => _accelerationPeaksZDeadzone = value;
        }

        [SerializeField] private Vector2 _isPanningThreshold = new(0.075f, 0.1f);

        public Vector2 IsPanningThreshold
        {
            get => _isPanningThreshold;
            set => _isPanningThreshold = value;
        }

        [SerializeField] private float _bufferWindow = 1.5f;

        public float BufferWindow
        {
            get => _bufferWindow;
            set => _bufferWindow = value;
        }

        [SerializeField, ReadOnly] public DeltaTimeBufferMulti _accelerationBuffer = new(3);
        public DeltaTimeBufferMulti AccelerationBuffer => _accelerationBuffer;

        [SerializeField, ReadOnly] private Vector2 _projectedRadiansAtRest;
        [SerializeField, ReadOnly] private bool _panningExceededX;
        [SerializeField, ReadOnly] private bool _panningExceededY;
        [SerializeField, ReadOnly] private float _lastGestureTime;

        [SerializeField, ReadOnly] private List<DeltaTimeBuffer.Event> _peaks = new();
        [SerializeField, ReadOnly] private Dictionary<float, string> _tags = new();
        public Dictionary<float, string> Tags => _tags;

        private float _bufferResetTime;

        public void Reset(Vector2 projectedRadians)
        {
            _found = None;
            _bufferResetTime = _accelerationBuffer.Z.AccumulatedTime;
            _projectedRadiansAtRest = projectedRadians;
            _panningExceededX = false;
            _panningExceededY = false;

            _tags[_accelerationBuffer.Z.Latest.BufferTime] = "Reset";
            Debug.Log($"PushPullDetector: Reset");
        }

        public void AddSample(Vector3 acceleration, float sampleDeltaTime)
        {
            _accelerationBuffer.Duration = _bufferWindow;
            _accelerationBuffer.AddEvent(acceleration, sampleDeltaTime);
        }

        [SerializeField, ReadOnly] private DiscreteHandMotion _found;

        public DiscreteHandMotion AnalyzeAccelerationBuffer(Vector2 projectedRadians)
        {
            if (_tags.Count > 2000) _tags.Clear(); // hacky

            _accelerationBuffer.Z.FindPeaks(_peaks, _bufferResetTime, AccelerationPeaksZDeadzone);

            var orientationDeltaSinceRest = projectedRadians - _projectedRadiansAtRest;
            _panningExceededX |= Mathf.Abs(orientationDeltaSinceRest.x) > _isPanningThreshold.x;
            _panningExceededY |= Mathf.Abs(orientationDeltaSinceRest.y) > _isPanningThreshold.y;

            if (_found == None && _peaks.Count == 1)
            {
                var firstPeak = _peaks[0];
                _found = Mathf.Sign(firstPeak.Val) > 0 ? Push : Pull;
                _tags[firstPeak.BufferTime] = _found.ToString();
                _lastGestureTime = Time.time;

                Debug.Log($"PushPullDetector: {firstPeak.BufferTime} {_found}");
                return _found;
            }

            var isZero = 0 == _accelerationBuffer.Z.Latest.Val.WithDeadzone(AccelerationPeaksZDeadzone);

            if (_found != None && isZero)
            {
                if (_peaks.Count > 1)
                {
                    var firstPeak = _peaks[0];
                    var secondPeak = _peaks[1];
                    _tags[secondPeak.BufferTime] = "Peak";
                    Reset(projectedRadians);
                }

                if (Time.time - _lastGestureTime > GestureThrottle) Reset(projectedRadians);
            }


            return None;
        }
    }
}
