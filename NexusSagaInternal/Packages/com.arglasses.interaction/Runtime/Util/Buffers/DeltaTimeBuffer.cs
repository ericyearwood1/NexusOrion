using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class DeltaTimeBuffer
    {
        public const float DefaultDuration = 0.2f;

        [Serializable]
        public struct Event
        {
            public float BufferTime;
            public float Val;
            public float DeltaTime;

            public Event(float val, float deltaTime, float bufferTime)
            {
                Val = val;
                DeltaTime = deltaTime;
                BufferTime = bufferTime;
            }
        }

        [SerializeField] private List<Event> _buffer = new();
        public Event this[int i] => _buffer[i];

        public int Count => _buffer.Count;
        [SerializeField, ReadOnly] private int _count;

        public Event Latest => Count > 0 ? _buffer[^1] : default;
        public Event Oldest => Count > 0 ? _buffer[0] : default;

        [SerializeField] private float _duration = DefaultDuration;

        [SerializeField, ReadOnly] private float _accumulatedTime = 0;
        public float AccumulatedTime => _accumulatedTime;

        [SerializeField, ReadOnly] private float _average;
        public float Average => _average;

        [SerializeField, ReadOnly] private float _averageSlope;
        public float AverageSlope => _averageSlope;

        [SerializeField, ReadOnly] private float _min;
        public float Min => _min;

        [SerializeField, ReadOnly] private float _max;
        public float Max => _max;

        public float Range => _max - _min;

        public float Duration
        {
            get => _duration;
            set => _duration = value;
        }

        public DeltaTimeBuffer(float duration = DefaultDuration)
        {
            _accumulatedTime = 0;
            _duration = duration;
        }

        public void Reset()
        {
            _accumulatedTime = 0;
            _buffer.Clear();
        }

        public float ToUnityTime(Event e) => Time.time - _accumulatedTime + e.BufferTime;

        public void AddEvent(float val, float deltaTime)
        {
            _accumulatedTime += deltaTime;

            _buffer.Add(new Event(val: val, deltaTime: deltaTime, bufferTime: _accumulatedTime));
            _buffer.RemoveAll(e => e.BufferTime <= _accumulatedTime - _duration);

            _average = 0f;
            _averageSlope = 0f;
            _min = val;
            _max = val;

            _count = _buffer.Count;
            if (_count == 0) return;

            for (var i = 0; i < _buffer.Count; i++)
            {
                var v = _buffer[i].Val;
                _average += v;

                _min = Mathf.Min(_min, v);
                _max = Mathf.Max(_max, v);

                if (i > 0) _averageSlope += _buffer[i].Val - _buffer[i - 1].Val;
            }

            _average /= _buffer.Count;
        }

        public void FindPeaks(List<Event> peaks, float startTime, float deadzone = ExtensionsUnity.FloatCompareThreshold)
        {
            peaks.Clear();

            Event potentialPeak = default;

            for (int i = 0; i < _buffer.Count; i++)
            {
                var bufferEvent = _buffer[i];
                if(bufferEvent.BufferTime < startTime) continue;

                var value = bufferEvent.Val;
                var isZero = value.WithDeadzone(deadzone) == 0;

                // var isOppositeSign = Mathf.Sign(value) != Mathf.Sign(potentialPeak.Val);
                var peakFound = potentialPeak.Val != 0 && isZero;
                if (peakFound)
                {
                    peaks.Add(potentialPeak);
                    potentialPeak = default;
                }

                if (!isZero && Mathf.Abs(value) > Mathf.Abs(potentialPeak.Val)) potentialPeak = bufferEvent;
            }
        }
    }
}
