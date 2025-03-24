using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class DeltaTimeBufferMulti
    {
        [SerializeField] private DeltaTimeBuffer[] _buffers;
        public DeltaTimeBuffer this[int i] => _buffers[i];

        private int _count;
        public int Count => _count;

        private float _duration;

        public DeltaTimeBufferMulti(int count, float duration = DeltaTimeBuffer.DefaultDuration)
        {
            _duration = duration;
            _count = count;
            _buffers = new DeltaTimeBuffer[count];
            for (int i = 0; i < _count; i++) _buffers[i] = new DeltaTimeBuffer(duration);
        }

        public float Duration
        {
            get => _duration;
            set
            {
                for (int i = 0; i < _count; i++) _buffers[i].Duration = value;
            }
        }

        public DeltaTimeBuffer X => _buffers[0];
        public DeltaTimeBuffer Y => _buffers[1];
        public DeltaTimeBuffer Z => _buffers[2];

        public Vector3 Latest => new(X.Latest.Val, Y.Latest.Val, Z.Latest.Val);
        public Vector3 Average => new(X.Average, Y.Average, Z.Average);
        public Vector3 AverageSlope => new(X.AverageSlope, Y.AverageSlope, Z.AverageSlope);

        public void AddEvent(Vector3 value, float deltaTime)
        {
            for (int i = 0; i < _count; i++) _buffers[i].AddEvent(value[i], deltaTime);
        }

        public void Reset()
        {
            for (int i = 0; i < _count; i++) _buffers[i].Reset();
        }

        public void FindPeaks(List<List<DeltaTimeBuffer.Event>> peaks, float deadzone)
        {
            for (int i = 0; i < _count; i++) _buffers[i].FindPeaks(peaks[i], deadzone);
        }
    }
}
