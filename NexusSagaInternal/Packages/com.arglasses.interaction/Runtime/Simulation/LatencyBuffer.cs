using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class LatencyBuffer<T>
    {
        [SerializeField] private float _latencyMillis = 0;

        [Serializable]
        public struct EventData<T>
        {
            public float Timestamp;
            public T Data;
        }

        [SerializeField, ReadOnly] private EventData<T> _replayedData;
        private readonly Queue<EventData<T>> _buffer = new();
        public event Action<T> WhenDataReplayed = delegate { };

        public LatencyBuffer(float latencyMillis, Action<T> whenDataReplayed)
        {
            _latencyMillis = latencyMillis;
            WhenDataReplayed += whenDataReplayed;
        }

        public float LatencyMillis
        {
            get => _latencyMillis;
            set => _latencyMillis = value;
        }

        public void Add(T sourceData)
        {
            if (_latencyMillis == 0)
            {
                WhenDataReplayed(sourceData);
                return;
            }

            _buffer.Enqueue(new EventData<T> { Timestamp = Time.time, Data = sourceData });
            Update(LatencyMillis);
        }

        public void Update(float latencyMillis)
        {
            LatencyMillis = latencyMillis;
            while (_buffer.Count > 0 && _buffer.Peek().Timestamp <= Time.time - _latencyMillis * 0.001f)
            {
                _replayedData = _buffer.Dequeue();
                WhenDataReplayed(_replayedData.Data);
            }
        }
    }
}
