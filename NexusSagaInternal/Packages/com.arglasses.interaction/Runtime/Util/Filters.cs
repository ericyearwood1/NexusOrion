using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class Filters
    {
        [Serializable]
        public class LowPass
        {
            [SerializeField, Range(0.0f, 0.999f), Tooltip("0: No smoothing, 1: Full Smoothing")]
            private float _smoothing;

            [SerializeField, ReadOnly] private float _value;

            public float Value => _value;
            public float Smoothing => _smoothing;
            private bool _initialized;

            public LowPass(float smoothing = 0.75f) => _smoothing = smoothing;

            public void Reset(float? value = null)
            {
                if (value.HasValue) _value = value.Value;
                else _initialized = false;
            }

            public float Filter(float value, float? smoothing = null)
            {
                if (smoothing.HasValue) _smoothing = smoothing.Value;
                if (!_initialized)
                {
                    _initialized = true;
                    _value = value;
                }

                return _value = (1 - _smoothing) * value + _smoothing * _value;
            }
        }
    }
}
