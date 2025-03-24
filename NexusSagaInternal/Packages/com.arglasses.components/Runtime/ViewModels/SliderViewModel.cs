using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class SliderViewModel : ViewModelBase
    {
        [FormerlySerializedAs("Style")]
        [SerializeField]
        private SliderStyle _style;

        [FormerlySerializedAs("IconSprite")]
        [SerializeField]
        private Sprite _iconSprite;

        [FormerlySerializedAs("MinValue")]
        [SerializeField]
        private float _minValue;

        [FormerlySerializedAs("MaxValue")]
        [SerializeField]
        private float _maxValue;

        [FormerlySerializedAs("Interval")]
        [SerializeField]
        private float _interval;

        [FormerlySerializedAs("Value")]
        [SerializeField]
        private float _value;

        [FormerlySerializedAs("NormalizedValue")]
        [SerializeField]
        private float _normalizedValue;

        public SliderStyle Style
        {
            get => _style;
            set => SetField(ref _style, value);
        }

        public Sprite IconSprite
        {
            get => _iconSprite;
            set => SetField(ref _iconSprite, value);
        }

        public float MinValue
        {
            get => _minValue;
            set => SetField(ref _minValue, value);
        }

        public float MaxValue
        {
            get => _maxValue;
            set => SetField(ref _maxValue, value);
        }

        public float Interval
        {
            get => _interval;
            set => SetField(ref _interval, value);
        }

        public float Value
        {
            get => _value;
            set
            {
                if (SetField(ref _value, ClampValue(value)))
                {
                    NormalizedValue = Mathf.InverseLerp(MinValue, MaxValue, _value);
                }
            }
        }

        private float ClampValue(float value)
        {
            return Mathf.Clamp(value, MinValue, MaxValue);
        }

        public float NormalizedValue
        {
            get => _normalizedValue;
            private set => SetField(ref _normalizedValue, value);
        }
    }
}
