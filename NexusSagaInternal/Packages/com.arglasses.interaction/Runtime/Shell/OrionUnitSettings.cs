// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using OSIG.Tools.Units;
using UnityEngine;
#if UNITY_EDITOR
using OSIG.Tools.Utility;
using UnityEditor;
#endif

namespace ARGlasses.Interaction
{
    public class OrionUnitSettings : OCUnitsSettings
    {
        [SerializeField, ReadOnly] private float _orionPixelsPerMeter = 1100;
        [SerializeField, ReadOnly] private float _orionPixelsPerDegree = 32;
        [SerializeField, ReadOnly] private float _orionViewDistance = 1.1f;
        [SerializeField] private float _scaleOverride = 1f;

        public float ScaleOverride
        {
            get => _scaleOverride;
            set
            {
                _scaleOverride = Mathf.Max(value, 0.1f);
                OnValidate();
            }
        }

        public override OCPixelsToMetersConversion? PixelConversion => OCPixelsToMetersConversion.PixelsPerDegree;
        public override float? PixelsPerMeter => _orionPixelsPerMeter / ScaleOverride;
        public override float? PixelsPerDegree => _orionPixelsPerDegree / ScaleOverride;
        public override float? ViewDistance => _orionViewDistance;


#if UNITY_EDITOR
        [CanEditMultipleObjects]
        [CustomEditor(typeof(OrionUnitSettings), true)]
        public class E : EditorBase
        {
            protected override void OnEnable()
            {
                Hide(PixelConversionName);
                Hide(PixelsPerMeterName);
                Hide(PixelsPerDegreeName);
                Hide(ViewDistanceName);
                Hide($"{PixelConversionName}Override");
                Hide($"{PixelsPerMeterName}Override");
                Hide($"{PixelsPerDegreeName}Override");
                Hide($"{ViewDistanceName}Override");
                base.OnEnable();
            }
        }
#endif
    }
}
