using OSIG.Tools.Units;
using UnityEditor;
using UnityEngine;
using OSIG.Tools.Utility;

namespace ARGlasses.Interaction
{
    public class ArtemisUnitSettings : OCUnitsSettings
    {
        public const float DefaultPixelsPerMeter = 1100;
        public const float DefaultPixelsPerDegree = 31;
        public const float DefaultViewDistance = 1.1f;

        [SerializeField, ReadOnly] private float _artemisPixelsPerMeter = DefaultPixelsPerMeter;
        [SerializeField, ReadOnly] private float _artemisPixelsPerDegree = DefaultPixelsPerDegree;
        [SerializeField, ReadOnly] private float _artemisViewDistance = DefaultViewDistance;
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
        public override float? PixelsPerMeter => _artemisPixelsPerMeter / ScaleOverride;
        public override float? PixelsPerDegree => _artemisPixelsPerDegree / ScaleOverride;
        public override float? ViewDistance => _artemisViewDistance;


#if UNITY_EDITOR
        [CanEditMultipleObjects]
        [CustomEditor(typeof(ArtemisUnitSettings), true)]
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
