using GDPX;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ArtemisDisplaySimulatorSettingsCheck : MonoBehaviour
    {
        [SerializeField] private float _editorMaskOpacity = 0.9f;
        [SerializeField, ReadOnly] private ARDisplaySimulator _displaySimulator;
        private float Ppd => 31;
        private bool enablePPD => true;
        private float HorizontalFOV => 40;
        private float VerticalFOV => 30;
        private float ZDepthMeters => 1.5f;
        private float VerticalFOVBias => -4;
        private float HorizontalFOVBias => 0;
        private float FOVBlend => 2;
        private float BrightnessScale => 1;
        private ARDisplaySimulator.EyeRenderMode EyeMode => ARDisplaySimulator.EyeRenderMode.Both;

        private void Awake() => this.Sibling(ref _displaySimulator);

        private void OnEnable()
        {
            if (Application.isEditor) _displaySimulator.Opacity = _editorMaskOpacity;

            CheckValue(_displaySimulator.HorizontalFOV, HorizontalFOV);
            CheckValue(_displaySimulator.VerticalFOV, VerticalFOV);
            CheckValue(_displaySimulator.ZDepth, ZDepthMeters);
            CheckValue(_displaySimulator.VerticalFOVBias, VerticalFOVBias);
            CheckValue(_displaySimulator.HorizontalFOVBias, HorizontalFOVBias);
            CheckValue(_displaySimulator.FOVBlend, FOVBlend);
            CheckValue(_displaySimulator.EyeMode, EyeMode);
            CheckValue(_displaySimulator.BrightnessScale, BrightnessScale);
            CheckValue(_displaySimulator.SimulatedPixelPerDegree, Ppd);
            CheckValue(_displaySimulator.PerformPPDBasedRenderScale, enablePPD);
        }

        private void CheckValue<T>(T expected, T actual)
        {
            if (!Equals(expected, actual)) Debug.LogError(Message);
        }

        private const string Message = "ARDisplaySimulator settings don't match the expected values for Artemis (feel free to disable ArtemisDisplaySimulatorSettingsCheck if you don't care about that)";

        private void ForceSettings()
        {
            _displaySimulator.PerformPPDBasedRenderScale = enablePPD;
            _displaySimulator.SimulatedPixelPerDegree = Ppd;
            _displaySimulator.HorizontalFOV = HorizontalFOV;
            _displaySimulator.VerticalFOV = VerticalFOV;
            _displaySimulator.ZDepth = ZDepthMeters;
            _displaySimulator.VerticalFOVBias = VerticalFOVBias;
            _displaySimulator.HorizontalFOVBias = HorizontalFOVBias;
            _displaySimulator.FOVBlend = FOVBlend;
            _displaySimulator.EyeMode = EyeMode;
            _displaySimulator.BrightnessScale = BrightnessScale;
            // if (_hide) _displaySimulator.EyeMode = ARDisplaySimulator.EyeRenderMode.None;
        }
    }
}
