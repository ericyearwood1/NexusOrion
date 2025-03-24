using ARGlasses.Interaction;
using ITK;
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgSlider : ViewController<SliderViewModel, SliderModel, SliderView>
    {
        public UnityEvent<float> WhenValueChanged = new UnityEvent<float>();
        public UnityEvent<float> WhenValueNormalizedChanged = new UnityEvent<float>();

        [Group("Slider Events")] public UnityEvent WhenHandleGrabbed = new UnityEvent();

        [Group("Slider Events")] public UnityEvent WhenHandleReleased = new UnityEvent();


        [SerializeField, ReadOnly] private bool _isGrabbed;


        private void Awake()
        {
            InitializeComponents(ensure: true);
            InteractionModel.WhenValueNormalizedChanged += HandleValueNormalizedChanged;
            InteractionModel.WhenStateChanged += HandleStateChanged;
            InteractionModel.ValueNormalized =
                Mathf.InverseLerp(ViewModel.MinValue, ViewModel.MaxValue, ViewModel.Value);
            ForceUpdateView();
            View.Initialize(InteractionModel);
        }

        private void HandleValueNormalizedChanged(float valueNormalized)
        {
            WhenValueNormalizedChanged.Invoke(valueNormalized);
            ViewModel.Value = ClampValue(Mathf.Lerp(ViewModel.MinValue, ViewModel.MaxValue, valueNormalized));
            WhenValueChanged.Invoke(ViewModel.Value);
        }

        public float ClampValue(float value)
        {
            if (ViewModel.Interval == 0)
            {
                return Mathf.Clamp(value, ViewModel.MinValue, ViewModel.MaxValue);
            }
            else
            {
                return Mathf.Round(Mathf.Clamp(value, ViewModel.MinValue, ViewModel.MaxValue) / ViewModel.Interval) *
                       ViewModel.Interval;
            }
        }

        private void HandleStateChanged(TargetState state)
        {
            if (!_isGrabbed && state.IsPress())
            {
                _isGrabbed = true;
                WhenHandleGrabbed.Invoke();
            }

            if (_isGrabbed && !state.IsPress())
            {
                _isGrabbed = false;
                WhenHandleReleased.Invoke();
            }
        }
    }
}
