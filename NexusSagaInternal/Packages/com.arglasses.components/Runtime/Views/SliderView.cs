using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using OSIG.Tools.Units;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    public class SliderView : MonoBehaviour, IArgView<SliderViewModel, SliderModel>
    {
        [FormerlySerializedAs("_absolutePositionProvider")] [SerializeField]
        private DriftProvider _driftProvider;

        public DriftProvider DriftProvider => _driftProvider;
        [SerializeField] private OCLayoutSlider _fillAndHandleProvider;
        public OCLayoutSlider FillAndHandleProvider => _fillAndHandleProvider;

        [SerializeField] private BackgroundProvider _backgroundProvider;
        public BackgroundProvider BackgroundProvider => _backgroundProvider;

        [SerializeField] private ArgIcon _iconPanelProvider;
        public ArgIcon IconPanelProvider => _iconPanelProvider;

        [SerializeField] private LabelProvider _labelProvider;
        public LabelProvider LabelProvider => _labelProvider;

        [SerializeField] private SliderStateVisuals _stateVisuals;

        //todo inject
        [SerializeField] private SliderStyle _style;
        public SliderStyle Style => _style;

        private SliderModel _sliderModel;
        private float _minValue;
        private float _maxValue;
        private float _interval;

        public void Initialize(SliderModel sliderModel)
        {
            if (_sliderModel != null)
            {
                _sliderModel.WhenValueNormalizedChanged -= HandleValueNormalizedChanged;
                _sliderModel.WhenDisplacementLocalChanged -= DisplacementLocalChanged;
            }

            _sliderModel = sliderModel;

            if (_sliderModel != null)
            {
                UpdateFillAmount(_sliderModel.ValueNormalized);
                _sliderModel.WhenValueNormalizedChanged += HandleValueNormalizedChanged;
                _sliderModel.WhenDisplacementLocalChanged += DisplacementLocalChanged;
            }

            _stateVisuals.SetModelAndView(_sliderModel, this);
            _stateVisuals.InitializeStateMachine();
        }


        private void DisplacementLocalChanged(Vector3 displacement)
        {
            _driftProvider.MoveAbsolute(displacement);
        }

        // public void UpdateViewData(Sprite iconSprite = null, float normalizedValue = 0, float minValue = 0,
        //     float maxValue = 1, float actualValue = 0, float interval = 0)
        public void UpdateViewData(SliderViewModel viewModel)
        {
            _minValue = viewModel.MinValue;
            _maxValue = viewModel.MaxValue;
            _interval = viewModel.Interval;

            if (_style.SliderType == SliderType.Number && _labelProvider != null)
                _labelProvider.SetLabel(_sliderModel != null
                    ? $"{_sliderModel.Value:F1}"
                    : $"{(viewModel.Value % 1 == 0 ? viewModel.Value : viewModel.Value.ToString("F1"))}");

            if (_style.SliderType == SliderType.Icon && _iconPanelProvider != null)
                _iconPanelProvider.IconRenderer.Sprite = viewModel.IconSprite;

            UpdateFillAmount(_sliderModel != null ? _sliderModel.ValueNormalized : viewModel.NormalizedValue);
            _stateVisuals.UpdateViewData(this);
        }

        public void Select(bool on)
        {
            throw new System.NotImplementedException();
        }

        public void HandleValueNormalizedChanged(float normalizedValue)
        {
            var actualValue = ClampValue(Mathf.Lerp(_minValue, _maxValue, normalizedValue));

            UpdateFillAmount(normalizedValue);

            if (_style.SliderType == SliderType.Number && _labelProvider != null)
                _labelProvider.SetLabel($"{(actualValue % 1 == 0 ? actualValue : actualValue.ToString("F1"))}");
        }

        private void UpdateFillAmount(float normalizedValue)
        {
            _fillAndHandleProvider.Value = normalizedValue;
            _fillAndHandleProvider.SetDirty();
        }

        private float ClampValue(float value)
        {
            if (_interval == 0)
            {
                return value;
            }
            else
            {
                return Mathf.Round(value / _interval) * _interval;
            }
        }
    }
}
