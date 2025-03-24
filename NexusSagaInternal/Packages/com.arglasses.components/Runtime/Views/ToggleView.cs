using ARGlasses.Components.ProtoKit.UI;
using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    public class ToggleView : MonoBehaviour, IArgView<ToggleViewModel, ToggleSliderModel>
    {
        [FormerlySerializedAs("_absolutePositionProvider")] [SerializeField] private DriftProvider _driftProvider;
        public DriftProvider DriftProvider => _driftProvider;
        [SerializeField] private ToggleHandleProvider _handleProvider;
        public ToggleHandleProvider HandleProvider => _handleProvider;

        [SerializeField] private BackgroundProvider _backgroundProvider;
        public BackgroundProvider BackgroundProvider => _backgroundProvider;

        [SerializeField] private ToggleStateVisuals _stateVisuals;

        [SerializeField] private ToggleStyle _style;
        public ToggleStyle Style => _style;

        private ToggleSliderModel _toggle;
        private bool _value;

        public bool Value => _value;

        public void Initialize(ToggleSliderModel toggle)
        {
            if (_toggle != null)
            {
                _toggle.WhenDisplacementLocalChanged -= DisplacementLocalChanged;
            }
            _toggle = toggle;
            if (_toggle != null)
            {
                _toggle.WhenDisplacementLocalChanged += DisplacementLocalChanged;
            }

            _stateVisuals.SetModelAndView(_toggle, this);
            _stateVisuals.InitializeStateMachine();
        }

        private void DisplacementLocalChanged(Vector3 displacementLocal)
        {
            _driftProvider.MoveAbsolute(displacementLocal);
        }

        public void UpdateViewData(ToggleViewModel viewModel)
        {
            _value = viewModel.Value;
            _stateVisuals.UpdateViewData(this);
        }

        public void Select(bool on)
        {
            throw new System.NotImplementedException();
        }
    }
}
