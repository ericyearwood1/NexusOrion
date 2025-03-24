using System;
using System.Linq;
using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.StateMachines;
using OSIG.Tools.Units;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;
using UnityEngine.Serialization;


namespace ARGlasses.Components
{
    /// <summary>
    /// this should be the sole interface between view controller and view
    /// tells controller which style this view applies. helpful for finding
    /// out whether or not there are icons/labels to render
    /// also provides access to specific view components and methods to set data
    /// </summary>
    public class ExpButtonView : MonoBehaviour, IArgView<ExpButtonViewModel, ButtonModel>
    {
        [FormerlySerializedAs("_absolutePositionProvider")] [SerializeField] private DriftProvider _driftProvider;
        public DriftProvider DriftProvider => _driftProvider;

        [SerializeField] private LabelProvider _labelProvider;
        public LabelProvider LabelProvider => _labelProvider;

        [SerializeField] private OverlayProvider _overlayProvider;
        public OverlayProvider OverlayProvider => _overlayProvider;

        [SerializeField] private ArgIcon _iconPanelProvider;
        public ArgIcon IconPanelProvider => _iconPanelProvider;

        [SerializeField] private BackgroundProvider _backgroundProvider;
        public BackgroundProvider BackgroundProvider => _backgroundProvider;

        [SerializeField] private RadialProgressBar _radialProgressBar;
        public RadialProgressBar RadialProgressBar => _radialProgressBar;

        [SerializeField] private ViewStateVisuals<ButtonModel, ExpButtonView> _stateVisuals;
        public ViewStateVisuals<ButtonModel,ExpButtonView> StateVisuals => _stateVisuals;

        private ButtonModel _button;

        public void Initialize(ButtonModel button)
        {
            if (_button != null)
            {
                _button.WhenDisplacementLocalChanged -= DisplacementLocalChanged;
            }

            _button = button;
            if (_button != null)
            {
                _button.WhenDisplacementLocalChanged += DisplacementLocalChanged;
            }

            _stateVisuals.SetModelAndView(button, this);
            _stateVisuals.InitializeStateMachine();
        }

        private void DisplacementLocalChanged(Vector3 drift)
        {
            _driftProvider.MoveAbsolute(drift);
        }

        public void Select(bool on)
        {
            _stateVisuals.Select(on);
        }

        public void UpdateViewData(ExpButtonViewModel viewModel)
        {
            if (_labelProvider != null)
                _labelProvider.SetLabel(viewModel.LabelText);
            if (_iconPanelProvider != null) _iconPanelProvider.IconRenderer.Sprite = viewModel.Icon;

            if (_stateVisuals != null)
            {
                _stateVisuals.UpdateViewData(this);
            }
        }
    }
}
