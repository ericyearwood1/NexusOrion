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
    public class TileButtonView : MonoBehaviour, IArgView<TileButtonViewModel, ButtonModel>
    {
        [SerializeField] private TileButtonStyle _style;
        public TileButtonStyle Style => _style;
        [FormerlySerializedAs("_absolutePositionProvider")] [SerializeField] private DriftProvider _driftProvider;
        public DriftProvider DriftProvider => _driftProvider;
        [SerializeField] private LabelProvider _labelProvider;
        public LabelProvider LabelProvider => _labelProvider;
        [SerializeField] private OverlayProvider _overlayProvider;
        [SerializeField] private LabelProvider _descriptionProvider;
        public LabelProvider Descriptionprovider => _descriptionProvider;
        public OverlayProvider OverlayProvider => _overlayProvider;
        [SerializeField] private ArgIcon _iconPanelProvider;
        public ArgIcon IconPanelProvider => _iconPanelProvider;
        [SerializeField] private BackgroundProvider _backgroundProvider;
        public BackgroundProvider BackgroundProvider => _backgroundProvider;
        [SerializeField] private ViewStateVisuals<ButtonModel, TileButtonView>  _stateVisuals;
        public ViewStateVisuals<ButtonModel, TileButtonView> StateVisuals => _stateVisuals;

        public Action<bool> SelectChanged;
        public bool Value;


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


        private void DisplacementLocalChanged(Vector3 displacementLocal)
        {
            _driftProvider.MoveAbsolute(displacementLocal);
        }

        public void UpdateViewData(TileButtonViewModel viewModel)
        {
            Value = viewModel.Value;
            if (_labelProvider != null)
                _labelProvider.SetLabel(viewModel.LabelText);
            if (_descriptionProvider != null)
                _descriptionProvider.SetLabel(viewModel.DescriptionText);
            if (_iconPanelProvider != null) _iconPanelProvider.IconRenderer.Sprite = viewModel.IconSprite;

            if (_stateVisuals != null)
            {
                _stateVisuals.UpdateViewData(this);
            }
        }

        public void Select(bool on)
        {
            Value = on;
            if (!Toggleable)
                return;
            SelectChanged?.Invoke(on);
        }

        public bool HasLabel => true;
        public bool HasDescription => _descriptionProvider != null;
        public bool HasIcon => _iconPanelProvider != null;
        public bool Toggleable => true;
    }
}
