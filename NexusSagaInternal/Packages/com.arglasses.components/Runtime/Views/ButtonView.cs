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
    public class ButtonView : MonoBehaviour , IArgView<ButtonViewModel, ButtonModel>
    {
        [SerializeField] private ButtonStyle _style;
        [SerializeField] private DriftProvider _driftProvider;
        [SerializeField] private LabelProvider _labelProvider;
        [SerializeField] private OverlayProvider _overlayProvider;
        [SerializeField] private LabelProvider _descriptionProvider;
        [SerializeField] private ArgIcon _iconPanelProvider;
        [SerializeField] private ArgIcon _appImagePanelProvider;
        [SerializeField] private BackgroundProvider _backgroundProvider;
        [SerializeField] private ViewStateVisuals<ButtonModel, ButtonView> _stateVisuals;
        public LabelProvider DescriptionProvider => _descriptionProvider;
        public ButtonStyle Style => _style;
        public DriftProvider DriftProvider => _driftProvider;
        public LabelProvider LabelProvider => _labelProvider;
        public OverlayProvider OverlayProvider => _overlayProvider;
        public ArgIcon IconPanelProvider => _iconPanelProvider;
        public BackgroundProvider BackgroundProvider => _backgroundProvider;
        public ViewStateVisuals<ButtonModel, ButtonView> StateVisuals => _stateVisuals;
        
        //todo remove state from view
        public Action<bool> SelectChanged;
        public bool Value;

        [SerializeField] private Use _use;

        private ButtonModel _button;
        private ButtonViewModel _viewModel;
        
        
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
        
        public void UpdateViewData(ButtonViewModel viewModelBase)
        {
            _viewModel = viewModelBase;
            
            Value = viewModelBase.Selected;
            if (_labelProvider != null)
                _labelProvider.SetLabel(viewModelBase.LabelText);
            if (_descriptionProvider != null)
                _descriptionProvider.SetLabel(viewModelBase.DescText);
            if (_iconPanelProvider != null) _iconPanelProvider.SetIcon(viewModelBase.Icon);
            if (_appImagePanelProvider != null) _appImagePanelProvider.IconRenderer.Sprite = viewModelBase.AppImage;
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
            
            if(_iconPanelProvider) _iconPanelProvider.SetIcon(on ? _viewModel.OnIcon : _viewModel.OffIcon);
        }

        public void SetUse(Use use, StateMachineDefinition smd)
        {
            _use = use;
            _stateVisuals.ColorMachine.Definition = smd;
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
            EditorUtility.SetDirty(_stateVisuals);
#endif
            _stateVisuals.UpdateViewData(this);
        }

        public void SetStateful(bool stateful)
        {
            _style.Stateful = stateful;
        }
        
        public bool HasLabel => _labelProvider != null;
        public bool HasDescription => _descriptionProvider != null;
        public bool HasIcon => _iconPanelProvider != null;

        public bool HasAppImage => _appImagePanelProvider != null;
        public bool HadBg => _style.ButtonType != ButtonType.App;
        public bool 
            Toggleable => _use == Use.Segmented || _use == Use.FlatSegmented || _use == Use.Toggle || _style.Stateful;
    }
}
