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
    public class LauncherButtonView : MonoBehaviour , IArgView<LauncherButtonViewModel, ButtonModel>
    {
        [FormerlySerializedAs("_absolutePositionProvider")] [SerializeField] private DriftProvider _driftProvider;
        [SerializeField] private LabelProvider _labelProvider;
        [SerializeField] private ArgIcon _appImagePanelProvider;
        [SerializeField] private BackgroundProvider _backgroundProvider;
        [SerializeField] private ViewStateVisuals<ButtonModel, LauncherButtonView> _stateVisuals;
        public DriftProvider DriftProvider => _driftProvider;
        public LabelProvider LabelProvider => _labelProvider;
        public BackgroundProvider BackgroundProvider => _backgroundProvider;
        public ViewStateVisuals<ButtonModel, LauncherButtonView>  StateVisuals => _stateVisuals;


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


        public void UpdateViewData(LauncherButtonViewModel viewModelBase)
        {
            if (_labelProvider != null)
                _labelProvider.SetLabel(viewModelBase.LabelText);
            if (_appImagePanelProvider != null) _appImagePanelProvider.IconRenderer.Sprite = viewModelBase.AppImage;
            if (_stateVisuals != null)
            {
                _stateVisuals.UpdateViewData(this);
            }
        }

        public void Select(bool on)
        {

        }

        public bool HasLabel => _labelProvider != null;

        public bool HasAppImage => _appImagePanelProvider != null;
    }
}
