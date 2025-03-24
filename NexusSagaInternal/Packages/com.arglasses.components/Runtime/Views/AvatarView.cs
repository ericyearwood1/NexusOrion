using ARGlasses.Interaction;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;


namespace ARGlasses.Components
{
    /// <summary>
    /// this should be the sole interface between view controller and view
    /// tells controller which style this view applies. helpful for finding
    /// out whether or not there are icons/labels to render
    /// also provides access to specific view components and methods to set data
    /// </summary>
    public class AvatarView : MonoBehaviour, IArgView<AvatarViewModel, ButtonModel>
    {
        [SerializeField]
        [RequireStates(PKUIUtils.DEFAULT_NORMAL, PKUIUtils.DEFAULT_HOVERED, PKUIUtils.DEFAULT_PRESSED,
            PKUIUtils.DEFAULT_SELECTED)]
        private StateMachine _motionMachine = new StateMachine();

        [SerializeField]
        [RequireStates(PKUIUtils.DEFAULT_NORMAL, PKUIUtils.DEFAULT_HOVERED, PKUIUtils.DEFAULT_PRESSED,
            PKUIUtils.DEFAULT_SELECTED)]
        private StateMachine _colorMachine = new StateMachine();

        [SerializeField] private StateMapping _stateMapping = new StateMapping()
        {
            Normal = PKUIUtils.DEFAULT_NORMAL,
            Hovered = PKUIUtils.DEFAULT_HOVERED,
            Pressed = PKUIUtils.DEFAULT_PRESSED,
            Selected = PKUIUtils.DEFAULT_SELECTED
        };

        [SerializeField] private PKUIPanel _avatarPanel;
        [SerializeField] private PKUIPanel _appImagePanel;
        [SerializeField] private PKUIPanel _indicatorPanel;
        [SerializeField] private PKUIText _labelText;
        [SerializeField] private PKUIText _counterText;
        [SerializeField] private AvatarStyle _avatarStyle;


        private bool _showLabelOnHover;
        private Material instantiatedMat;

        private ButtonModel _button;
        private static readonly int ImageScale = Shader.PropertyToID("_ImageScale");

        public void Initialize(ButtonModel button)
        {
            _button = button;
            _button.WhenStateChanged += HandleStateChanged;

            instantiatedMat = new Material(_avatarPanel.material);
            _avatarPanel.material = instantiatedMat;
            _motionMachine.Create(this, _stateMapping.Normal);
            _colorMachine.Create(this, _stateMapping.Normal);
            _motionMachine.Bind<float>("AvatarScale", f => { _avatarPanel.material.SetFloat(ImageScale, f); });
            if (_labelText != null)
            {
                _colorMachine.Bind<Color>("LabelColor", color => _labelText.color = color);
            }
        }

        private void HandleStateChanged(TargetState state)
        {
            var stateString = state.IsHover() ? _stateMapping.Hovered :
                state.IsPress() ? _stateMapping.Hovered : _stateMapping.Normal;
            _motionMachine.TransitionToState(stateString);
            _colorMachine.TransitionToState(stateString);
        }

        public void UpdateViewData(AvatarViewModel viewModel)
        {
            _showLabelOnHover = viewModel.ShowLabelOnHover;

            _avatarPanel.Sprite = viewModel.AvatarImage;
            _appImagePanel.Sprite = viewModel.AppImage;
            if (_labelText != null)
            {
                _labelText.gameObject.SetActive(_showLabelOnHover);
                _labelText.text = viewModel.LabelText;
            }
            if (_indicatorPanel != null)
                _indicatorPanel.SetColorA(viewModel.IndicatorColor);
            if (_counterText != null)
            {
                _counterText.text = viewModel.CounterText;
            }
        }

        public void Select(bool on)
        {
            throw new System.NotImplementedException();
        }


        private void OnDestroy()
        {
            if (_button != null)
                _button.WhenStateChanged -= HandleStateChanged;

            if(_colorMachine.IsCreated)
                _colorMachine.Destroy();
            if(_motionMachine.IsCreated)
                _motionMachine.Destroy();

            if (instantiatedMat != null)
                Destroy(instantiatedMat);
        }
    }
}
