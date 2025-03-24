using ARGlasses.Interaction;
using UnityEngine;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    namespace ProtoKit.UI
    {
        public class SegmentedButtonStateVisuals : ViewStateVisuals<ButtonModel, ButtonView>
        {
            [FormerlySerializedAs("_machine")]
            [SerializeField]
            [RequireStates(PKUIUtils.DEFAULT_NORMAL_OFF, PKUIUtils.DEFAULT_HOVERED_OFF, PKUIUtils.DEFAULT_PRESSED_OFF,
                PKUIUtils.DEFAULT_SELECTED_OFF,
                PKUIUtils.DEFAULT_NORMAL_ON, PKUIUtils.DEFAULT_HOVERED_ON, PKUIUtils.DEFAULT_PRESSED_ON,
                PKUIUtils.DEFAULT_SELECTED_ON)]
            private StateMachine _colorMachine = new StateMachine();

            public override StateMachine ColorMachine => _colorMachine;

            [SerializeField]
            [RequireStates(PKUIUtils.DEFAULT_NORMAL_OFF, PKUIUtils.DEFAULT_HOVERED_OFF, PKUIUtils.DEFAULT_PRESSED_OFF,
                PKUIUtils.DEFAULT_SELECTED_OFF,
                PKUIUtils.DEFAULT_NORMAL_ON, PKUIUtils.DEFAULT_HOVERED_ON, PKUIUtils.DEFAULT_PRESSED_ON,
                PKUIUtils.DEFAULT_SELECTED_ON)]
            private StateMachine _motionMachine = new StateMachine();

            public override StateMachine MotionMachine => _motionMachine;


            [SerializeField]
            [Tooltip("Can be used to remap the names of the states this component uses if you provided your own " +
                     "StateMachine with different state names.")]
            private StateMapping _stateMappingOff = new StateMapping()
            {
                Normal = PKUIUtils.DEFAULT_NORMAL_OFF,
                Hovered = PKUIUtils.DEFAULT_HOVERED_OFF,
                Pressed = PKUIUtils.DEFAULT_PRESSED_OFF,
                Selected = PKUIUtils.DEFAULT_SELECTED_OFF
            };

            [SerializeField]
            [Tooltip("Can be used to remap the names of the states this component uses if you provided your own " +
                     "StateMachine with different state names.")]
            private StateMapping _stateMappingOn = new StateMapping()
            {
                Normal = PKUIUtils.DEFAULT_NORMAL_ON,
                Hovered = PKUIUtils.DEFAULT_HOVERED_ON,
                Pressed = PKUIUtils.DEFAULT_PRESSED_ON,
                Selected = PKUIUtils.DEFAULT_SELECTED_ON
            };

            private StateMapping _currentMapping => _view.Value ? _stateMappingOn : _stateMappingOff;


            public override void UpdateViewData(ButtonView view)
            {
                _view = view;

                var newValue = _model != null ? _model.Value : view.Value;
                var stateName = newValue ? _stateMappingOn.Normal : _stateMappingOff.Normal;
                
                if (_colorMachine.Definition == null)
                    return;

                if (TryExtractComponentData<FloatVariableComponent, float>(_colorMachine, "BorderWidth", stateName,
                        out var borderWidth))
                {
                    if (view.BackgroundProvider != null)
                        view.BackgroundProvider.BackgroundRenderer.SetBorder(borderWidth);
                    if (view.OverlayProvider != null)
                        view.OverlayProvider.OverlayPanel.SetBorder(borderWidth);
                }

                if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "BorderColor", stateName,
                        out var borderColor))
                {
                    if (view.BackgroundProvider != null)
                        view.BackgroundProvider.BackgroundRenderer.SetBorderColor(borderColor);
                    if (view.OverlayProvider != null)
                        view.OverlayProvider.OverlayPanel.SetBorderColor(borderColor);
                }

                if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "BGColor", stateName,
                        out var bgColor))
                {
                    if (view.BackgroundProvider != null)
                        view.BackgroundProvider.BackgroundRenderer.SetColorA(bgColor);
                }

                if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "AppOverlayColor", stateName,
                        out var overlayColor))
                {
                    if (view.OverlayProvider != null)
                        view.OverlayProvider.OverlayPanel.SetColorA(overlayColor);
                }
            }


            [SerializeField] private bool _useSounds = true;

            protected override void HandleCancelled()
            {
                if (_useSounds) SoundController.Instance.PlayCancel();
            }

            protected override void HandleClicked()
            {
                if (_useSounds) SoundController.Instance.PlaySelected();
            }

            protected override void HandleStateChanged(TargetState state)
            {
                _colorMachine.TransitionToState(state.IsHover() ? _currentMapping.Hovered :
                    state.IsPress() ? _currentMapping.Pressed : _currentMapping.Normal);
                _motionMachine.TransitionToState(state.IsHover() ? _currentMapping.Hovered :
                    state.IsPress() ? _currentMapping.Pressed : _currentMapping.Normal);
            }

            protected override void HandleToggleValueChanged(bool val)
            {
                var oldValue = val ? "_Off" : "_On";
                var newValue = val ? "_On" : "_Off";
                var colorState = _colorMachine.DestinationState.Replace(oldValue, newValue);
                var motionState = _motionMachine.DestinationState.Replace(oldValue, newValue);
                _colorMachine.TransitionToState(colorState);
                _motionMachine.TransitionToState(motionState);
                if (_useSounds)
                    SoundController.Instance.Play(val
                        ? SoundController.ESoundClip.ToggleOn
                        : SoundController.ESoundClip.ToggleOff);
            }

            public override void InitializeStateMachine()
            {
                DestroyStateMachine();
                if (!_view) return;

                _colorMachine.Create(this, _currentMapping.Normal);
                _motionMachine.Create(this, _currentMapping.Normal);

                if (_view.BackgroundProvider != null)
                {
                    _colorMachine.Bind<Color>("BGColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));
                    _colorMachine.Bind<Color>("BorderColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetBorderColor(color));
                    _colorMachine.Bind<float>("BorderWidth",
                        width => _view.BackgroundProvider.BackgroundRenderer.SetBorder(width));
                }

                if (_view.OverlayProvider != null)
                {
                    _colorMachine.Bind<Color>("AppOverlayColor",
                        color => _view.OverlayProvider.OverlayPanel.SetColorA(color));
                    _colorMachine.Bind<Color>("BorderColor",
                        color => _view.OverlayProvider.OverlayPanel.SetBorderColor(color));
                    _colorMachine.Bind<float>("BorderWidth",
                        width => _view.OverlayProvider.OverlayPanel.SetBorder(width));
                }
            }

            public override void DestroyStateMachine()
            {
                if (_colorMachine.IsCreated) _colorMachine.Destroy();
                if (_motionMachine.IsCreated) _motionMachine.Destroy();
            }
        }
    }
}
