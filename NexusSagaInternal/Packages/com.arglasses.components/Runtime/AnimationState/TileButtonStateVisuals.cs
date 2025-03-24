using ARGlasses.Interaction;
using UnityEngine;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    namespace ProtoKit.UI
    {
        public class TileButtonStateVisuals : ViewStateVisuals<ButtonModel, TileButtonView>
        {
            public const string PROP_MACHINE = nameof(_colorMachine);
            public const string PROP_STATE_MAPPING_OFF = nameof(_stateMappingOff);
            public const string PROP_STATE_MAPPING_ON = nameof(_stateMappingOn);

            [FormerlySerializedAs("_machine")]
            [SerializeField]
            [RequireStates(PKUIUtils.DEFAULT_NORMAL_OFF, PKUIUtils.DEFAULT_HOVERED_OFF, PKUIUtils.DEFAULT_PRESSED_OFF,
                PKUIUtils.DEFAULT_SELECTED_OFF,
                PKUIUtils.DEFAULT_NORMAL_ON, PKUIUtils.DEFAULT_HOVERED_ON, PKUIUtils.DEFAULT_PRESSED_ON,
                PKUIUtils.DEFAULT_SELECTED_ON)]
            private StateMachine _colorMachine = new StateMachine();

            public override StateMachine ColorMachine => _colorMachine;

            public override StateMachine MotionMachine => null;


            [SerializeField]
            [Tooltip("Can be used to remap the names of the states this component uses if you are provided your own " +
                     "StateMachine with different state names.")]
            private StateMapping _stateMappingOff = new StateMapping()
            {
                Normal = PKUIUtils.DEFAULT_NORMAL_OFF,
                Hovered = PKUIUtils.DEFAULT_HOVERED_OFF,
                Pressed = PKUIUtils.DEFAULT_PRESSED_OFF,
                Selected = PKUIUtils.DEFAULT_SELECTED_OFF
            };

            public StateMapping StateMappingOff
            {
                get => _stateMappingOff;
                set => _stateMappingOff = value;
            }

            [SerializeField]
            [Tooltip("Can be used to remap the names of the states this component uses if you are provided your own " +
                     "StateMachine with different state names.")]
            private StateMapping _stateMappingOn = new StateMapping()
            {
                Normal = PKUIUtils.DEFAULT_NORMAL_ON,
                Hovered = PKUIUtils.DEFAULT_HOVERED_ON,
                Pressed = PKUIUtils.DEFAULT_PRESSED_ON,
                Selected = PKUIUtils.DEFAULT_SELECTED_ON
            };

            public StateMapping StateMappingOn
            {
                get => _stateMappingOn;
                set => _stateMappingOn = value;
            }

            private StateMapping _currentMapping => _view.Value ? _stateMappingOn : _stateMappingOff;


            public override void UpdateViewData(TileButtonView view)
            {
                _view = view;

                var newValue = view.Value;
                var stateName = newValue ? _stateMappingOn.Normal : _stateMappingOff.Normal;

                if (view.BackgroundProvider != null)
                {
                    if (HasColorMachineOverride())
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(
                                _viewStateVisualsOverride.ColorMachineOverride, "BGColor", stateName,
                                out var bgColor))
                        {
                            view.BackgroundProvider.BackgroundRenderer.SetColorA(bgColor);
                        }
                    }
                    else
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "BGColor", stateName,
                                out var bgColor))
                        {
                            view.BackgroundProvider.BackgroundRenderer.SetColorA(bgColor);
                        }
                    }
                }
            }

            protected override void HandleCancelled()
            {
                SoundController.Instance.PlayCancel();
            }

            protected override  void HandleClicked()
            {
                SoundController.Instance.PlaySelected();
            }

            protected override  void HandleStateChanged(TargetState state)
            {
                if (HasColorMachineOverride())
                {
                    _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(state.IsHover()
                        ? _currentMapping.Hovered
                        : state.IsPress()
                            ? _currentMapping.Pressed
                            : _currentMapping.Normal);
                }
                else
                {
                    _colorMachine.TransitionToState(state.IsHover() ? _currentMapping.Hovered :
                        state.IsPress() ? _currentMapping.Pressed : _currentMapping.Normal);
                }
            }

            protected override void HandleToggleValueChanged(bool val)
            {
                var oldValue = val ? "_Off" : "_On";
                var newValue = val ? "_On" : "_Off";

                if (HasColorMachineOverride())
                {
                    var colorState =
                        _viewStateVisualsOverride.ColorMachineOverride.DestinationState.Replace(oldValue, newValue);
                    _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(colorState);
                }
                else
                {
                    var colorState = _colorMachine.DestinationState.Replace(oldValue, newValue);
                    _colorMachine.TransitionToState(colorState);
                }


                SoundController.Instance.Play(val
                    ? SoundController.ESoundClip.ToggleOn
                    : SoundController.ESoundClip.ToggleOff);
            }

            public override void InitializeStateMachine()
            {
                DestroyStateMachine();
                if (!_view) return;

                if (HasColorMachineOverride())
                {
                    _viewStateVisualsOverride.ColorMachineOverride.Create(this, _currentMapping.Normal);
                    _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("BGColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));
                }
                else
                {
                    _colorMachine.Create(this, _currentMapping.Normal);
                    _colorMachine.Bind<Color>("BGColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));
                }
            }

            public override void DestroyStateMachine()
            {
                if (HasColorMachineOverride())
                {
                    if (_viewStateVisualsOverride.ColorMachineOverride.IsCreated)
                        _viewStateVisualsOverride.ColorMachineOverride.Destroy();
                }
                else
                {
                    if (_colorMachine.IsCreated) _colorMachine.Destroy();
                }
            }
        }
    }
}
