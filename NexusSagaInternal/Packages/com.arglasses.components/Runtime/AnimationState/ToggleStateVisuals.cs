using System;
using System.Linq;
using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using UnityEngine;
using UnityEngine.Assertions;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    namespace ProtoKit.UI
    {
        public class ToggleStateVisuals : ViewStateVisuals<IToggle, ToggleView>
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

            [SerializeField]
            [RequireStates(PKUIUtils.DEFAULT_NORMAL_OFF, PKUIUtils.DEFAULT_HOVERED_OFF, PKUIUtils.DEFAULT_PRESSED_OFF,
                PKUIUtils.DEFAULT_SELECTED_OFF,
                PKUIUtils.DEFAULT_NORMAL_ON, PKUIUtils.DEFAULT_HOVERED_ON, PKUIUtils.DEFAULT_PRESSED_ON,
                PKUIUtils.DEFAULT_SELECTED_ON)]
            private StateMachine _motionMachine = new StateMachine();

            public override StateMachine MotionMachine => _motionMachine;


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

            private StateMapping _currentMapping => _model.Value ? _stateMappingOn : _stateMappingOff;

            private bool _value;

            public override void UpdateViewData(ToggleView view)
            {
                _value = _model != null ? _model.Value : view.Value;
                var handlePosVarName = view.Style.ToggleType == ToggleType.Normal
                    ? "HandlePos"
                    : "HandlePosMini";
                var stateName = _value ? _stateMappingOn.Normal : _stateMappingOff.Normal;

                if (HasMotionMachineOverride())
                {
                    if (TryExtractComponentData<FloatVariableComponent, float>(
                            _viewStateVisualsOverride.MotionMachineOverride, handlePosVarName,
                            stateName,
                            out var handlePosition))
                    {
                        view.HandleProvider.HandleAttachData.SetPaddingLeft(handlePosition);
                    }
                }
                else
                {
                    if (TryExtractComponentData<FloatVariableComponent, float>(_motionMachine, handlePosVarName,
                            stateName,
                            out var handlePosition))
                    {
                        view.HandleProvider.HandleAttachData.SetPaddingLeft(handlePosition);
                    }
                }

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

                //OCLayoutSingletonDriver.ForceUpdateNowForAllLayoutInScene();
            }

            private DragCancelState _dragCancelState;
            private bool _pressedActive;
            private bool _valueWhenFirstPress;

            protected override void HandleDragStateChanged(DragCancelState dragCancelState)
            {
                _dragCancelState = dragCancelState;
            }

            protected override void HandleStateChanged(TargetState state)
            {
                if (state.IsPress())
                {
                    _valueWhenFirstPress = _value;
                    _pressedActive = true;
                    _dragCancelState = DragCancelState.In;
                }
                else if (state.IsNormalOrHover() && _pressedActive)
                {
                    _pressedActive = false;
                    //only cancel if we didn't swipe to a new value
                    if (_dragCancelState == DragCancelState.Out && _valueWhenFirstPress == _value)
                    {
                        OnCancel();
                    }
                }

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

                if (HasMotionMachineOverride())
                {
                    _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(state.IsHover()
                        ? _currentMapping.Hovered
                        : state.IsPress()
                            ? _currentMapping.Pressed
                            : _currentMapping.Normal);
                }
                else
                {
                    _motionMachine.TransitionToState(state.IsHover() ? _currentMapping.Hovered :
                        state.IsPress() ? _currentMapping.Pressed : _currentMapping.Normal);
                }
            }

            private void OnCancel()
            {
                SoundController.Instance.PlayCancel();
            }

            protected override void HandleToggleValueChanged(bool val)
            {
                _value = val;
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

                if (HasMotionMachineOverride())
                {
                    var motionState =
                        _viewStateVisualsOverride.MotionMachineOverride.DestinationState.Replace(oldValue, newValue);
                    _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(motionState);
                }
                else
                {
                    var motionState = _motionMachine.DestinationState.Replace(oldValue, newValue);
                    _motionMachine.TransitionToState(motionState);
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
                    _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("HandleColor",
                        color => _view.HandleProvider.HandleRenderer.SetColorA(color));
                }
                else
                {
                    _colorMachine.Create(this, _currentMapping.Normal);
                    _colorMachine.Bind<Color>("BGColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));
                    _colorMachine.Bind<Color>("HandleColor",
                        color => _view.HandleProvider.HandleRenderer.SetColorA(color));
                }

                if (HasMotionMachineOverride())
                {
                    _viewStateVisualsOverride.MotionMachineOverride.Create(this, _currentMapping.Normal);
                    _viewStateVisualsOverride.MotionMachineOverride.Bind<float>(
                        _view.Style.ToggleType == ToggleType.Normal ? "HandlePos" : "HandlePosMini",
                        f => _view.HandleProvider.HandleAttachData.SetPaddingLeft(f));
                }
                else
                {
                    _motionMachine.Create(this, _currentMapping.Normal);
                    _motionMachine.Bind<float>(
                        _view.Style.ToggleType == ToggleType.Normal ? "HandlePos" : "HandlePosMini",
                        f => _view.HandleProvider.HandleAttachData.SetPaddingLeft(f));
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

                if (HasMotionMachineOverride())
                {
                    if (_viewStateVisualsOverride.MotionMachineOverride.IsCreated)
                        _viewStateVisualsOverride.MotionMachineOverride.Destroy();
                }
                else
                {
                    if (_motionMachine.IsCreated) _motionMachine.Destroy();
                }
            }
        }
    }
}
