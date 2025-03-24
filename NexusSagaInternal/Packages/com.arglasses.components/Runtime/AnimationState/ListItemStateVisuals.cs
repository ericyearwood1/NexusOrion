using ARGlasses.Interaction;
using UnityEngine;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    namespace ProtoKit.UI
    {
        public class ListItemStateVisuals : ViewStateVisuals<ButtonModel, ListItemView>
        {

            [FormerlySerializedAs("_machine")]
            [SerializeField]
            [RequireStates(PKUIUtils.DEFAULT_NORMAL_OFF, PKUIUtils.DEFAULT_HOVERED_OFF, PKUIUtils.DEFAULT_PRESSED_OFF,
                PKUIUtils.DEFAULT_SELECTED_OFF,
                PKUIUtils.DEFAULT_NORMAL_ON, PKUIUtils.DEFAULT_HOVERED_ON, PKUIUtils.DEFAULT_PRESSED_ON,
                PKUIUtils.DEFAULT_SELECTED_ON)]
            private StateMachine _colorMachine = new StateMachine();

            [SerializeField]
            [RequireStates(PKUIUtils.DEFAULT_NORMAL_OFF, PKUIUtils.DEFAULT_HOVERED_OFF, PKUIUtils.DEFAULT_PRESSED_OFF,
                PKUIUtils.DEFAULT_SELECTED_OFF,
                PKUIUtils.DEFAULT_NORMAL_ON, PKUIUtils.DEFAULT_HOVERED_ON, PKUIUtils.DEFAULT_PRESSED_ON,
                PKUIUtils.DEFAULT_SELECTED_ON)]
            private StateMachine _motionMachine = new StateMachine();

            public override StateMachine ColorMachine => _colorMachine;

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

            private StateMapping _currentMapping => _view.Value ? _stateMappingOn : _stateMappingOff;


            public override void UpdateViewData(ListItemView view)
            {
                _view = view;

                var newValue = view.Value;
                var stateName = newValue ? _stateMappingOn.Normal : _stateMappingOff.Normal;

                if (HasColorMachineOverride())
                {
                    if (view.BackgroundProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(
                                _viewStateVisualsOverride.ColorMachineOverride, "BGColor", stateName,
                                out var bgColor))
                        {
                            view.BackgroundProvider.BackgroundRenderer.SetColorA(bgColor);
                        }
                    }

                    if (view.CheckFillProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(
                                _viewStateVisualsOverride.ColorMachineOverride, "CheckFill",
                                stateName,
                                out var bgColor))
                        {
                            view.CheckFillProvider.BackgroundRenderer.SetColorA(bgColor);
                        }
                    }

                    if (view.ChevronProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(
                                _viewStateVisualsOverride.ColorMachineOverride, "ChevronColor",
                                stateName,
                                out var color))
                        {
                            view.ChevronProvider.IconRenderer.SetColorA(color);
                        }
                    }

                    if (view.MediaPanelProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(
                                _viewStateVisualsOverride.ColorMachineOverride, "AvatarAndMediaColor",
                                stateName,
                                out var color))
                        {
                            view.MediaPanelProvider.IconRenderer.SetColorA(color);
                        }
                    }

                    if (view.AvatarPanelprovider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(
                                _viewStateVisualsOverride.ColorMachineOverride, "AvatarAndMediaColor",
                                stateName,
                                out var color))
                        {
                            view.AvatarPanelprovider.IconRenderer.SetColorA(color);
                        }
                    }
                }
                else
                {
                    if (view.BackgroundProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "BGColor", stateName,
                                out var bgColor))
                        {
                            view.BackgroundProvider.BackgroundRenderer.SetColorA(bgColor);
                        }
                    }

                    if (view.CheckFillProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "CheckFill",
                                stateName,
                                out var bgColor))
                        {
                            view.CheckFillProvider.BackgroundRenderer.SetColorA(bgColor);
                        }
                    }

                    if (view.ChevronProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "ChevronColor",
                                stateName,
                                out var color))
                        {
                            view.ChevronProvider.IconRenderer.SetColorA(color);
                        }
                    }

                    if (view.MediaPanelProvider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "AvatarAndMediaColor",
                                stateName,
                                out var color))
                        {
                            view.MediaPanelProvider.IconRenderer.SetColorA(color);
                        }
                    }

                    if (view.AvatarPanelprovider != null)
                    {
                        if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "AvatarAndMediaColor",
                                stateName,
                                out var color))
                        {
                            view.AvatarPanelprovider.IconRenderer.SetColorA(color);
                        }
                    }
                }


                if (HasMotionMachineOverride())
                {
                    if (view.ChevronProvider != null)
                    {
                        if (TryExtractComponentData<FloatVariableComponent, float>(
                                _viewStateVisualsOverride.MotionMachineOverride, "ChevronPos",
                                stateName,
                                out var pos))
                        {
                            view.ChevronProvider.AttachData.SetPaddingRight(pos);
                        }
                    }

                    if (view.LabelProvider != null)
                    {
                        if (TryExtractComponentData<FloatVariableComponent, float>(
                                _viewStateVisualsOverride.MotionMachineOverride, "LabelSize",
                                stateName,
                                out var size))
                        {
                            view.LabelProvider.SetSize(size);
                        }
                    }
                }

                else
                {
                    if (view.ChevronProvider != null)
                    {
                        if (TryExtractComponentData<FloatVariableComponent, float>(_motionMachine, "ChevronPos",
                                stateName,
                                out var pos))
                        {
                            view.ChevronProvider.AttachData.SetPaddingRight(pos);
                        }
                    }

                    if (view.LabelProvider != null)
                    {
                        if (TryExtractComponentData<FloatVariableComponent, float>(_motionMachine, "LabelSize",
                                stateName,
                                out var size))
                        {
                            view.LabelProvider.SetSize(size);
                        }
                    }
                }
            }

            protected override void HandleStateChanged(TargetState state)
            {
                var stateString = state.IsHover() ? _currentMapping.Hovered :
                    state.IsPress() ? _currentMapping.Pressed : _currentMapping.Normal;

                if (HasColorMachineOverride())
                {
                    _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(stateString);
                }
                else
                {
                    _colorMachine.TransitionToState(stateString);
                }

                if (HasMotionMachineOverride())
                {
                    _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(stateString);
                }
                else
                {
                    _motionMachine.TransitionToState(stateString);
                }


                if (_view.IconPanelProvider != null)
                {
                    _view.IconPanelProvider.TransitionToState(stateString);
                }
            }

            protected override void HandleToggleValueChanged(bool val)
            {
                var oldValue = val ? "_Off" : "_On";
                var newValue = val ? "_On" : "_Off";
                var colorState = _colorMachine.DestinationState.Replace(oldValue, newValue);
                if (HasColorMachineOverride())
                {
                    _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(colorState);
                }
                else
                {
                    _colorMachine.TransitionToState(colorState);
                }

                if (HasMotionMachineOverride())
                {
                    _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(colorState);
                }
                else
                {
                    _motionMachine.TransitionToState(colorState);
                }
            }

            public override void InitializeStateMachine()
            {
                DestroyStateMachine();

                if (!_view)
                    return;

                if (HasColorMachineOverride())
                {
                    _viewStateVisualsOverride.ColorMachineOverride.Create(this, _currentMapping.Normal);
                    _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("BGColor",
                        color => { _view.BackgroundProvider.BackgroundRenderer.SetColorA(color); });

                    if (_view.CheckFillProvider != null)
                    {
                        _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("CheckFill",
                            color => _view.CheckFillProvider.BackgroundRenderer.SetColorA(color));
                    }

                    if (_view.ChevronProvider != null)
                    {
                        _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("ChevronColor",
                            color => _view.ChevronProvider.IconRenderer.SetColorA(color));
                    }

                    if (_view.MediaPanelProvider != null)
                    {
                        _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("AvatarAndMediaColor",
                            color => _view.MediaPanelProvider.IconRenderer.SetColorA(color));
                    }

                    if (_view.AvatarPanelprovider != null)
                    {
                        _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("AvatarAndMediaColor",
                            color => _view.AvatarPanelprovider.IconRenderer.SetColorA(color));
                    }
                }
                else
                {
                    _colorMachine.Create(this, _currentMapping.Normal);
                    _colorMachine.Bind<Color>("BGColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));

                    if (_view.CheckFillProvider != null)
                    {
                        _colorMachine.Bind<Color>("CheckFill",
                            color => _view.CheckFillProvider.BackgroundRenderer.SetColorA(color));
                    }

                    if (_view.ChevronProvider != null)
                    {
                        _colorMachine.Bind<Color>("ChevronColor",
                            color => _view.ChevronProvider.IconRenderer.SetColorA(color));
                    }

                    if (_view.MediaPanelProvider != null)
                    {
                        _colorMachine.Bind<Color>("AvatarAndMediaColor",
                            color => _view.MediaPanelProvider.IconRenderer.SetColorA(color));
                    }

                    if (_view.AvatarPanelprovider != null)
                    {
                        _colorMachine.Bind<Color>("AvatarAndMediaColor",
                            color => _view.AvatarPanelprovider.IconRenderer.SetColorA(color));
                    }
                }

                if (HasMotionMachineOverride())
                {
                    _viewStateVisualsOverride.MotionMachineOverride.Create(this, _currentMapping.Normal);

                    if (_view.ChevronProvider != null)
                    {
                        _viewStateVisualsOverride.MotionMachineOverride.Bind<float>("ChevronPos",
                            f => _view.ChevronProvider.AttachData.SetPaddingRight(f));
                    }

                    if (_view.MediaPlayIconProvider != null)
                    {
                        _viewStateVisualsOverride.MotionMachineOverride.Bind<float>("PlayIconPos",
                            f => _view.MediaPlayIconProvider.AttachData.SetPaddingLeft(f));
                    }

                    if (_view.LabelProvider != null)
                    {
                        _viewStateVisualsOverride.MotionMachineOverride.Bind<float>("LabelSize",
                            f => { _view.LabelProvider.SetSize(f); });
                    }
                }
                else
                {
                    _motionMachine.Create(this, _currentMapping.Normal);

                    if (_view.ChevronProvider != null)
                    {
                        _motionMachine.Bind<float>("ChevronPos",
                            f => _view.ChevronProvider.AttachData.SetPaddingRight(f));
                    }

                    if (_view.MediaPlayIconProvider != null)
                    {
                        _motionMachine.Bind<float>("PlayIconPos",
                            f => _view.MediaPlayIconProvider.AttachData.SetPaddingLeft(f));
                    }

                    if (_view.LabelProvider != null)
                    {
                        _motionMachine.Bind<float>("LabelSize", f => { _view.LabelProvider.SetSize(f); });
                    }
                }
            }

            public override void DestroyStateMachine()
            {
                if (_colorMachine.IsCreated)
                {
                    _colorMachine.Destroy();
                }

                if (_motionMachine.IsCreated)
                {
                    _motionMachine.Destroy();
                }


                if (HasColorMachineOverride())
                {
                    if (_viewStateVisualsOverride.ColorMachineOverride.IsCreated)
                    {
                        _viewStateVisualsOverride.ColorMachineOverride.Destroy();
                    }
                }


                if (HasMotionMachineOverride())
                {
                    if (_viewStateVisualsOverride.MotionMachineOverride.IsCreated)
                    {
                        _viewStateVisualsOverride.MotionMachineOverride.Destroy();
                    }
                }
            }
        }
    }
}
