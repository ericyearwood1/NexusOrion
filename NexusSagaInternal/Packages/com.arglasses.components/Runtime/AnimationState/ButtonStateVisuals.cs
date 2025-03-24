using System;
using ARGlasses.Interaction;
using UnityEngine;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    public class ButtonStateVisuals : ViewStateVisuals<ButtonModel, ButtonView>
    {
        [FormerlySerializedAs("_machine")]
        [SerializeField]
        [RequireStates(PKUIUtils.DEFAULT_NORMAL, PKUIUtils.DEFAULT_HOVERED, PKUIUtils.DEFAULT_PRESSED,
            PKUIUtils.DEFAULT_SELECTED)]
        private StateMachine _colorMachine = new StateMachine();

        [SerializeField]
        [RequireStates(PKUIUtils.DEFAULT_NORMAL, PKUIUtils.DEFAULT_HOVERED, PKUIUtils.DEFAULT_PRESSED,
            PKUIUtils.DEFAULT_SELECTED)]
        private StateMachine _motionMachine = new StateMachine();

        public override StateMachine ColorMachine => _colorMachine;
        public override StateMachine MotionMachine => _motionMachine;

        [SerializeField] private StateMapping _stateMapping = new StateMapping()
        {
            Normal = PKUIUtils.DEFAULT_NORMAL,
            Hovered = PKUIUtils.DEFAULT_HOVERED,
            Pressed = PKUIUtils.DEFAULT_PRESSED,
            Selected = PKUIUtils.DEFAULT_SELECTED
        };

        public override void UpdateViewData(ButtonView view)
        {
            if (_colorMachine.Definition == null)
                return;


            if (HasColorMachineOverride())
            {
                if (view.BackgroundProvider != null)
                {
                    if (TryExtractComponentData<ColorVariableComponent, Color>(
                            _viewStateVisualsOverride.ColorMachineOverride, "BGColor", "Normal",
                            out var color))
                    {
                        view.BackgroundProvider.BackgroundRenderer.SetColorA(color);
                    }
                }

                if (view.OverlayProvider != null)
                {
                    if (TryExtractComponentData<ColorVariableComponent, Color>(
                            _viewStateVisualsOverride.ColorMachineOverride, "AppOverlayColor", "Normal",
                            out var color))
                    {
                        view.OverlayProvider.OverlayPanel.SetColorA(color);
                    }
                }
            }
            else
            {
                if (view.BackgroundProvider != null)
                {
                    if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "BGColor", "Normal",
                            out var color))
                    {
                        view.BackgroundProvider.BackgroundRenderer.SetColorA(color);
                    }
                }

                if (view.OverlayProvider != null)
                {
                    if (TryExtractComponentData<ColorVariableComponent, Color>(_colorMachine, "AppOverlayColor",
                            "Normal",
                            out var color))
                    {
                        view.OverlayProvider.OverlayPanel.SetColorA(color);
                    }
                }
            }
        }

        public override void InitializeStateMachine()
        {
            DestroyStateMachine();

            if (!_view) return;

            if (HasColorMachineOverride())
            {
                _viewStateVisualsOverride.ColorMachineOverride.Create(this, _stateMapping.Normal);
                switch (_view.Style.ButtonType)
                {
                    case ButtonType.Text:
                    case ButtonType.Round:
                        _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("BGColor",
                            color => { _view.BackgroundProvider.BackgroundRenderer.SetColorA(color); });
                        break;
                    case ButtonType.App:
                        _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("AppOverlayColor",
                            color => _view.OverlayProvider.OverlayPanel.SetColorA(color));
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }
            else
            {
                _colorMachine.Create(this, _stateMapping.Normal);
                switch (_view.Style.ButtonType)
                {
                    case ButtonType.Text:
                    case ButtonType.Round:
                        _colorMachine.Bind<Color>("BGColor",
                            color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));
                        break;
                    case ButtonType.App:
                        _colorMachine.Bind<Color>("AppOverlayColor",
                            color => _view.OverlayProvider.OverlayPanel.SetColorA(color));
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }
        }

        public override void DestroyStateMachine()
        {
            if (_colorMachine.IsCreated) _colorMachine.Destroy();
            if (HasColorMachineOverride() && _viewStateVisualsOverride.ColorMachineOverride.IsCreated)
                _viewStateVisualsOverride.ColorMachineOverride.Destroy();
        }

        protected override void HandleCancelled()
        {
            // todo this should move to a sound system
            SoundController.Instance.PlayCancel();
        }

        protected override  void HandleClicked()
        {
            // todo this should move to a sound system
            SoundController.Instance.PlaySelected();
        }

        private string ToStateString(TargetState state) => state.IsHover() ? _stateMapping.Hovered : state.IsPress() ? _stateMapping.Pressed : _stateMapping.Normal;

        protected override  void HandleStateChanged(TargetState state)
        {
            var stateString = ToStateString(state);

            if (HasColorMachineOverride()) _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(stateString);
            else if(_colorMachine.IsCreated) _colorMachine.TransitionToState(stateString);

            if (HasMotionMachineOverride()) _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(stateString);
            else if(_motionMachine.IsCreated) _motionMachine.TransitionToState(stateString);

            if (_view.IconPanelProvider != null) _view.IconPanelProvider.TransitionToState(stateString);
        }

        void OnDestroy()
        {
            DestroyStateMachine();
            SetModelAndView(null, null);
        }
    }
}
