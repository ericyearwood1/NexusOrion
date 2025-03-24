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
        public class ExpButtonStateVisuals : ViewStateVisuals<ButtonModel, ExpButtonView>
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

            /// <summary>
            /// need to save this info if we are in a locked/selection state so we can revert
            /// </summary>
            private string _lastStateChangeRecorded;

            [SerializeField] private StateMapping _stateMapping = new StateMapping()
            {
                Normal = PKUIUtils.DEFAULT_NORMAL,
                Hovered = PKUIUtils.DEFAULT_HOVERED,
                Pressed = PKUIUtils.DEFAULT_PRESSED,
                Selected = PKUIUtils.DEFAULT_SELECTED
            };

            public StateMapping StateMapping
            {
                get => _stateMapping;
                set => _stateMapping = value;
            }

            private bool _selected;

            public override void UpdateViewData(ExpButtonView view)
            {
                var newValue = _model != null ? _model.Value : false;

                var stateName = _stateMapping.Normal;

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


            public override void SetModelAndView(ButtonModel button, ExpButtonView view)
            {
                if (_model != null)
                {
                    _model.WhenStateChanged -= HandleStateChanged;
                    _model.WhenClicked -= HandleClicked;
                    _model.WhenCancelled -= HandleCancelled;
                }

                _model = button;
                _view = view;

                if (_model != null)
                {
                    _model.WhenStateChanged += HandleStateChanged;
                    _model.WhenClicked += HandleClicked;
                    _model.WhenCancelled += HandleCancelled;
                }
            }

            public override void Select(bool on)
            {
                _selected = on;

                if (_selected)
                {
                    if (HasColorMachineOverride())
                    {
                        if (_viewStateVisualsOverride.ColorMachineOverride.IsCreated)
                            _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(_stateMapping.Selected);
                    }
                    else
                    {
                        if (_colorMachine.IsCreated)
                            _colorMachine.TransitionToState(_stateMapping.Selected);
                    }

                    if (HasMotionMachineOverride())
                    {
                        if (_viewStateVisualsOverride.MotionMachineOverride.IsCreated)
                            _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(_stateMapping.Selected);
                    }
                    else
                    {
                        if (_motionMachine.IsCreated)
                            _motionMachine.TransitionToState(_stateMapping.Selected);
                    }
                }
                else
                {
                    if (HasColorMachineOverride())
                    {
                        if (_viewStateVisualsOverride.ColorMachineOverride.IsCreated)
                            _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(_lastStateChangeRecorded);
                    }
                    else
                    {
                        if (_colorMachine.IsCreated)
                            _colorMachine.TransitionToState(_lastStateChangeRecorded);
                    }

                    if (HasMotionMachineOverride())
                    {
                        if (_viewStateVisualsOverride.MotionMachineOverride.IsCreated)
                            _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(_lastStateChangeRecorded);
                    }
                    else
                    {
                        if (_motionMachine.IsCreated)
                            _motionMachine.TransitionToState(_lastStateChangeRecorded);
                    }
                }
            }


            private void HandleCancelled()
            {
                // todo this should move to a sound system
                SoundController.Instance.PlayCancel();
            }

            private void HandleClicked()
            {
                // todo this should move to a sound system
                SoundController.Instance.PlaySelected();
            }

            private void HandleStateChanged(TargetState state)
            {
                _lastStateChangeRecorded = state.IsHover() ? _stateMapping.Hovered :
                    state.IsPress() ? _stateMapping.Pressed : _stateMapping.Normal;

                //with experience buttons, select means experience is loading. button interactions should be disabled
                if (_selected)
                {
                    return;
                }

                if (HasColorMachineOverride())
                {
                    if (_viewStateVisualsOverride.ColorMachineOverride.IsCreated)
                        _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(_lastStateChangeRecorded);
                }
                else
                {
                    if (_colorMachine.IsCreated)
                        _colorMachine.TransitionToState(_lastStateChangeRecorded);
                }

                if (HasColorMachineOverride())
                {
                    if (_viewStateVisualsOverride.MotionMachineOverride.IsCreated)
                        _viewStateVisualsOverride.MotionMachineOverride.TransitionToState(_lastStateChangeRecorded);
                }
                else
                {
                    if (_motionMachine.IsCreated)
                        _motionMachine.TransitionToState(_lastStateChangeRecorded);
                }
            }

            public override void InitializeStateMachine()
            {
                DestroyStateMachine();
                if (!_view) return;


                if (HasColorMachineOverride())
                {
                    _viewStateVisualsOverride.ColorMachineOverride.Create(this, _stateMapping.Normal);
                    _viewStateVisualsOverride.ColorMachineOverride.Bind<Color>("BGColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));
                    _viewStateVisualsOverride.ColorMachineOverride.Bind<float>("ProgressAlpha",
                        f => _view.RadialProgressBar.CanvasGroupAlpha = f);
                    _viewStateVisualsOverride.ColorMachineOverride.Bind<float>("OverlayProportionalHeight",
                        f => _view.OverlayProvider.ProportionalLayout.SetHeightMax(f));
                }
                else
                {
                    _colorMachine.Create(this, _stateMapping.Normal);
                    _colorMachine.Bind<Color>("BGColor",
                        color => _view.BackgroundProvider.BackgroundRenderer.SetColorA(color));
                    _colorMachine.Bind<float>("ProgressAlpha", f => _view.RadialProgressBar.CanvasGroupAlpha = f);
                    _colorMachine.Bind<float>("OverlayProportionalHeight",
                        f => _view.OverlayProvider.ProportionalLayout.SetHeightMax(f));
                }

                if (HasMotionMachineOverride())
                {
                    _viewStateVisualsOverride.MotionMachineOverride.Create(this, _stateMapping.Normal);
                    _viewStateVisualsOverride.MotionMachineOverride.Bind<Vector2>("IconSize",
                        vector2 => _view.IconPanelProvider.IconLayoutSize.SetSize2D(vector2));
                }
                else
                {
                    _motionMachine.Create(this, _stateMapping.Normal);
                    _motionMachine.Bind<Vector2>("IconSize",
                        vector2 => _view.IconPanelProvider.IconLayoutSize.SetSize2D(vector2));
                }
            }

            public override void DestroyStateMachine()
            {
                if (_colorMachine.IsCreated) _colorMachine.Destroy();
                if (HasColorMachineOverride() && _viewStateVisualsOverride.ColorMachineOverride.IsCreated)
                    _viewStateVisualsOverride.ColorMachineOverride.Destroy();

                if (_motionMachine.IsCreated) _motionMachine.Destroy();
                if (HasMotionMachineOverride() && _viewStateVisualsOverride.MotionMachineOverride.IsCreated)
                    _viewStateVisualsOverride.MotionMachineOverride.Destroy();
            }
        }
    }
}
