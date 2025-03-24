using ARGlasses.Interaction;
using UnityEngine;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    public class LauncherButtonStateVisuals : ViewStateVisuals<IButton, LauncherButtonView>
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

        public override void UpdateViewData(LauncherButtonView view)
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
            }
            else

            {
                _colorMachine.Create(this, _stateMapping.Normal);
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


        protected override void HandleCancelled()
        {
            // todo this should move to a sound system
            SoundController.Instance.PlayCancel();
        }

        protected override void HandleClicked()
        {
            // todo this should move to a sound system
            SoundController.Instance.PlaySelected();
        }

        protected override void HandleStateChanged(TargetState state)
        {
            var stateString = state.IsHover() ? _stateMapping.Hovered :
                state.IsPress() ? _stateMapping.Pressed : _stateMapping.Normal;

            if (HasColorMachineOverride())
            {
                _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(stateString);
            }
            else
            {
                _colorMachine.TransitionToState(stateString);
            }
        }
    }
}
