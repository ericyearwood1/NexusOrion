using System;
using System.Threading.Tasks;
using ARGlasses.Interaction;
using UnityEngine;
using OSIG.Tools.StateMachines;
using ProtoKit.UI;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    public class SliderStateVisuals : ViewStateVisuals<ISlider, SliderView>
    {
        [SerializeField] private int _scrollSoundDebounceMs = 100;

        [FormerlySerializedAs("_machine")]
        [SerializeField]
        [RequireStates(PKUIUtils.DEFAULT_NORMAL, PKUIUtils.DEFAULT_HOVERED, PKUIUtils.DEFAULT_PRESSED,
            PKUIUtils.DEFAULT_SELECTED)]
        private StateMachine _colorMachine = new StateMachine();

        public override StateMachine ColorMachine => _colorMachine;
        public override StateMachine MotionMachine { get; }


        [SerializeField] private StateMapping _stateMapping = new StateMapping()
        {
            Normal = PKUIUtils.DEFAULT_NORMAL,
            Hovered = PKUIUtils.DEFAULT_HOVERED,
            Pressed = PKUIUtils.DEFAULT_PRESSED,
            Selected = PKUIUtils.DEFAULT_SELECTED
        };

        private float _lastSoundValueNormalized;

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


            //todo bindings here
            switch (_view.Style.SliderType)
            {
                case SliderType.Number:
                    break;
                case SliderType.Icon:
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        public override void DestroyStateMachine()
        {
            if (_colorMachine.IsCreated) _colorMachine.Destroy();
        }

        private bool _pressed;

        protected override void HandleStateChanged(TargetState state)
        {
            if (HasColorMachineOverride())
            {
                _viewStateVisualsOverride.ColorMachineOverride.TransitionToState(state.IsHover()
                    ?
                    _stateMapping.Hovered
                    :
                    state.IsPress()
                        ? _stateMapping.Pressed
                        : _stateMapping.Normal);
            }
            else
            {
                _colorMachine.TransitionToState(state.IsHover() ? _stateMapping.Hovered :
                    state.IsPress() ? _stateMapping.Pressed : _stateMapping.Normal);
            }

            switch (state)
            {
                case TargetState.Normal:
                    if (_pressed)
                    {
                        _pressed = false;
                        SoundController.Instance.PlayDriftOut();
                    }

                    break;
                case TargetState.Hover:
                    if (_pressed)
                    {
                        _pressed = false;
                        SoundController.Instance.PlayDriftOut();
                    }

                    break;
                case TargetState.Press:
                    SoundController.Instance.PlaySelected();
                    _pressed = true;
                    break;
                case TargetState.Drift:
                case TargetState.Disabled:
                    break;
            }
        }

        protected override void HandleValueNormalizedChanged(float valueNormalized)
        {
            //todo use inertia to change pitch for some cool sound design
            //PlaySoundDebounced(valueNormalized);
        }

        public override void UpdateViewData(SliderView view)
        {
        }

        private bool _playing;

        private async void PlaySoundDebounced(float valueNormalized)
        {
            if (_playing) return;

            _playing = true;
            var clip = valueNormalized > _lastSoundValueNormalized
                ? SoundController.ESoundClip.ScrollUp
                : SoundController.ESoundClip.ScrollDown;
            _lastSoundValueNormalized = valueNormalized;

            var pitch = _colorMachine.CurrentState == _stateMapping.Pressed ? 1 : .7f;
            SoundController.Instance.Play(clip, pitch: pitch);
            await Task.Delay(_scrollSoundDebounceMs);
            _playing = false;
        }
    }
}
