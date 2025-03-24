using System;
using System.Linq;
using ARGlasses.Interaction;
using Cysharp.Threading.Tasks;
#if DOTWEEN_ENABLED
using DG.Tweening;
#endif
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
    public class ListItemView : MonoBehaviour, IArgView<ListItemViewModel, ButtonModel>
    {
        [SerializeField] private ListItemStyle _style;
        public ListItemStyle Style => _style;

        [FormerlySerializedAs("_absolutePositionProvider")] [SerializeField]
        private DriftProvider _driftProvider;

        [SerializeField] private LabelProvider _labelProvider;
        [SerializeField] private OverlayProvider _overlayProvider;
        [SerializeField] private LabelProvider _descriptionProvider;
        [SerializeField] private LabelProvider _rightTextProvider;
        [SerializeField] private ArgIcon _iconPanelProvider;
        [SerializeField] private ArgIcon _mediaPlayIconProvider;
        [SerializeField] private ArgIcon _mediaPanelProvider;
        [SerializeField] private ArgIcon _avatarPanelProvider;
        [SerializeField] private ArgIcon _chevronProvider;
        [SerializeField] private BackgroundProvider _backgroundProvider;
        [SerializeField] private BackgroundProvider _checkFillProvider;
        [SerializeField] private ViewStateVisuals<ButtonModel, ListItemView> _stateVisuals;
        [SerializeField] private ArgToggle _toggle;
        [SerializeField] private CanvasGroup _canvasGroup;
        [SerializeField] private AnimationCurve _scrollInMoveCurve;
        [SerializeField] private AnimationCurve _scrollOutMoveCurve;
        [SerializeField] private AnimationCurve _scrollInFadeCurve;
        [SerializeField] private AnimationCurve _scrollOutFadeCurve;
        [SerializeField] private Gradient _scrollInGradient;

        public ArgIcon ChevronProvider => _chevronProvider;

        public BackgroundProvider BackgroundProvider => _backgroundProvider;
        public BackgroundProvider CheckFillProvider => _checkFillProvider;
        public ArgIcon MediaPlayIconProvider => _mediaPlayIconProvider;
        public ArgIcon MediaPanelProvider => _mediaPanelProvider;
        public ArgIcon AvatarPanelprovider => _avatarPanelProvider;
        public ArgIcon IconPanelProvider => _iconPanelProvider;
        public LabelProvider LabelProvider => _labelProvider;

        public ArgToggle Toggle => _toggle;
        public bool HasLabel => true;
        public bool HasDescription => true;
        public bool HasIcon => _style.ListItemType is ListItemType.Icon or ListItemType.IconWithToggle;
        public bool HasMedia => _style.ListItemType is ListItemType.Media;
        public bool HasAvatar => _style.ListItemType is ListItemType.Avatar;

        public bool Toggleable =>
            _style.ListItemType is ListItemType.Checklist or ListItemType.IconWithToggle or ListItemType.TextWithToggle;

        public bool HasRightText => _style.ListItemType is ListItemType.EnumeratedTextWithRightText;

        //todo remove state from view
        public Action<bool> SelectChanged;
        public bool Value;


        private ButtonModel _button;
        private ListItemViewModel _viewModel;

#if DOTWEEN_ENABLED
        private Sequence _scrollFadeInFromBottomOfViewport;
        private Sequence _scrollFadeOutFromBottomOfViewport;
        private Sequence _scrollFadeInFromTopOfViewport;
        private Sequence _scrollFadeOutFromTopOfViewport;
        private VisibilityState _lastVisibilityState;
        private bool _previousVisibilityState;

        public struct AnimationParams
        {
            public float InitPixelPaddingBottom;
            public float InitPixelPaddingFront;
            public float InitCanvasGroupAlpha;
            public float FinalCanvasGroupAlpha;
            public float FinalPixelPaddingBottom;
            public float FinalPixelPaddingFront;
            public float Duration;
            public bool InAnimation;
            public float GradientDuration;
        }

        private bool init = false;
        private void Start()
        {
            DOTween.SetTweensCapacity(5000,5000);

            // Common duration
            float duration = .6f;
            float gradientDuration = .65f;

            // Initialize the animation parameters
            AnimationParams fadeInFromBottomParams = new AnimationParams
            {
                InitPixelPaddingBottom = 100,
                InitPixelPaddingFront = 200,
                InitCanvasGroupAlpha = 0,
                FinalCanvasGroupAlpha = 1,
                FinalPixelPaddingBottom = 0,
                FinalPixelPaddingFront = 0,
                Duration = duration,
                GradientDuration = gradientDuration,
                InAnimation = true
            };


            AnimationParams fadeOutFromBottomParams = new AnimationParams
            {
                InitPixelPaddingBottom = 0,
                InitPixelPaddingFront = 0,
                InitCanvasGroupAlpha = 1,
                FinalCanvasGroupAlpha = 0,
                FinalPixelPaddingBottom = 100,
                FinalPixelPaddingFront = 200,
                Duration = duration,
                GradientDuration = gradientDuration,
                InAnimation = false
            };

            AnimationParams fadeInFromTopParams = new AnimationParams
            {
                InitPixelPaddingBottom = -100,
                InitPixelPaddingFront = 200,
                InitCanvasGroupAlpha = 0,
                FinalCanvasGroupAlpha = 1,
                FinalPixelPaddingBottom = 0,
                FinalPixelPaddingFront = 0,
                Duration = duration,
                GradientDuration = gradientDuration,
                InAnimation = true
            };

            AnimationParams fadeOutFromTopParams = new AnimationParams
            {
                InitPixelPaddingBottom = 0,
                InitPixelPaddingFront = 0,
                InitCanvasGroupAlpha = 1,
                FinalCanvasGroupAlpha = 0,
                FinalPixelPaddingBottom = -100,
                FinalPixelPaddingFront = 200,
                Duration = duration,
                GradientDuration = gradientDuration,
                InAnimation = false
            };

            // Initialize the sequences
            _scrollFadeInFromBottomOfViewport = InitializeSequence(fadeInFromBottomParams);
            _scrollFadeOutFromBottomOfViewport = InitializeSequence(fadeOutFromBottomParams);
            _scrollFadeInFromTopOfViewport = InitializeSequence(fadeInFromTopParams);
            _scrollFadeOutFromTopOfViewport = InitializeSequence(fadeOutFromTopParams);
            _sequencesInitialized = true;
        }

        private Sequence InitializeSequence(AnimationParams animationParams)
        {
            var fadeCurve = animationParams.InAnimation ? _scrollInFadeCurve : _scrollOutFadeCurve;
            var moveCurve = animationParams.InAnimation ? _scrollInMoveCurve : _scrollOutMoveCurve;

            //init steps
            var sequence = DOTween.Sequence();
            sequence.Join(
                DOTween.To(_backgroundProvider.AttachData.GetPixelPaddingBottom,
                    _backgroundProvider.AttachData.SetPaddingBottom, animationParams.InitPixelPaddingBottom, 0));
            sequence.Join(
                DOTween.To(_backgroundProvider.AttachData.GetPixelPaddingFront,
                    _backgroundProvider.AttachData.SetPaddingFront, animationParams.InitPixelPaddingFront, 0));
            sequence.Join(_canvasGroup.DOFade(animationParams.InitCanvasGroupAlpha, 0));

            //animation steps
            sequence.Join(_canvasGroup.DOFade(animationParams.FinalCanvasGroupAlpha, animationParams.Duration))
                .SetEase(fadeCurve);
            sequence.Join(
                DOTween.To(_backgroundProvider.AttachData.GetPixelPaddingBottom,
                    _backgroundProvider.AttachData.SetPaddingBottom, animationParams.FinalPixelPaddingBottom,
                    animationParams.Duration)).SetEase(moveCurve);
            sequence.Join(
                DOTween.To(_backgroundProvider.AttachData.GetPixelPaddingFront,
                    _backgroundProvider.AttachData.SetPaddingFront, animationParams.FinalPixelPaddingFront,
                    animationParams.Duration)).SetEase(moveCurve);

            if(animationParams.InAnimation)
            {
                sequence.Join(
                    _backgroundProvider.BackgroundRenderer.DOGradientColor(_scrollInGradient,
                        animationParams.GradientDuration)).SetEase(Ease.Linear);
            }

            sequence.Pause();
            sequence.SetAutoKill(false);
            return sequence;
        }

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

        private void DisplacementLocalChanged(Vector3 displacement)
        {
            //todo figure out if this is needed for list items
            //_absolutePositionProvider.MoveAbsolute(drift);
        }

        public void UpdateViewData(ListItemViewModel viewModel)
        {
            _viewModel = viewModel;
            Value = _viewModel.Selected;
            if (_labelProvider != null)
                _labelProvider.SetLabel(_viewModel.LabelText);
            if (_descriptionProvider != null)
                _descriptionProvider.SetLabel(_viewModel.DescText);
            if (_rightTextProvider != null)
                _rightTextProvider.SetLabel(_viewModel.RightText);
            if (_iconPanelProvider != null)
                _iconPanelProvider.IconRenderer.Sprite = _viewModel.IconSprite;
            if (_mediaPanelProvider != null)
                _mediaPanelProvider.IconRenderer.Sprite = _viewModel.MediaSprite;
            if (_avatarPanelProvider != null)
                _avatarPanelProvider.IconRenderer.Sprite = _viewModel.AvatarSprite;

            if (_stateVisuals != null)
            {
                _stateVisuals.UpdateViewData(this);
            }

            if (_toggle != null)
            {
                _toggle.Value = Value;
                _toggle.ForceUpdateView();
            }

            if (_checkFillProvider != null)
            {
                _stateVisuals.UpdateViewData(this);
            }
        }


        public void Select(bool on)
        {
            Value = on;
            if (!Toggleable)
                return;
            if (_toggle != null)
            {
                _toggle.Value = on;
            }

            SelectChanged?.Invoke(on);
        }

        private Sequence _currentSequence;
        private bool _sequencesInitialized;

        public void OnScrollVisibilityChange(bool isVisible, bool comingFromTop, float speed = 1, bool resolveImmediately = false, Action onFinishFadeOut = null)
        {
            var oldSequence = _currentSequence;
            oldSequence?.Pause();

            if (resolveImmediately)
            {
                _canvasGroup.alpha = isVisible ? 1 : 0;
                return;
            }

            Sequence newSequence = null;

            if (isVisible)
            {
                newSequence = comingFromTop ? _scrollFadeInFromTopOfViewport : _scrollFadeInFromBottomOfViewport;
            }
            else
            {
                newSequence = comingFromTop ? _scrollFadeOutFromTopOfViewport : _scrollFadeOutFromBottomOfViewport;
            }


            // Get the progress of the old sequence.
            float oldSequenceProgress = oldSequence?.ElapsedPercentage(false) ?? 0;

            //now that we have the old progress. reset the old sequence
            oldSequence?.Goto(0, false);

            // If oldSequenceProgress is 0 or 1, make newSequenceProgress 0, otherwise calculate newSequenceProgress
            var newSequenceProgress = oldSequenceProgress == 0 || Mathf.Approximately(oldSequenceProgress, 1)
                ? 0
                : 1 - oldSequenceProgress;

            // Start the new sequence from the same progress point
            newSequence.Goto(newSequenceProgress);
            newSequence.timeScale = speed;
            newSequence.Play();

            newSequence.OnComplete(null);
            if(!isVisible && onFinishFadeOut != null)
            {
                TweenCallback newOnCompleteCallback = () =>
                {
                    onFinishFadeOut?.Invoke();
                };

                newSequence.OnComplete(newOnCompleteCallback);
            }
            // Update _previousVisibilityState
            _previousVisibilityState = isVisible;
            _currentSequence = newSequence;
        }
        public void DistanceVisibilityChange(float animationProgress, VisibilityState visibilityState)
        {
            if(!_sequencesInitialized)
                return;

            switch (visibilityState)
            {
                case VisibilityState.Invisible:
                    break;
                case VisibilityState.Visible:
                    break;
                case VisibilityState.TopEdge:
                    _currentSequence = _scrollFadeOutFromTopOfViewport;
                    _currentSequence.Goto(_currentSequence.Duration(false) * animationProgress, false);

                    break;
                case VisibilityState.BottomEdge:
                    _currentSequence = _scrollFadeOutFromBottomOfViewport;
                    _currentSequence.Goto(_currentSequence.Duration(false) * animationProgress, false);
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(visibilityState), visibilityState, null);
            }
            //_canvasGroup.alpha = 1- animationProgress;
        }
#else
        public void Initialize(ButtonModel controller)
        {
            throw new NotImplementedException();
        }

        public void UpdateViewData(ListItemViewModel viewModel)
        {
            throw new NotImplementedException();
        }

        public void Select(bool on)
        {
            throw new NotImplementedException();
        }

        public void DistanceVisibilityChange(float maxDistanceForFadeOut, VisibilityState currentState)
        {
            throw new NotImplementedException();
        }

        public void OnScrollVisibilityChange(bool p0, bool closerToTopOfViewport, float animationSpeed, bool resolveImmediately, Action action)
        {
            throw new NotImplementedException();
        }

        public void OnScrollVisibilityChange(bool p0, bool closerToTopOfViewport, float animationSpeed, bool resolveImmediately)
        {
            throw new NotImplementedException();
        }
#endif
    }
}
