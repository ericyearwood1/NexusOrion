using System;
using System.Diagnostics;
using System.Threading.Tasks;
using ARGlasses.Interaction;
using ARGlasses.Interaction.Motion;
using ProtoKit.UI;
using UnityEngine;
using UnityEngine.Events;
using Debug = UnityEngine.Debug;

namespace ARGlasses.Components
{
    public class TooltipView : MonoBehaviour, IArgView<TooltipViewModel, ButtonModel>
    {
        [SerializeField] private TooltipStyle _style;

        private ButtonModel _buttonModel;
        
        public ArgIcon Icon;
        public PKUIText TitleText;
        public PKUIText DescriptionText;

        public CanvasGroup _canvasGroup;
        
        public GenericDictionary<FromDirection, GameObject> Arrows;
        
        public TooltipStyle Style => _style;

        public UnityEvent OnHidden;

        private TooltipViewModel _viewModel;
        private Vector3 _animHiddenPos;
        private Vector3 _animShownPos;
        private bool _debugging = false;
        
        private Motion<float> _currentMotion;
        
        private enum VisibilityState {Hidden, Hiding, Shown, Showing}

        private VisibilityState _visibilityState = VisibilityState.Hidden;
        
        public void Initialize(ButtonModel controller)
        {
            _buttonModel = controller;
            _currentMotion = new Motion<float>(0, 0, new MotionParamsEasing(Easing.Linear, 1f));
            
            if(_canvasGroup.alpha > 0f) _canvasGroup.alpha = 0f;
        }
        
        void Update()
        {
            if(_currentMotion != null) ApplyValues(_currentMotion.Step(Time.deltaTime));
        }
        
        public void UpdateViewData(TooltipViewModel viewModel)
        {
            _viewModel = viewModel;
            
            Icon.IconRenderer.Sprite = viewModel.IconSprite;
            TitleText.Text = viewModel.TitleText;
            DescriptionText.Text = viewModel.DescriptionText;

            transform.localScale = Vector3.one * viewModel.HiddenScale;
            
            foreach (var arrow in Arrows)
            {
                arrow.Value.SetActive(arrow.Key == viewModel.FromDirection);
            }

            transform.position = GetTranslationPos(0f);
        }

        public void Show()
        {
            _visibilityState = VisibilityState.Showing;
            
            if(_debugging) Debug.Log("Showing");
            
            _currentMotion.MotionParams = _viewModel.EaseIn;
            _currentMotion.SetGoal(1f);
            _currentMotion.ResetCallbacks();
            
            _currentMotion.OnComplete(() =>
            {
                _visibilityState = VisibilityState.Shown;
            });
        }

        public async void Hide(float delay = 0f)
        {
            _visibilityState = VisibilityState.Hiding;

            _currentMotion.ResetCallbacks();
            
            if(_debugging) Debug.Log("Hiding");
            
            float hidingTime = Time.time;

            while (Time.time < hidingTime + delay)
            {
                await Task.Yield();
                if (_visibilityState != VisibilityState.Hiding)
                {
                    if(_debugging) Debug.Log($"State: {_visibilityState} Canceling Hiding");
                    return;
                }
            }
            
            _currentMotion.MotionParams = _viewModel.EaseOut;
            _currentMotion.SetGoal(0f);
            
            _currentMotion.OnComplete(() =>
            {
                if(_debugging) Debug.Log("OnHidden");
                OnHidden?.Invoke();
            });
        }
        
        public void Select(bool on)
        {
            throw new NotImplementedException();
        }

        void ApplyValues(float i)
        {
            SetTranslation(i);
            
            transform.localScale = Vector3.LerpUnclamped(Vector3.one * _viewModel.HiddenScale, Vector3.one, i);
            
            _canvasGroup.alpha = i;
        }
        
        public void SetTranslation(float i)
        {
            if (_viewModel == null || this == null) return;
            
            transform.position = GetTranslationPos(i);
        }

        private Vector3 GetTranslationPos(float i)
        {
            var originPos = _viewModel.SourceTarget.transform.position;
            _animShownPos = transform.parent.position;
            
            //Calculate the hidden position
            _animHiddenPos = Vector3.LerpUnclamped(originPos, _animShownPos, _viewModel.HiddenTranslation);
            
            //Calculate the current lerped position
            Vector3 currentPos = Vector3.LerpUnclamped(_animHiddenPos, _animShownPos, i);

            var vToDestination = _animShownPos - _animHiddenPos;
            
            if (_debugging)
            {
                Debug.Log($"<b><color=orange>Set Translation</color></b>. i: <b>{i}</b> // vToEnd: {vToDestination} // HiddenPos: {_animHiddenPos} // ShownPos: {_animShownPos}", gameObject);
                Debug.DrawLine(_animShownPos, _animHiddenPos, Color.red, 4f);
                Debug.DrawLine(_animHiddenPos, _animHiddenPos + Vector3.up * vToDestination.magnitude * 0.25f, Color.yellow, 3f);
            }
            
            return currentPos;
        }
    }
}