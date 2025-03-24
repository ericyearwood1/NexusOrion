using Robot.Runtime.Data;
using Robot.Runtime.View;
using UnityEngine;

namespace Robot.Runtime.Utils
{
    public abstract class SimpleAnimatable : MonoBehaviour, IResettable
    {
        private const float DefaultFadeInTime = 0.4f;
        private const float DefaultFadeOutTime = 0.4f;
        protected HighlightState _state;
        private float _currentFadeTime;
        
        public HighlightState State => _state;
        public bool IsDisposed { get; set; }

        public void Hide()
        {
            if (_state == HighlightState.AnimatingOut || _state == HighlightState.Hidden) return;
            Debug.Log($"{name} | hide");
            _state = HighlightState.AnimatingOut;
        }

        public void Show()
        {
            if (_state == HighlightState.AnimatingIn || _state == HighlightState.OnDisplay) return;
            if (IsDisposed) return;
            Debug.Log($"{name} | show");
            _state = HighlightState.AnimatingIn;
        }

        public void ShowImmediate()
        {
            if (IsDisposed) return;
            UpdateDisplay(1);
            AnimateInComplete();
        }
        
        public void HideImmediate()
        {
            if (IsDisposed) return;
            UpdateDisplay(0);
            AnimateOutComplete();
        }

        protected virtual void Update()
        {
            if (_state == HighlightState.None) return;
            switch (_state)
            {
                case HighlightState.AnimatingIn:
                    AnimateIn();
                    break;
                case HighlightState.OnDisplay:
                    break;
                case HighlightState.AnimatingOut:
                    AnimateOut();
                    break;
            }
        }

        protected abstract void UpdateDisplay(float progress);

        private void AnimateOut()
        {
            _currentFadeTime += Time.deltaTime;
            var progress = 1 - Mathf.Max(0, _currentFadeTime / DefaultFadeOutTime);
            UpdateDisplay(progress);
            if (progress <= 0)
            {
                AnimateOutComplete();
            }
        }

        private void AnimateIn()
        {
            _currentFadeTime += Time.deltaTime;
            var progress = Mathf.Max(0, _currentFadeTime / DefaultFadeInTime);
            UpdateDisplay(progress);
            if (progress >= 1)
            {
                AnimateInComplete();
            }
        }

        protected virtual void AnimateInComplete()
        {
            _currentFadeTime = 0;
            _state = HighlightState.OnDisplay;
        }

        protected virtual void AnimateOutComplete()
        {
            _currentFadeTime = 0;
            _state = HighlightState.Hidden;
        }

        public virtual void Reset()
        {
            IsDisposed = false;
            _state = HighlightState.None;
            _currentFadeTime = 0;
            UpdateDisplay(0);
        }
    }
}