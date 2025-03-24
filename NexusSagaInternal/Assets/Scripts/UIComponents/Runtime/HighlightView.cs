using UnityEngine;
using HighlightState = Robot.Runtime.Data.HighlightState;

namespace Robot.Runtime.View
{
    public class HighlightView : MonoBehaviour
    {
        protected const float DefaultFadeInTime = 0.4f;
        public const float DefaultFadeOutTime = 0.4f;
        
        [SerializeField] private bool _isBillboard = true;
        [SerializeField] protected SpriteRenderer _display;
        [SerializeField] protected float _maxAlpha = 1.0f;

        protected Transform _cameraTransform;
        protected float _currentFadeTime;
        protected HighlightState _state = HighlightState.OnDisplay;
        protected float _fadeInTime;
        protected float _fadeOutTime;

        protected Vector3 _targetScale;
        private float _fadeOutDelay;
        private float _currentDelayTime = 0f;
        protected Color _color = Color.white;

        public HighlightState State => _state;

        private void Awake()
        {
            _state = HighlightState.Hidden;
        }

        public void SetColor(Color color)
        {
            _color = color;
        }

        protected virtual void StartShow(Transform cameraTransform, float fadeInTime = DefaultFadeInTime)
        {
            _fadeInTime = fadeInTime;
            _cameraTransform = cameraTransform;
            _isBillboard = _isBillboard && _cameraTransform != null;
            _currentFadeTime = 0;
            _state = HighlightState.AnimatingIn;
            _targetScale = Vector3.one;
            transform.localScale = new Vector3(0, _targetScale.y, _targetScale.z);
            gameObject.SetActive(true);
        }

        public virtual void Show(Transform cameraTransform)
        {
            StartShow(cameraTransform);
        }
        
        public void Dispose()
        {
            _cameraTransform = null;
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
                    _currentDelayTime += Time.deltaTime;
                    if (_currentDelayTime >= _fadeOutDelay)
                    {
                        AnimateOut();
                    }
                    break;
            }
            Billboard();
        }

        protected virtual void UpdateDisplay(float progress)
        {
            _display.color = new Color(_color.r, _color.g, _color.b, progress * _maxAlpha);
        }

        protected virtual void AnimateOut()
        {
            _currentFadeTime += Time.deltaTime;
            var progress = 1 - Mathf.Max(0, _currentFadeTime / _fadeOutTime);
            progress = EaseInCubic(progress);
            UpdateDisplay(progress);
            transform.localScale = _targetScale;
            if (progress <= 0)
            {
                AnimateOutComplete();
            }
        }

        protected virtual void AnimateIn()
        {
            _currentFadeTime += Time.deltaTime;
            var progress = Mathf.Max(0, _currentFadeTime / _fadeInTime);
            progress = EaseOutCubic(progress);
            UpdateDisplay(progress);
            transform.localScale = Vector3.Lerp(new Vector3(0, _targetScale.y, _targetScale.z), _targetScale, progress);
            if (progress >= 1)
            {
                transform.localScale = _targetScale;
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

        private void Billboard()
        {
            if (!_isBillboard || _cameraTransform == null) return;
            var position = transform.position;
            var cameraPosition = _cameraTransform.position;
            if (position == cameraPosition) return;
            var lookRotation = Quaternion.LookRotation(position - cameraPosition);
            lookRotation = Quaternion.Euler(new Vector3(0f, lookRotation.eulerAngles.y, 0f));
            transform.rotation = lookRotation;
        }

        public virtual void Reset()
        {
            _currentFadeTime = 0;
            UpdateDisplay(0);
            gameObject.SetActive(false);
            _state = HighlightState.Hidden;
        }

        public virtual void Hide(float fadeOutTime = DefaultFadeOutTime, float delay = 0f)
        {
            if (_state != HighlightState.AnimatingIn && _state != HighlightState.OnDisplay) return;
            _fadeOutTime = fadeOutTime;
            _fadeOutDelay = delay;
            _currentDelayTime = 0f;
            _state = HighlightState.AnimatingOut;
        }

        public virtual void OnInstructionComplete()
        {
            Hide();
        }

        public void OnActionComplete()
        {
            Hide();
        }

        public float EaseInCubic(float x)
        {
            return x * x * x;
        }

        public float EaseOutCubic(float x)
        {
            return 1 - Mathf.Pow(1 - x, 3);
        }
    }
}