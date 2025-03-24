using Robot.Runtime.Data;
using UnityEngine;

namespace Robot.Runtime.View
{
    public class DrawerMarker : HighlightView
    {
        [SerializeField] private SpriteRenderer _trackingHighlightUI;
        [SerializeField] private Transform _trackingHighlight;

        public Transform TrackingHighlight => _trackingHighlight;
        
        protected override void UpdateDisplay(float progress)
        {
            base.UpdateDisplay(progress);
            _trackingHighlightUI.color = new Color(1, 1, 1, progress);
        }
        
        protected override void StartShow(Transform cameraTransform, float fadeInTime = DefaultFadeInTime)
        {
            _fadeInTime = fadeInTime;
            _cameraTransform = cameraTransform;
            _currentFadeTime = 0;
            _state = HighlightState.AnimatingIn;
            _targetScale = _trackingHighlight.localScale;
            _trackingHighlight.localScale = new Vector3(0, _targetScale.y, _targetScale.z);
            gameObject.SetActive(true);
        }
        
        protected override void AnimateIn()
        {
            _currentFadeTime += Time.deltaTime;
            var progress = Mathf.Max(0, _currentFadeTime / base._fadeInTime);
            progress = EaseOutCubic(progress);
            UpdateDisplay(progress);
            _trackingHighlight.localScale = Vector3.Lerp(new Vector3(0, _targetScale.y, _targetScale.z), _targetScale, progress);
            if (progress >= 1)
            {
                _trackingHighlight.localScale = _targetScale;
                AnimateInComplete();
            }
        }
    }
}