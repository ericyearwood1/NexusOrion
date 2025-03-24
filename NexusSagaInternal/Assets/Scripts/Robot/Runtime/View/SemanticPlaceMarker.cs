using Robot.Runtime.Data;
using UnityEngine;

namespace Robot.Runtime.View
{
    public class SemanticPlaceMarker : HighlightView
    {
        [SerializeField] private Transform _gripperUpIndicator;
        [SerializeField] private Transform _upIndicator;
        
        public void SetIndicatorPosition(Vector3 position)
        {
            _upIndicator.position = position;
            _gripperUpIndicator.position = position;
        }
        
        public void SetIndicatorOrientation(Quaternion orientation)
        {
            _gripperUpIndicator.rotation = orientation;
        }
        
        public void SetUpIndicatorOrientation(Quaternion orientation)
        {
            _upIndicator.localRotation = orientation;
        }
        
        protected override void StartShow(Transform cameraTransform, float fadeInTime = DefaultFadeInTime)
        {
            _fadeInTime = fadeInTime;
            _cameraTransform = cameraTransform;
            _currentFadeTime = 0;
            _state = HighlightState.AnimatingIn;
            _targetScale = Vector3.one;
            _upIndicator.localScale = new Vector3(0, _targetScale.y, _targetScale.z);
            gameObject.SetActive(true);
        }
        
        protected override void AnimateIn()
        {
            _currentFadeTime += Time.deltaTime;
            var progress = Mathf.Max(0, _currentFadeTime / base._fadeInTime);
            progress = EaseOutCubic(progress);
            UpdateDisplay(progress);
            _upIndicator.localScale = Vector3.Lerp(new Vector3(0, _targetScale.y, _targetScale.z), _targetScale, progress);
            if (progress >= 1)
            {
                _upIndicator.localScale = _targetScale;
                AnimateInComplete();
            }
        }
    }
}