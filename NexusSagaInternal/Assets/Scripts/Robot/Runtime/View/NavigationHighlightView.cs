using UnityEngine;

namespace Robot.Runtime.View
{
    public class NavigationHighlightView : LabelHighlightView
    {
        private const float MAX_FADE_OUT_TIME = 10;
        [SerializeField] private Transform _lineContainer;
        [SerializeField] private SpriteRenderer _line;
        [SerializeField] private SpriteRenderer _head;
        [SerializeField] private float _lineFinalAlpha = 0.2f;

        public override void Hide(float fadeOutTime = MAX_FADE_OUT_TIME, float delay = 0f)
        {
            base.Hide(MAX_FADE_OUT_TIME);
        }

        public override void OnInstructionComplete()
        {
            _fadeOutTime = DefaultFadeOutTime;
            _currentFadeTime = (1f - _head.color.a) * DefaultFadeOutTime;
            base.Hide();
        }

        protected override void UpdateDisplay(float progress)
        {
            base.UpdateDisplay(progress);
            _line.color = new Color(_color.r, _color.g, _color.b, progress * _lineFinalAlpha);
            _head.color = new Color(_color.r, _color.g, _color.b, progress);
        }
    }
}