using UnityEngine;

namespace Robot.Runtime.View
{
    public class TargetObjectHighlightView : LabelHighlightView
    {
        [SerializeField] private Transform _lineContainer;
        [SerializeField] private SpriteRenderer _arrow;
        [SerializeField] private SpriteRenderer _line;
        [SerializeField] private SpriteRenderer _head;
        [SerializeField] private float _lineFinalAlpha = 0.2f;
        [SerializeField] private GameObject _carryState;
        [SerializeField] private GameObject _droppedState;
        protected override void UpdateDisplay(float progress)
        {
            base.UpdateDisplay(progress);
            _line.color = new Color(_color.r, _color.g, _color.b, progress * _lineFinalAlpha);
            _head.color = new Color(_color.r, _color.g, _color.b, progress);
            _label.alpha = progress;
        }

        public void ShowCarryState(float fadeOutDelay = -1.0f)
        {
            _arrow.color = new Color(_color.r, _color.g, _color.b, 1);
            _carryState.SetActive(true);
            _droppedState.SetActive(false);
            if (fadeOutDelay >= 0.0f)
            {
                base.Hide(delay : fadeOutDelay);
            }
        }

        public void ShowDroppedState()
        {
            _carryState.SetActive(false);
            _droppedState.SetActive(true);
        }

        public override void Show(Transform cameraTransform, string label)
        {
            _label.color = _color;
            base.Show(cameraTransform, label);
        }
    }
}