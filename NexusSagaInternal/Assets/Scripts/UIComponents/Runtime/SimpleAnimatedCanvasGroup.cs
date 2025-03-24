using Robot.Runtime.Data;
using Robot.Runtime.Utils;
using UnityEngine;

namespace UIComponents.Runtime
{
    public class SimpleAnimatedCanvasGroup : SimpleAnimatable
    {
        [SerializeField] private bool _isInteractable = true;
        [SerializeField] private bool _isBlocksRaycasts = true;
        [SerializeField] private CanvasGroup _group;

        private void Awake()
        {
            _state = _group.alpha == 0 ? HighlightState.Hidden : HighlightState.OnDisplay;
        }

        protected override void UpdateDisplay(float progress)
        {
            _group.alpha = progress;
        }

        protected override void AnimateInComplete()
        {
            base.AnimateInComplete();
            _group.interactable = _isInteractable;
            _group.blocksRaycasts = _isBlocksRaycasts;
        }

        protected override void AnimateOutComplete()
        {
            base.AnimateOutComplete();
            _group.interactable = false;
            _group.blocksRaycasts = false;
        }
    }
}