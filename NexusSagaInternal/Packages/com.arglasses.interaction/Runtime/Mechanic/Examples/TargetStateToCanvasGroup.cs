using ARGlasses.Interaction.Motion;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class TargetStateToCanvasGroup : MonoBehaviour
    {
        [SerializeField] MotionParamsEasing _motionParams;
        [SerializeField] private Selectable _selectable;
        [SerializeField] private CanvasGroup _canvasGroup;

        private void Awake()
        {
            this.Ensure(ref _selectable);
            this.Ensure(ref _canvasGroup);
            _selectable.WhenStateChanged += HandleStateChanged;
            _canvasGroup.alpha = 0;
        }

        private void HandleStateChanged(TargetState state)
        {
            var hoverOrPress = state.IsHoverOrPress();
            _canvasGroup.FadeOneShot(hoverOrPress ? 1 : 0, _motionParams);
        }
    }
}
