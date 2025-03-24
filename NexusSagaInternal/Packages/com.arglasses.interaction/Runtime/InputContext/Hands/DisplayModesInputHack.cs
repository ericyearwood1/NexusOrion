using UnityEngine;

namespace ARGlasses.Interaction
{
    public class DisplayModesInputHack : MonoBehaviour
    {
        [SerializeField, ReadOnly] private HandsInputContext _handsInputContext;
        [SerializeField, ReadOnly] private ContextPrioritizer prioritizer;

        private void Awake()
        {
            this.Scene(ref _handsInputContext);
            this.Scene(ref prioritizer);
        }

        private void OnEnable()
        {
            HandsInputContext.DisableRayPitch();
            prioritizer.Category = InputCategory.Hands;
        }

        private void OnDisable()
        {
            HandsInputContext.ResetRayPitch();
        }
    }
}
