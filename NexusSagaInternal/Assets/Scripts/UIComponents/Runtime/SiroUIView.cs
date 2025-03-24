using UnityEngine;

namespace View
{
    public class SiroUIView : MonoBehaviour
    {
        [SerializeField] private bool _isRaycastable = true;
        [SerializeField] private bool _isInteractable = true;
        public bool IsShown { get; private set; }

        [SerializeField] protected CanvasGroup _canvasGroup;
        
        private void Awake()
        {
            Hide();
        }
        
        [ContextMenu("Show")]
        public virtual void Show()
        {
            _canvasGroup.interactable = _isInteractable;
            _canvasGroup.blocksRaycasts = _isRaycastable;
            _canvasGroup.alpha = 1;
            IsShown = true;
        }

        [ContextMenu("Hide")]
        public virtual void Hide()
        {
            _canvasGroup.interactable = false;
            _canvasGroup.blocksRaycasts = false;
            _canvasGroup.alpha = 0;
            IsShown = false;
        }
        
        public void Disable()
        {
            _canvasGroup.interactable = false;
            _canvasGroup.blocksRaycasts = false;
        }
        
        public void Enable()
        {
            _canvasGroup.interactable = _isInteractable;
            _canvasGroup.blocksRaycasts = _isRaycastable;
        }
    }
}