using TMPro;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class PushPullButtonLabel : MonoBehaviour
    {
        [SerializeField] private TextMeshProUGUI _text;
        private CanvasGroup _pkuiTextCanvasGroup;
        [SerializeField] private float _textFadeAlpha = 40f;
        [SerializeField] private float _textFadeLerp = 12f;
        [SerializeField, ReadOnly] private float _lastCanvasAlpha;

        private void Awake()
        {
            this.Sibling(ref _text);
            this.Ensure(ref _pkuiTextCanvasGroup);
        }

        private void Update()
        {
            _lastCanvasAlpha = Mathf.Lerp(_lastCanvasAlpha, 0, Time.deltaTime * _textFadeLerp);
            _pkuiTextCanvasGroup.alpha = _lastCanvasAlpha;
        }

        public void SetText(string text)
        {
            _text.text = text;
            _lastCanvasAlpha = _textFadeAlpha;
        }
    }
}
