using System;
using OSIG.Tools.Units;
using ProtoKit.UI;
using TMPro;
using UnityEngine;

namespace ARGlasses.Components
{
    [RequireComponent(typeof(PKUIText))]
    public class TyperampSwitcher : MonoBehaviour
    {

        [SerializeField] private PKUIText _textRenderer;

        [SerializeField] private Typeramp _typeramp;

        public Typeramp Typeramp
        {
            get => _typeramp;
            set
            {
                _typeramp = value;
                if (_textRenderer == null)
                {
                    _textRenderer = GetComponent<PKUIText>();
                }

                if (_textRenderer == null)
                {
                    Debug.LogError("type ramp switcher requires pkuitext");
                    enabled = false;
                }

                ApplyStyle();
            }
        }

        private void OnValidate()
        {
            PopulateDependencies();
        }

        private void Awake()
        {
            PopulateDependencies();
        }

        private void PopulateDependencies()
        {
            if (_textRenderer == null)
                _textRenderer = GetComponent<PKUIText>();

            if (_textRenderer == null)
            {
                Debug.LogError("type ramp switcher requires pkuitext");
                enabled = false;
            }
            ApplyStyle();
        }

        public void ApplyStyle()
        {
            var style = ViewCollectionManager.TyperampCollection.TyperampSettings[_typeramp];
            _textRenderer.font = style.Font;
            _textRenderer.UnitsContext.UpdateIfDirty(gameObject);
            _textRenderer.fontSize = new OCValue().SetFromPixels(_textRenderer.UnitsContext, style.FontSize);
            _textRenderer.fontWeight = FontWeight.Medium; //actual weight is baked into the font asset. enforce medium to match
            _textRenderer.fontStyle = style.Styles;
            _textRenderer.lineSpacing = style.LineSpacing;
            _textRenderer.characterSpacing = style.CharacterSpacing;
        }
    }
}
