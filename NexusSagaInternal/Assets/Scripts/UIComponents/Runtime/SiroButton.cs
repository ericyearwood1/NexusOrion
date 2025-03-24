using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace View.UI
{
    public class SiroButton : Button
    {
        [SerializeField] private TMP_Text _label;
        [SerializeField] private Color _normalColor = new Color(1, 1, 1, 1);
        [SerializeField] private Color _selectedColor = new Color(1, 1, 1, 1);
        [SerializeField] private Color _highlightedColor = new Color(0.93f, 0.93f, 0.93f, 1);
        [SerializeField] private Color _disabledColor = new Color(1, 1, 1, 0.25f);
        
        protected override void DoStateTransition(SelectionState state, bool instant)
        {
            base.DoStateTransition(state, instant);
            if (_label == null) return; 
            switch (state)
            {
                case SelectionState.Disabled : _label.color = _disabledColor;
                    break;
                case SelectionState.Selected : _label.color = _selectedColor;
                    break;
                case SelectionState.Highlighted : _label.color = _highlightedColor;
                    break;
                case SelectionState.Normal : _label.color = _normalColor;
                    break;
                default:
                    _label.color = _normalColor;
                    break;
            }
            // if (m_TargetGraphic == null)
            //     return;
            //
            // m_TargetGraphic.CrossFadeColor(tintColor, instant ? 0f : m_Button.colors.fadeDuration, true, true);
        }
    }
}