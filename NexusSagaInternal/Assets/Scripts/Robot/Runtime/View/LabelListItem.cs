using TMPro;
using UnityEngine;

namespace Robot.Runtime.View
{
    public class LabelListItem : MonoBehaviour
    {
        [SerializeField] private Color _activeColor;
        [SerializeField] private Color _inactiveColor;
        [SerializeField] private TMP_Text _label;

        public void SetLabel(string label)
        {
            _label.text = label;
        }

        public void SetActionActive()
        {
            _label.color = _activeColor;
        }
        
        public void SetActionInactive()
        {
            _label.color = _inactiveColor;
        }
    }
}