using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace UIComponents.Runtime
{
    public class SiroLabelToggle : Toggle
    {
        [SerializeField] private string _onLabel;
        [SerializeField] private string _offLabel;
        [SerializeField] private TMP_Text _labelField;
        
        private void Awake()
        {
            onValueChanged.AddListener(ChangeLabel);
        }
        
        private void ChangeLabel(bool isOn)
        {
            _labelField.text = isOn ? _onLabel : _offLabel;
        }

        public void Initialise(bool isOn)
        {
            onValueChanged.AddListener(ChangeLabel);
            this.isOn = isOn;
        }

        public void SetState(bool isActive)
        {
            SetIsOnWithoutNotify(isActive);
            ChangeLabel(isActive);
        }
    }
}