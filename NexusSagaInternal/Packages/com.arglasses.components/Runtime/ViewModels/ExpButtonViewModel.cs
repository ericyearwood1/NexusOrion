using System;
using UnityEngine;
using System.ComponentModel;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class ExpButtonViewModel : ViewModelBase
    {
        [SerializeField, FormerlySerializedAs("LabelText")]
        private string _labelText;
        [SerializeField, FormerlySerializedAs("Icon")]
        private Sprite _icon;

        public string LabelText
        {
            get => _labelText;
            set => SetField(ref _labelText, value);
        }

        public Sprite Icon
        {
            get => _icon;
            set => SetField(ref _icon, value);
        }
    }
}
