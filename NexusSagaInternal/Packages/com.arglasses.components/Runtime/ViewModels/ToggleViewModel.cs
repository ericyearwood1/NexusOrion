using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class ToggleViewModel : ViewModelBase
    {
        [SerializeField]
        [FormerlySerializedAs("Style")]
        private ToggleStyle _style;

        [SerializeField]
        [FormerlySerializedAs("Value")]
        private bool _value;

        public ToggleStyle Style
        {
            get => _style;
            set => SetField(ref _style, value);
        }

        public bool Value
        {
            get => _value;
            set => SetField(ref _value, value);
        }
    }
}
