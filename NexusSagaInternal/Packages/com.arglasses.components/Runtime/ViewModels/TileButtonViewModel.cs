using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{

    [Serializable]
    public class TileButtonViewModel : ViewModelBase
    {
        [SerializeField] private string _labelText;
        [SerializeField] private string _descriptionText;
        [SerializeField] private Sprite _iconSprite;
        [SerializeField] private bool _value;

        public string LabelText
        {
            get => _labelText;
            set => SetField(ref _labelText, value);
        }

        public string DescriptionText
        {
            get => _descriptionText;
            set => SetField(ref _descriptionText, value);
        }

        public Sprite IconSprite
        {
            get => _iconSprite;
            set => SetField(ref _iconSprite, value);
        }

        public bool Value
        {
            get => _value;
            set => SetField(ref _value, value);
        }
    }
}
