using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class ButtonViewModel : ViewModelBase
    {
        [FormerlySerializedAs("Style")]
        [SerializeField]
        private ButtonStyle _style;

        [FormerlySerializedAs("Use")]
        [SerializeField]
        private Use _use;

        [FormerlySerializedAs("LabelText")]
        [SerializeField]
        private string _labelText;

        [FormerlySerializedAs("DescText")]
        [SerializeField]
        private string _descText;

        [FormerlySerializedAs("Icon")]
        [SerializeField]
        private Sprite _icon;
        
        [FormerlySerializedAs("OffIcon")]
        [SerializeField]
        private Sprite _offIcon;
        
        [FormerlySerializedAs("OnIcon")]
        [SerializeField]
        private Sprite _onIcon;
        
        [FormerlySerializedAs("AppImage")]
        [SerializeField]
        private Sprite _appImage;

        [FormerlySerializedAs("Selected")]
        [SerializeField]
        private bool _selected;

        public ButtonStyle Style
        {
            get => _style;
            set => SetField(ref _style, value);
        }

        public Use Use
        {
            get => _use;
            set => SetField(ref _use, value);
        }

        public string LabelText
        {
            get => _labelText;
            set => SetField(ref _labelText, value);
        }

        public string DescText
        {
            get => _descText;
            set => SetField(ref _descText, value);
        }

        public Sprite Icon
        {
            get => _icon;
            set => SetField(ref _icon, value);
        }
       
        public Sprite OffIcon
        {
            get => _offIcon;
            set => SetField(ref _offIcon, value);
        }
        
        public Sprite OnIcon
        {
            get => _onIcon;
            set => SetField(ref _onIcon, value);
        }

        public Sprite AppImage
        {
            get => _appImage;
            set => SetField(ref _appImage, value);
        }

        public bool Selected
        {
            get => _selected;
            set
            {
                SetField(ref _selected, value);
            }
        }
    }
}
