using System;
using UnityEngine;
using System.ComponentModel;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class ListItemViewModel : ViewModelBase
    {
        [SerializeField, FormerlySerializedAs("Style")]
        private ListItemStyle _style;
        [SerializeField, FormerlySerializedAs("LabelText")]
        private string _labelText;
        [SerializeField, FormerlySerializedAs("DescText")]
        private string _descText;
        [SerializeField, FormerlySerializedAs("RightText")]
        private string _rightText;
        [SerializeField, FormerlySerializedAs("IconSprite")]
        private Sprite _iconSprite;
        [SerializeField, FormerlySerializedAs("AvatarSprite")]
        private Sprite _avatarSprite;
        [SerializeField, FormerlySerializedAs("MediaSprite")]
        private Sprite _mediaSprite;
        [SerializeField, FormerlySerializedAs("Selected")]
        private bool _selected;

        public ListItemStyle Style
        {
            get => _style;
            set => SetField(ref _style, value);
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

        public string RightText
        {
            get => _rightText;
            set => SetField(ref _rightText, value);
        }

        public Sprite IconSprite
        {
            get => _iconSprite;
            set => SetField(ref _iconSprite, value);
        }

        public Sprite AvatarSprite
        {
            get => _avatarSprite;
            set => SetField(ref _avatarSprite, value);
        }

        public Sprite MediaSprite
        {
            get => _mediaSprite;
            set => SetField(ref _mediaSprite, value);
        }

        public bool Selected
        {
            get => _selected;
            set => SetField(ref _selected, value);
        }

        public override bool Equals(object obj)
        {
            if (obj is ListItemViewModel item)
            {
                return Style.Equals(item.Style) &&
                       LabelText == item.LabelText &&
                       DescText == item.DescText &&
                       RightText == item.RightText &&
                       Equals(IconSprite, item.IconSprite) &&
                       Equals(AvatarSprite, item.AvatarSprite) &&
                       Equals(MediaSprite, item.MediaSprite) &&
                       Selected == item.Selected;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Style, LabelText, DescText, RightText, IconSprite, AvatarSprite, MediaSprite, Selected);
        }
    }
}
