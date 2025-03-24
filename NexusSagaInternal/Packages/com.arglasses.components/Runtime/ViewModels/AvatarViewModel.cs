using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class AvatarViewModel : ViewModelBase
    {
        [FormerlySerializedAs("Style")]
        [SerializeField]
        private AvatarStyle _style;

        [FormerlySerializedAs("AvatarImage")]
        [SerializeField]
        private Sprite _avatarImage;

        [FormerlySerializedAs("AppImage")]
        [SerializeField]
        private Sprite _appImage;

        [FormerlySerializedAs("IndicatorColor")]
        [SerializeField]
        private Color _indicatorColor;

        [FormerlySerializedAs("LabelText")]
        [SerializeField]
        private string _labelText;

        [FormerlySerializedAs("ShowLabelOnHover")]
        [SerializeField]
        private bool _showLabelOnHover;

        [FormerlySerializedAs("CounterText")]
        [SerializeField]
        private string _counterText;

        public AvatarStyle Style
        {
            get => _style;
            set => SetField(ref _style, value);
        }

        public Sprite AvatarImage
        {
            get => _avatarImage;
            set => SetField(ref _avatarImage, value);
        }

        public Sprite AppImage
        {
            get => _appImage;
            set => SetField(ref _appImage, value);
        }

        public Color IndicatorColor
        {
            get => _indicatorColor;
            set => SetField(ref _indicatorColor, value);
        }

        public string LabelText
        {
            get => _labelText;
            set => SetField(ref _labelText, value);
        }

        public bool ShowLabelOnHover
        {
            get => _showLabelOnHover;
            set => SetField(ref _showLabelOnHover, value);
        }

        public string CounterText
        {
            get => _counterText;
            set => SetField(ref _counterText, value);
        }
    }
}
