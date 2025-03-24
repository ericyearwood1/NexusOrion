using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class AlertDialogueViewModel : ViewModelBase
    {
        [SerializeField] [FormerlySerializedAs("Title")]
        private string _title;
        [SerializeField] [FormerlySerializedAs("Desc")]
        private string _desc;
        [SerializeField] [FormerlySerializedAs("AppImage")]
        private Sprite _appImage;
        [SerializeField] [FormerlySerializedAs("ButtonIcon1")]
        private Sprite _buttonIcon1;
        [SerializeField] [FormerlySerializedAs("ButtonIcon2")]
        private Sprite _buttonIcon2;
        [SerializeField] [FormerlySerializedAs("ButtonIcon3")]
        private Sprite _buttonIcon3;
        [SerializeField] [FormerlySerializedAs("ButtonText1")]
        private string _buttonText1;
        [SerializeField] [FormerlySerializedAs("ButtonText2")]
        private string _buttonText2;
        [SerializeField] [FormerlySerializedAs("ButtonText3")]
        private string _buttonText3;

        public string Title
        {
            get => _title;
            set => SetField(ref _title, value);
        }

        public string Desc
        {
            get => _desc;
            set => SetField(ref _desc, value);
        }

        public Sprite AppImage
        {
            get => _appImage;
            set => SetField(ref _appImage, value);
        }

        public Sprite ButtonIcon1
        {
            get => _buttonIcon1;
            set => SetField(ref _buttonIcon1, value);
        }

        public Sprite ButtonIcon2
        {
            get => _buttonIcon2;
            set => SetField(ref _buttonIcon2, value);
        }

        public Sprite ButtonIcon3
        {
            get => _buttonIcon3;
            set => SetField(ref _buttonIcon3, value);
        }

        public string ButtonText1
        {
            get => _buttonText1;
            set => SetField(ref _buttonText1, value);
        }

        public string ButtonText2
        {
            get => _buttonText2;
            set => SetField(ref _buttonText2, value);
        }

        public string ButtonText3
        {
            get => _buttonText3;
            set => SetField(ref _buttonText3, value);
        }
    }
}
