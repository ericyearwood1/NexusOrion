using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public class LauncherButtonViewModel : ViewModelBase
    {
        [FormerlySerializedAs("LabelText")]
        [SerializeField]
        private string _labelText;

        [FormerlySerializedAs("AppImage")]
        [SerializeField]
        private Sprite _appImage;

        public string LabelText
        {
            get => _labelText;
            set => SetField(ref _labelText, value);
        }

        public Sprite AppImage
        {
            get => _appImage;
            set => SetField(ref _appImage, value);
        }
    }
}
