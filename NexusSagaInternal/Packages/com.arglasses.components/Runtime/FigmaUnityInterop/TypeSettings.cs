using System;
using TMPro;

namespace ARGlasses.Components
{
    [Serializable]
    public struct TypeSettings
    {
        public TMP_FontAsset Font;
        //public FontWeight Weight; //this is baked into the font asset. enforce normal
        public FontStyles Styles;
        public float FontSize;
        public float CharacterSpacing;
        public float LineSpacing;
        public float ParagraphSpacing;
    }
}