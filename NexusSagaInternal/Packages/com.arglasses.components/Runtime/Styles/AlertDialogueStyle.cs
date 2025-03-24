using System;
using FigmaUnity.FigmaComponents;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    [Serializable]
    public struct AlertDialogueStyle : IComponentStyle
    {
        public AlertDialogueLayout Layout;
        public Skin Skin;
        public AlertDialogueSource Source;
        public AlertDialogueFromStyle FromStyle;
        [Range(0,3)]public int NumberOfButtons;
        public bool HasDescription;
        [EnumSubset("Text", "Round")]
        public ButtonType ButtonType;

        public override string ToString()
        {
            return
                $"Source={Source}, Style={FromStyle}, Layout={Layout}, # of Buttons={NumberOfButtons}, Button Type={ButtonType}, Description?={HasDescription}";
        }

        public void FromString(string componentStyleString)
        {
            throw new System.NotImplementedException();
        }
    }

    public enum AlertDialogueLayout
    {
        Horizontal,
        SideBySide,
        Vertical
    }

    public enum AlertDialogueSource
    {
        Augment,
        System
    }
    public enum AlertDialogueFromStyle
    {
        FromApp,
        System
    }
}
