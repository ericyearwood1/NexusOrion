using System;
using FigmaUnity.FigmaComponents;

namespace ARGlasses.Components
{
    [Serializable]
    public struct ListItemStyle : IComponentStyle
    {
        public Skin Skin;
        public ListItemType ListItemType;

        public override string ToString()
        {
            return
                $"Skin={Skin}, List Item Type={ListItemType}";
        }

        public void FromString(string componentStyleString)
        {
            throw new System.NotImplementedException();
        }

        public override bool Equals(object obj)
        {
            if (obj is ListItemStyle style)
            {
                return Skin == style.Skin &&
                       ListItemType == style.ListItemType;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Skin, ListItemType);
        }
    }

    public enum ListItemType
    {
        Media,
        Avatar,
        Icon,
        IconWithToggle,
        TextOnly,
        TextWithToggle,
        EnumeratedText,
        EnumeratedTextWithRightText,
        Checklist
    }
}
