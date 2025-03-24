using System;
using FigmaUnity.FigmaComponents;

namespace ARGlasses.Components
{
    [Serializable]
    public struct ToggleStyle : IComponentStyle
    {
        public Skin Skin;
        public ToggleType ToggleType;

        public override string ToString()
        {
            return
                $"Skin={Skin}, Toggle Type={ToggleType}";
        }

        public void FromString(string componentStyleString)
        {
            throw new System.NotImplementedException();
        }
    }

    public enum ToggleType
    {
        Normal,
        Mini
    }
}
