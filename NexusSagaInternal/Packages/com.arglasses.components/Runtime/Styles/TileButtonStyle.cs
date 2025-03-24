using System;
using FigmaUnity.FigmaComponents;

namespace ARGlasses.Components
{
    [Serializable]
    public struct TileButtonStyle : IComponentStyle
    {
        public bool HasDescription;
        public bool HasIcon;

        public override string ToString()
        {
            return
                $"Description={HasDescription}, Icon={HasIcon}";
        }

        public void FromString(string componentStyleString)
        {
            throw new NotImplementedException();
        }
    }
}
