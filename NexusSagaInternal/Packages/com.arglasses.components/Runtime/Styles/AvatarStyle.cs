using System;
using FigmaUnity.FigmaComponents;

namespace ARGlasses.Components
{
    [Serializable]
    public struct AvatarStyle : IComponentStyle
    {
        public AvatarSize Size;

        public override string ToString()
        {
            return
                $"Size={Size}";
        }

        public void FromString(string componentStyleString)
        {
            throw new NotImplementedException();
        }
    }
}
