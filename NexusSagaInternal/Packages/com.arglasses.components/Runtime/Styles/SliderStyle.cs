using System;
using System.Text.RegularExpressions;
using FigmaUnity.FigmaComponents;

namespace ARGlasses.Components
{
    [Serializable]
    public struct SliderStyle : IComponentStyle
    {
        public Skin Skin;
        public SliderType SliderType;
        public Orientation Orientation;

        public override string ToString()
        {
            return
                $"Skin={Skin}, Slider Type={SliderType}, Orientation?={Orientation}";
        }

        public void FromString(string componentStyleString)
        {
            throw new NotImplementedException();
        }
    }

    public enum Orientation
    {
        Horizontal,
        Vertical
    }
    public enum SliderType
    {
        Number,
        Icon,
    }
}
