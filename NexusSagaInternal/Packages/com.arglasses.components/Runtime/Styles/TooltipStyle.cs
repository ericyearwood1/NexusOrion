using System;
using FigmaUnity.FigmaComponents;

namespace ARGlasses.Components
{
    [Serializable]
    public struct TooltipStyle : IComponentStyle
    {
        public Skin Skin;
        
        public bool HasIcon;
        public bool HasTitle;
        
        public void FromString(string componentStyleString)
        {
            throw new System.NotImplementedException();
        }

        public override string ToString()
        {
            return $"Skin={Skin}, Has Icon={HasIcon}, Has Title ={HasTitle}";
        }
    }
    
    public enum FromDirection { Bottom, Top, Left, Right }
}