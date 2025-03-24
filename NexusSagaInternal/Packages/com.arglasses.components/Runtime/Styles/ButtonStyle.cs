using System;
using System.Text.RegularExpressions;
using FigmaUnity.FigmaComponents;

namespace ARGlasses.Components
{
    [Serializable]
    public struct ButtonStyle : IComponentStyle
    {
        public Skin Skin;
        public ButtonType ButtonType;
        public Use Use;
        public bool Icon;
        
        public bool HideLabel;
        public bool Stateful;

        public override string ToString()
        {
            return
                $"Skin={Skin}, Button Type={ButtonType}, Use={Use}, Icon?={Icon}, Icon Size=Small (64px), Hide Label?={HideLabel}";
        }

        //Skin=ARDS, Button Type=Text, Use=Warning, Icon?=true, Icon Size=Small (64px), Hide Label?=false, State=Gaze
        //Skin=ARDS,ButtonType=Round,Use=Segmented,Icon=true,IconSize=Small(64px),HideLabel=true,State=Default
        public void FromString(string componentStyleString)
        {
            // Remove all spaces from the input string
            componentStyleString = componentStyleString.Replace(" ", "");

            var skinMatch = Regex.Match(componentStyleString, @"Skin=(\w+)");
            var buttonTypeMatch = Regex.Match(componentStyleString, @"ButtonType=(\w+)");
            //var useMatch = Regex.Match(componentStyleString, @"Use=([\w-]+)");
            var iconMatch = Regex.Match(componentStyleString, @"Icon=(\w+)");
            //var iconSizeMatch = Regex.Match(componentStyleString, @"IconSize=([\w\s]+)(?:\(\d+px\))?");
            var hideLabelMatch = Regex.Match(componentStyleString, @"HideLabel=(\w+)");

            //string useValue = useMatch.Groups[1].Value.Replace("-", "");
            //string iconSizeValue = iconSizeMatch.Groups[1].Value.Trim();

            Skin = Enum.TryParse(skinMatch.Groups[1].Value, out Skin skin) ? skin : Skin.ARDS;
            ButtonType = Enum.TryParse(buttonTypeMatch.Groups[1].Value, out ButtonType buttonType)
                ? buttonType
                : ButtonType.Text;
            //Use = Enum.TryParse(useValue, out Use use) ? use : Use.Warning;
            Icon = bool.TryParse(iconMatch.Groups[1].Value, out bool icon) && icon;
            //IconSize = Enum.TryParse(iconSizeValue, out IconSize iconSize) ? iconSize : IconSize.Small;
            HideLabel = bool.TryParse(hideLabelMatch.Groups[1].Value, out bool hideLabel) && hideLabel;
        }
    }

    public enum ButtonType
    {
        Text,
        Round,
        App
    }

    public enum Use
    {
        Standard,
        CTA,
        Segmented,
        Negative,
        Positive,
        Educational,
        Flat,
        FlatSegmented, //"Flat - Segmented"
        Warning,
        Toggle,
        Custom
    }

    public enum IconSize
    {
        Small, //"Small (64px)"
        Medium,
        Large
    }
}
