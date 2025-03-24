using UnityEngine;

namespace ARGlasses.Components
{
    public class ButtonStyleAttribute : PropertyAttribute
    {
        public enum PropertyUsage
        {
            Full,
            Context
        }

        public PropertyUsage Usage;

        public ButtonStyleAttribute(PropertyUsage usage = PropertyUsage.Full)
        {
            Usage = usage;
        }
    }
}
