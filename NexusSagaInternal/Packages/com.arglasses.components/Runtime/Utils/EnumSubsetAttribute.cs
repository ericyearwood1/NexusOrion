namespace ARGlasses.Components
{
    using System;
    using UnityEngine;

    [AttributeUsage(AttributeTargets.Field, AllowMultiple = false)]
    public class EnumSubsetAttribute : PropertyAttribute
    {
        public string[] AllowedValues { get; private set; }

        public EnumSubsetAttribute(params string[] allowedValues)
        {
            AllowedValues = allowedValues;
        }
    }

}
