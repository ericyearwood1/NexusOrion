using System;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomPropertyDrawer(typeof(EnumSubsetAttribute))]
    public class EnumSubsetDrawer : PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EnumSubsetAttribute enumSubsetAttribute = (EnumSubsetAttribute)attribute;
            string[] allowedValues = enumSubsetAttribute.AllowedValues;

            if (property.propertyType == SerializedPropertyType.Enum)
            {
                EditorGUI.BeginProperty(position, label, property);

                string currentValue = property.enumNames[property.enumValueIndex];

                int selectedIndex = Array.IndexOf(allowedValues, currentValue);
                selectedIndex = Mathf.Max(selectedIndex, 0);

                selectedIndex = EditorGUI.Popup(position, label.text, selectedIndex, allowedValues);

                property.enumValueIndex = Array.IndexOf(property.enumNames, allowedValues[selectedIndex]);

                EditorGUI.EndProperty();
            }
            else
            {
                EditorGUI.PropertyField(position, property, label, true);
                Debug.LogError("EnumSubsetAttribute can only be used with enum fields.");
            }
        }
    }
}
