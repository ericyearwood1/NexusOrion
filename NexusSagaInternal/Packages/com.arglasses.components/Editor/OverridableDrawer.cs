using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomPropertyDrawer(typeof(OverridableValue<>))]
    public class OverridableDrawer : PropertyDrawer
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            SerializedProperty overrideProp = property.FindPropertyRelative("Override");
            SerializedProperty valueProp = property.FindPropertyRelative("Value");

            EditorGUI.BeginProperty(position, label, property);

            float toggleWidth = 20;  // Width for the toggle

            Rect toggleRect = new Rect(position.x, position.y, toggleWidth, position.height);
            overrideProp.boolValue = EditorGUI.Toggle(toggleRect, overrideProp.boolValue);

            position.x += toggleWidth;     // Move the position to the right for the next field
            position.width -= toggleWidth; // Adjust the width for the main field to take up remaining space

            using (new EditorGUI.DisabledGroupScope(!overrideProp.boolValue))
            {
                EditorGUI.PropertyField(position, valueProp, label, true);
            }

            EditorGUI.EndProperty();
        }
    }
}
