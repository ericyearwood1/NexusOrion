using System.Linq;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomPropertyDrawer(typeof(ButtonStyle))]
    public class ButtonStylePropertyDrawer : PropertyDrawer
    {
        private ButtonStyleAttribute.PropertyUsage usage = ButtonStyleAttribute.PropertyUsage.Full;

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            // GUIContent styleLabel = new GUIContent("Style");
            // GUIStyle boldLabelStyle = new GUIStyle(EditorStyles.label);
            // boldLabelStyle.fontStyle = FontStyle.Bold;
            // EditorGUI.LabelField(new Rect(position.x, position.y, 40, EditorGUIUtility.singleLineHeight), styleLabel, boldLabelStyle);
            EditorGUI.BeginProperty(position, label, property);
            // Retrieve properties
            SerializedProperty skinProperty = property.FindPropertyRelative("Skin");
            SerializedProperty buttonTypeProperty = property.FindPropertyRelative("ButtonType");
            SerializedProperty useProperty = property.FindPropertyRelative("Use");
            SerializedProperty iconProperty = property.FindPropertyRelative("Icon");
            SerializedProperty hideLabelProperty = property.FindPropertyRelative("HideLabel");
            SerializedProperty statefulProperty = property.FindPropertyRelative("Stateful");

            ButtonStyleAttribute customAttribute =
                fieldInfo.GetCustomAttributes(typeof(ButtonStyleAttribute), inherit: true).FirstOrDefault() as
                    ButtonStyleAttribute;
            usage = ButtonStyleAttribute.PropertyUsage.Full;
            if (customAttribute != null)
            {
                usage = customAttribute.Usage;
            }
            
            //draw skin
            position.y += EditorGUIUtility.singleLineHeight;
            EditorGUI.PropertyField(
                new Rect(position.x, position.y, position.width, EditorGUIUtility.singleLineHeight), skinProperty,
                new GUIContent("Skin"));

            var buttonTypeValue = buttonTypeProperty.enumValueIndex;
            //draw ButtonType
            position.y += EditorGUIUtility.singleLineHeight;
            EditorGUI.PropertyField(
                new Rect(position.x, position.y, position.width,
                    EditorGUIUtility.singleLineHeight), buttonTypeProperty, new GUIContent("ButtonType"));

            //draw optionals
            position.y += EditorGUIUtility.singleLineHeight;
            if (buttonTypeValue == (int)ButtonType.Text)
            {
                EditorGUI.PropertyField(
                    new Rect(position.x, position.y, position.width,
                        EditorGUIUtility.singleLineHeight), iconProperty, new GUIContent("Icon"));
            }
            else
            {
                EditorGUI.PropertyField(
                    new Rect(position.x, position.y, position.width,
                        EditorGUIUtility.singleLineHeight), hideLabelProperty, new GUIContent("Label"));
            }
            
            if (usage == ButtonStyleAttribute.PropertyUsage.Full || useProperty.intValue == (int)Use.Custom)
            {
                position.y += EditorGUIUtility.singleLineHeight;
                EditorGUI.PropertyField(
                    new Rect(position.x, position.y, position.width,
                        EditorGUIUtility.singleLineHeight), statefulProperty, new GUIContent("Stateful"));
            }

            EditorGUI.EndProperty();
        }

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            return EditorGUIUtility.singleLineHeight * (usage == ButtonStyleAttribute.PropertyUsage.Context ? 4 : 5);
        }
    }
}
