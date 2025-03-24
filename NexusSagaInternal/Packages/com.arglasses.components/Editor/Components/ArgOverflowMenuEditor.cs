using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgOverflowMenu))]
    public class ArgOverflowMenuEditor : Editor
    {
        private SerializedProperty showVisibilityButtonLabel;
        private SerializedProperty visibilityToggle;
        private SerializedProperty direction;

        private void OnEnable()
        {
            // Initialize the SerializedProperties for the properties.
            showVisibilityButtonLabel = serializedObject.FindProperty("_showVisibilityButtonLabel");
            visibilityToggle = serializedObject.FindProperty("_visibilityToggle");
            direction = serializedObject.FindProperty("_direction");
        }

        public override void OnInspectorGUI()
        {
            // Update the SerializedObject with the latest values.
            serializedObject.Update();

            // Store the current values of the properties.
            bool currentShowVisibilityButtonLabel = showVisibilityButtonLabel.boolValue;
            int currentDirectionValue = direction.enumValueIndex;

            // Display the "_showVisibilityButtonLabel" property in the Inspector.
            EditorGUILayout.PropertyField(showVisibilityButtonLabel, new GUIContent("Show Visibility Button Label"));

            // Display the "_visibilityButton" property in the Inspector.
            //EditorGUILayout.PropertyField(visibilityToggle, new GUIContent("Visibility Toggle"));

            // Display the "_direction" property in the Inspector.
            EditorGUILayout.PropertyField(direction, new GUIContent("Direction"));

            // Apply any changes made to the SerializedObject.
            serializedObject.ApplyModifiedProperties();

            // Check if either property has changed and call "UpdateLayout" accordingly.
            if (currentShowVisibilityButtonLabel != showVisibilityButtonLabel.boolValue || currentDirectionValue != direction.enumValueIndex)
            {
                ArgOverflowMenu argOverflowMenu = (ArgOverflowMenu)target;

                argOverflowMenu.SetLabelVisibility(showVisibilityButtonLabel.boolValue);
                argOverflowMenu.UpdateLayout();
            }
        }

    }
}
