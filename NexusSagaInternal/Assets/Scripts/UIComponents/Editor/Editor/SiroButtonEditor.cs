using UnityEditor;
using UnityEditor.UI;
using UnityEngine;

namespace View.UI.Editor
{
    namespace UnityEditor.UI
    {
        [CustomEditor(typeof(SiroButton), true)]
        [CanEditMultipleObjects]
        public class SiroButtonEditor : ButtonEditor
        {
            SerializedProperty m_OnStateChangedProperty;
            private SerializedProperty _normalColor;
            private SerializedProperty _selectedColor;
            private SerializedProperty _highlightedColor;
            private SerializedProperty _disabledColor;
            private SerializedProperty _label;

            protected override void OnEnable()
            {
                base.OnEnable();
                _label = serializedObject.FindProperty("_label");
                _normalColor = serializedObject.FindProperty("_normalColor");
                _selectedColor = serializedObject.FindProperty("_selectedColor");
                _highlightedColor = serializedObject.FindProperty("_highlightedColor");
                _disabledColor = serializedObject.FindProperty("_disabledColor");
            }

            public override void OnInspectorGUI()
            {
                base.OnInspectorGUI();
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Label State Colours:");
                serializedObject.Update();
                EditorGUILayout.PropertyField(_label, new GUIContent("Target Label"));
                EditorGUILayout.PropertyField(_normalColor);
                EditorGUILayout.PropertyField(_selectedColor);
                EditorGUILayout.PropertyField(_highlightedColor);
                EditorGUILayout.PropertyField(_disabledColor);
                serializedObject.ApplyModifiedProperties();
            }
        }
    }
}