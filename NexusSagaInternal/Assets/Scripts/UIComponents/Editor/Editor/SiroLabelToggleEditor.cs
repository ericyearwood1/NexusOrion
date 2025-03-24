using UIComponents.Runtime;
using UnityEditor;
using UnityEditor.UI;

namespace View.UI.Editor
{
    [CustomEditor(typeof(SiroLabelToggle), true)]
    [CanEditMultipleObjects]
    public class SiroLabelToggleEditor : ToggleEditor
    {
        private SerializedProperty m_OnStateChangedProperty;
        private SerializedProperty _labelField;
        private SerializedProperty _onLabel;
        private SerializedProperty _offLabel;

        protected override void OnEnable()
        {
            base.OnEnable();
            _labelField = serializedObject.FindProperty("_labelField");
            _onLabel = serializedObject.FindProperty("_onLabel");
            _offLabel = serializedObject.FindProperty("_offLabel");
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            EditorGUILayout.Space();
            serializedObject.Update();
            EditorGUILayout.PropertyField(_labelField);
            EditorGUILayout.PropertyField(_onLabel);
            EditorGUILayout.PropertyField(_offLabel);
            serializedObject.ApplyModifiedProperties();
        }
    }
}