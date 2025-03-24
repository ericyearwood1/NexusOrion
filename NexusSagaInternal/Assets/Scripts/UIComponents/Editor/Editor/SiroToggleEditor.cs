using UnityEditor;
using UnityEditor.UI;

namespace View.UI.Editor
{
    [CustomEditor(typeof(SiroToggle), true)]
    [CanEditMultipleObjects]
    public class SiroToggleEditor : ToggleEditor
    {
        SerializedProperty m_OnStateChangedProperty;
        private SerializedProperty _background;

        protected override void OnEnable()
        {
            base.OnEnable();
            _background = serializedObject.FindProperty("_background");
        }

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            EditorGUILayout.Space();
            serializedObject.Update();
            EditorGUILayout.PropertyField(_background);
            serializedObject.ApplyModifiedProperties();
        }
    }
}