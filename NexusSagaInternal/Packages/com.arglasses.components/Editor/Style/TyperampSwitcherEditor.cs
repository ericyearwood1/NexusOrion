#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(TyperampSwitcher))]
    public class TyperampSwitcherEditor : Editor
    {
        SerializedProperty _typerampProperty;

        private void OnEnable()
        {
            _typerampProperty = serializedObject.FindProperty("_typeramp");
            TyperampSwitcher typerampSwitcher = (TyperampSwitcher)target;

            if (!EditorApplication.isPlayingOrWillChangePlaymode && !EditorApplication.isPlaying)
            {
                typerampSwitcher.ApplyStyle();
            }
        }
        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(_typerampProperty, new GUIContent("Typeramp"));

            if (EditorGUI.EndChangeCheck())
            {
                TyperampSwitcher typerampSwitcher = (TyperampSwitcher)target;
                typerampSwitcher.Typeramp = (Typeramp)_typerampProperty.intValue;
                EditorUtility.SetDirty(typerampSwitcher);
            }

            serializedObject.ApplyModifiedProperties();
        }
    }
}
#endif
