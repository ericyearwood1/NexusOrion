#if UNITY_EDITOR
using System.Reflection;
using ITK;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgToggle))]
    public class ArgToggleEditor : Editor
    {
        private bool showBaseEvents;
        private bool showToggleEvents;

        public override void OnInspectorGUI()
        {
            // Access the target object (ArgToggle)
            ArgToggle argToggle = (ArgToggle)target;

            if (argToggle.View == null)
            {
                base.OnInspectorGUI();
                return;
            }

            // Set Value using Property
            argToggle.ViewModel.Value = EditorGUILayout.Toggle("Value", argToggle.ViewModel.Value);

            EditorGUILayout.Space();

            // Draw WhenValueChanged UnityEvent
            EditorGUILayout.PropertyField(serializedObject.FindProperty(ArgToggle.WhenValueChangedName));

            // Draw properties with the Group attribute under a dropdown
            showBaseEvents = EditorGUILayout.Foldout(showBaseEvents, "Base Events");
            if (showBaseEvents)
            {
                DrawGroupedProperties("Base Events");
            }

            showToggleEvents = EditorGUILayout.Foldout(showToggleEvents, "Toggle Events");
            if (showToggleEvents)
            {
                DrawGroupedProperties("Toggle Events");
            }

            serializedObject.ApplyModifiedProperties();
        }

        private void DrawGroupedProperties(string groupName)
        {
            BindingFlags bindingFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.FlattenHierarchy;
            FieldInfo[] fields = typeof(ArgButton).GetFields(bindingFlags);

            EditorGUI.indentLevel++;

            foreach (FieldInfo field in fields)
            {
                GroupAttribute attribute = field.GetCustomAttribute<GroupAttribute>();
                if (attribute != null && attribute.Header == groupName)
                {
                    SerializedProperty serializedProperty = serializedObject.FindProperty(field.Name);
                    if (serializedProperty != null)
                    {
                        EditorGUILayout.PropertyField(serializedProperty, new GUIContent(field.Name));
                    }
                }
            }

            EditorGUI.indentLevel--;
        }
    }
}
#endif
