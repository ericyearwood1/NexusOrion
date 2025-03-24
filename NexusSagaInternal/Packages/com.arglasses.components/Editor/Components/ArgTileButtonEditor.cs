#if UNITY_EDITOR
using System.Reflection;
using ITK;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgTileButton))]
    public class ArgTileButtonEditor : Editor
    {
        private bool showBaseEvents;
        private ArgTileButton targetComponent;
        private SerializedProperty viewModelProperty;
        private SerializedProperty spriteProperty;
        private SerializedProperty labelProperty;
        private SerializedProperty descProperty;
        private SerializedProperty valueProperty;

        private void OnEnable()
        {
            targetComponent = (ArgTileButton)target;
            viewModelProperty = serializedObject.FindProperty("ViewModel");
            spriteProperty = viewModelProperty.FindPropertyRelative("IconSprite");
            labelProperty = viewModelProperty.FindPropertyRelative("LabelText");
            descProperty = viewModelProperty.FindPropertyRelative("DescriptionText");
            valueProperty = viewModelProperty.FindPropertyRelative("Value");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            // Access the target object (ARGButton)
            ArgTileButton argTileButton = (ArgTileButton)target;

            if (argTileButton.View == null)
            {
                base.OnInspectorGUI();
                return;
            }


            EditorGUI.BeginChangeCheck();
            bool selected = EditorGUILayout.Toggle("Selected", argTileButton.ViewModel.Value);
            if (EditorGUI.EndChangeCheck())
            {
                argTileButton.ViewModel.Value = selected;
            }

            if (argTileButton.View.HasLabel)
            {
                EditorGUI.BeginChangeCheck();
                string newText = EditorGUILayout.TextField("Label", argTileButton.ViewModel.LabelText);
                if (EditorGUI.EndChangeCheck())
                {
                    argTileButton.ViewModel.LabelText = newText;
                }
            }

            if (argTileButton.View.HasDescription)
            {
                EditorGUI.BeginChangeCheck();
                string newDesc = EditorGUILayout.TextField("Description", argTileButton.ViewModel.DescriptionText);
                if (EditorGUI.EndChangeCheck())
                {
                    argTileButton.ViewModel.DescriptionText = newDesc;
                }
            }

            if (argTileButton.View.HasIcon)
            {
                EditorGUI.BeginChangeCheck();
                Sprite newSprite = (Sprite)EditorGUILayout.ObjectField("Icon", argTileButton.ViewModel.IconSprite,
                    typeof(Sprite), false);
                if (EditorGUI.EndChangeCheck())
                {
                    argTileButton.ViewModel.IconSprite = newSprite;
                }
            }

            // Draw properties without the Group attribute using default Inspector
            EditorGUILayout.PropertyField(serializedObject.FindProperty("WhenClick"));

            serializedObject.ApplyModifiedProperties();
        }

        private void DrawGroupedProperties(string groupName)
        {
            BindingFlags bindingFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic |
                                        BindingFlags.FlattenHierarchy;
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
