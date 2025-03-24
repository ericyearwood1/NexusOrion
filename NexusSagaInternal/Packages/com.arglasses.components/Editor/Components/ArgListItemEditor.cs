#if UNITY_EDITOR
using System.Reflection;
using ITK;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgListItem))]
    public class ArgListItemEditor : Editor
    {
        private bool showBaseEvents;

        public override void OnInspectorGUI()
        {
            // Access the target object (ArgListItem)
            ArgListItem argListItem = (ArgListItem)target;

            if (argListItem.View == null)
                argListItem.ForceUpdateView();

            if (argListItem.View == null)
            {
                base.OnInspectorGUI();
                return;
            }

            if (argListItem.View.Toggleable)
            {
                argListItem.ViewModel.Selected = EditorGUILayout.Toggle("Selected", argListItem.ViewModel.Selected);
            }

            if (argListItem.View.HasLabel)
            {
                argListItem.ViewModel.LabelText = EditorGUILayout.TextField("Label Text", argListItem.ViewModel.LabelText);
            }

            if (argListItem.View.HasDescription)
            {
                argListItem.ViewModel.DescText = EditorGUILayout.TextField("Description Text", argListItem.ViewModel.DescText);
            }

            if (argListItem.View.HasIcon)
            {
                argListItem.ViewModel.IconSprite = (Sprite)EditorGUILayout.ObjectField("Icon Sprite", argListItem.ViewModel.IconSprite, typeof(Sprite), false);
            }

            if (argListItem.View.HasMedia)
            {
                argListItem.ViewModel.MediaSprite = (Sprite)EditorGUILayout.ObjectField("Media Sprite", argListItem.ViewModel.MediaSprite, typeof(Sprite), false);
            }

            if (argListItem.View.HasAvatar)
            {
                argListItem.ViewModel.AvatarSprite = (Sprite)EditorGUILayout.ObjectField("Avatar Sprite", argListItem.ViewModel.AvatarSprite, typeof(Sprite), false);
            }

            if (argListItem.View.HasRightText)
            {
                argListItem.ViewModel.RightText = EditorGUILayout.TextField("Right Text", argListItem.ViewModel.RightText);
            }

            // Draw other serialized properties
            DrawPropertiesExcluding(serializedObject, new string[] { "ViewModel" });

            // Draw properties with the Group attribute under a dropdown
            showBaseEvents = EditorGUILayout.Foldout(showBaseEvents, "Base Events");
            if (showBaseEvents)
            {
                DrawGroupedProperties("Base Events");
            }
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
