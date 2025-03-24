#if UNITY_EDITOR
using System.Reflection;
using ITK;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgAlertDialogue))]
    public class ArgAlertDialogueEditor : Editor
    {
        private bool showBaseEvents;

        public override void OnInspectorGUI()
        {
            ArgAlertDialogue argAlertDialogue = (ArgAlertDialogue)target;

            if (argAlertDialogue.View == null)
            {
                argAlertDialogue.PopulateDependencies();
                base.OnInspectorGUI();
                return;
            }

            var style = argAlertDialogue.View.Style;

            argAlertDialogue.ViewModel.Title = EditorGUILayout.TextField("Title Text", argAlertDialogue.ViewModel.Title);

            if (style.HasDescription)
            {
                argAlertDialogue.ViewModel.Desc = EditorGUILayout.TextField("Description Text", argAlertDialogue.ViewModel.Desc);
            }

            if (style.FromStyle == AlertDialogueFromStyle.FromApp)
            {
                argAlertDialogue.ViewModel.AppImage = (Sprite)EditorGUILayout.ObjectField("App Image", argAlertDialogue.ViewModel.AppImage, typeof(Sprite), allowSceneObjects: false);
            }

            if (style.ButtonType == ButtonType.Text)
            {
                var count = style.NumberOfButtons;
                if (count >= 1)
                    argAlertDialogue.ViewModel.ButtonText1 = EditorGUILayout.TextField("Button 1 Text", argAlertDialogue.ViewModel.ButtonText1);
                if (count >= 2)
                    argAlertDialogue.ViewModel.ButtonText2 = EditorGUILayout.TextField("Button 2 Text", argAlertDialogue.ViewModel.ButtonText2);
                if (count >= 3)
                    argAlertDialogue.ViewModel.ButtonText3 = EditorGUILayout.TextField("Button 3 Text", argAlertDialogue.ViewModel.ButtonText3);
            }

            if (style.ButtonType == ButtonType.Round)
            {
                var count = style.NumberOfButtons;
                if (count >= 1)
                    argAlertDialogue.ViewModel.ButtonIcon1 = (Sprite)EditorGUILayout.ObjectField("Button 1 Icon", argAlertDialogue.ViewModel.ButtonIcon1, typeof(Sprite), allowSceneObjects: false);
                if (count >= 2)
                    argAlertDialogue.ViewModel.ButtonIcon2 = (Sprite)EditorGUILayout.ObjectField("Button 2 Icon", argAlertDialogue.ViewModel.ButtonIcon2, typeof(Sprite), allowSceneObjects: false);
                if (count >= 3)
                    argAlertDialogue.ViewModel.ButtonIcon3 = (Sprite)EditorGUILayout.ObjectField("Button 3 Icon", argAlertDialogue.ViewModel.ButtonIcon3, typeof(Sprite), allowSceneObjects: false);
            }

            if(style.NumberOfButtons > 0)
            {
                // Draw properties with the Group attribute under a dropdown
                showBaseEvents = EditorGUILayout.Foldout(showBaseEvents, "Button Events");
                if (showBaseEvents)
                {
                    DrawGroupedProperties(argAlertDialogue, "Button Events");
                }
            }
        }

        private void DrawGroupedProperties(ArgAlertDialogue argAlertDialogue, string groupName)
        {
            BindingFlags bindingFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic |
                                        BindingFlags.FlattenHierarchy;
            FieldInfo[] fields = typeof(ArgAlertDialogue).GetFields(bindingFlags);

            EditorGUI.indentLevel++;

            foreach (FieldInfo field in fields)
            {
                GroupAttribute attribute = field.GetCustomAttribute<GroupAttribute>();
                if (attribute != null && attribute.Header == groupName)
                {
                    object fieldValue = field.GetValue(argAlertDialogue);
                    if (fieldValue is UnityEngine.Object obj)
                    {
                        EditorGUILayout.ObjectField(field.Name, obj, field.FieldType, allowSceneObjects: false);
                    }
                    else
                    {
                        EditorGUILayout.LabelField(field.Name, fieldValue.ToString());
                    }
                }
            }

            EditorGUI.indentLevel--;
        }
    }
}
#endif
