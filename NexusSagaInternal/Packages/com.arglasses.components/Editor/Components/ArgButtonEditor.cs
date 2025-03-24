#if UNITY_EDITOR
using System.Reflection;
using ITK;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgButton))]
    [CanEditMultipleObjects]
    public class ArgButtonEditor : Editor
    {
        private bool _showEvents;
        private ArgButton targetComponent;
        
        private SerializedProperty interactableProp;
        
        private void OnEnable()
        {
            targetComponent = (ArgButton)target;
            
            interactableProp = serializedObject.FindProperty("_interactable");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            if (targetComponent.View == null)
            {
                base.OnInspectorGUI();
                return;
            }

            if (targetComponent.View.HasLabel)
            {
                EditorGUI.BeginChangeCheck();
                string newLabelText = EditorGUILayout.TextField("Label Text", targetComponent.ViewModel.LabelText);
                if (EditorGUI.EndChangeCheck())
                {
                    targetComponent.ViewModel.LabelText = newLabelText;
                }
            }

            if (targetComponent.View.HasIcon)
            {
                EditorGUI.BeginChangeCheck();
                if (!targetComponent.View.Toggleable && !targetComponent.View.Style.Stateful)
                {
                    Sprite newIcon = (Sprite)EditorGUILayout.ObjectField("Icon", targetComponent.ViewModel.Icon,
                        typeof(Sprite), false);
                    if (EditorGUI.EndChangeCheck())
                    {
                        targetComponent.ViewModel.Icon = newIcon;
                    }
                }
                else
                {
                    Sprite offIcon = (Sprite)EditorGUILayout.ObjectField("Off Icon", targetComponent.ViewModel.OffIcon,
                        typeof(Sprite), false);
                    
                    
                    Sprite onIcon = (Sprite)EditorGUILayout.ObjectField("On Icon", targetComponent.ViewModel.OnIcon,
                        typeof(Sprite), false);
                    
                    if (EditorGUI.EndChangeCheck())
                    {
                        targetComponent.ViewModel.OffIcon = offIcon;
                        targetComponent.ViewModel.OnIcon = onIcon;
                        
                        targetComponent.ViewModel.Icon =
                            targetComponent.ViewModel.Selected ? targetComponent.ViewModel.OnIcon : targetComponent.ViewModel.OffIcon;
                    }
                }
                
            }

            if (targetComponent.View.HasAppImage)
            {
                EditorGUI.BeginChangeCheck();
                Sprite newAppImage = (Sprite)EditorGUILayout.ObjectField("App Image", targetComponent.ViewModel.AppImage, typeof(Sprite), false);
                if (EditorGUI.EndChangeCheck())
                {
                    targetComponent.ViewModel.AppImage = newAppImage;
                }
            }

            // Draw the Interactable property
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(interactableProp);
            if (EditorGUI.EndChangeCheck())
            {
                targetComponent.Interactable = interactableProp.boolValue;
            }
            
            if (targetComponent.View.Toggleable)
            {
                EditorGUI.BeginChangeCheck();
                bool newSelected = EditorGUILayout.Toggle("Selected", targetComponent.ViewModel.Selected);
                if (EditorGUI.EndChangeCheck())
                {
                    targetComponent.ViewModel.Selected = newSelected;
                    targetComponent.ViewModel.Icon =
                        newSelected ? targetComponent.ViewModel.OnIcon : targetComponent.ViewModel.OffIcon;
                }
            }

            _showEvents = EditorGUILayout.Foldout(_showEvents,"Events");
            if(_showEvents)
                DrawGroupedProperties("Events");

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
