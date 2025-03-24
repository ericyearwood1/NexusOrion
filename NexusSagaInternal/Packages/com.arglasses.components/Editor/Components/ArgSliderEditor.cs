#if UNITY_EDITOR
using System.Reflection;
using ITK;
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgSlider))]
    public class ArgSliderEditor : Editor
    {
        private bool showSliderEvents;
        private ArgSlider targetSlider;

        private void OnEnable()
        {
            targetSlider = (ArgSlider)target;
        }

        public override void OnInspectorGUI()
        {
            if (targetSlider.View == null)
            {
                base.OnInspectorGUI();
                return;
            }

            if (targetSlider.View.Style.SliderType == SliderType.Icon)
            {
                targetSlider.ViewModel.IconSprite = (Sprite)EditorGUILayout.ObjectField("Icon Sprite", targetSlider.ViewModel.IconSprite, typeof(Sprite), false);
            }

            targetSlider.ViewModel.MinValue = EditorGUILayout.FloatField("Min Value", targetSlider.ViewModel.MinValue);
            targetSlider.ViewModel.MaxValue = EditorGUILayout.FloatField("Max Value", targetSlider.ViewModel.MaxValue);
            targetSlider.ViewModel.Interval = EditorGUILayout.FloatField("Interval", targetSlider.ViewModel.Interval);
            targetSlider.ViewModel.Value = EditorGUILayout.Slider("Value", targetSlider.ViewModel.Value, targetSlider.ViewModel.MinValue, targetSlider.ViewModel.MaxValue);

            showSliderEvents = EditorGUILayout.Foldout(showSliderEvents, "Slider Events");
            if (showSliderEvents)
            {
                DrawGroupedProperties("Slider Events");
            }

            serializedObject.Update();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("WhenValueChanged"));
            serializedObject.ApplyModifiedProperties();
        }

        private void DrawGroupedProperties(string groupName)
        {
            BindingFlags bindingFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic |
                                        BindingFlags.FlattenHierarchy;
            FieldInfo[] fields = typeof(ArgSlider).GetFields(bindingFlags);

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
