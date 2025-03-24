#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(ArgTooltip))]
    public class ArgTooltipEditor : Editor
    {
        private ArgTooltip targetComponent;
        private SerializedProperty viewModelProperty;
        private SerializedProperty spriteProperty;
        private SerializedProperty titleProperty;
        private SerializedProperty descProperty;

        private void OnEnable()
        {
            targetComponent = (ArgTooltip)target;
            viewModelProperty = serializedObject.FindProperty("ViewModel");
            spriteProperty = viewModelProperty.FindPropertyRelative("IconSprite");
            titleProperty = viewModelProperty.FindPropertyRelative("TitleText");
            descProperty = viewModelProperty.FindPropertyRelative("DescriptionText");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            // Access the target object (ARGButton)
            ArgTooltip tooltip = (ArgTooltip)target;

            if (tooltip.View == null)
            {
                base.OnInspectorGUI();
                return;
            }


            EditorGUI.BeginChangeCheck();

            if (tooltip.View.Style.HasIcon)
            {
                EditorGUI.BeginChangeCheck();
                Sprite newSprite = (Sprite)EditorGUILayout.ObjectField("Icon", tooltip.ViewModel.IconSprite,
                    typeof(Sprite), false);
                if (EditorGUI.EndChangeCheck())
                {
                    tooltip.ViewModel.IconSprite = newSprite;
                }
            }
            
            
            if (tooltip.View.Style.HasTitle)
            {
                EditorGUI.BeginChangeCheck();
                string newText = EditorGUILayout.TextField("Title", tooltip.ViewModel.TitleText);
                if (EditorGUI.EndChangeCheck())
                {
                    tooltip.ViewModel.TitleText = newText;
                }
            }


            EditorGUI.BeginChangeCheck();
            string newDesc = EditorGUILayout.TextField("Description", tooltip.ViewModel.DescriptionText);
            if (EditorGUI.EndChangeCheck())
            {
                tooltip.ViewModel.DescriptionText = newDesc;
            }
            

            serializedObject.ApplyModifiedProperties();
        }
    }
}
#endif
