using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
namespace ARGlasses.Components
{
    [CustomEditor(typeof(ButtonStyleEnforcer))]
    [CanEditMultipleObjects]
    public class ButtonStyleEnforcerEditor : Editor
    {
        private ButtonStyleEnforcer styleEnforcer;
        private SerializedProperty currentStyleProperty;
        private SerializedProperty currentUseProperty;
        private SerializedProperty statefulProperty;
        private void OnEnable()
        {
            styleEnforcer = (ButtonStyleEnforcer)target;
            currentStyleProperty = serializedObject.FindProperty("_style");
            currentUseProperty = serializedObject.FindProperty("_use");
            statefulProperty = serializedObject.FindProperty("Stateful");
            
            if (!EditorApplication.isPlayingOrWillChangePlaymode && !EditorApplication.isPlaying)
            {
                SwitchPrefab();
            }
        }
        
        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUILayout.PropertyField(currentStyleProperty, new GUIContent("Style"));
            EditorGUILayout.PropertyField(currentUseProperty, new GUIContent("Use"));
            
            if (currentUseProperty.intValue == (int)Use.Custom) // Custom use
            {
                EditorGUILayout.PropertyField(statefulProperty, new GUIContent("Stateful"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("_customColorDefinition"), new GUIContent("Custom Color Definition"));
            }
            
            // Check if any changes were made
            if (GUI.changed)
            {
                serializedObject.ApplyModifiedProperties();
                SwitchPrefab();
            }
        }
        
        private void SwitchPrefab()
        {
            if (EditorUtilities.IsInSceneAndNotInPrefabEditMode(styleEnforcer.gameObject))
            {
                styleEnforcer.PopulatePrefab();
            }
            else
            {
                GameObject root = styleEnforcer.gameObject.transform.root.gameObject;
                PrefabStage prefabStage = PrefabStageUtility.GetCurrentPrefabStage();
                if (prefabStage != null)
                    root = prefabStage.prefabContentsRoot;
                if (root != styleEnforcer.gameObject)
                {
                    styleEnforcer.PopulatePrefab();
                }
            }
        }
    }
}