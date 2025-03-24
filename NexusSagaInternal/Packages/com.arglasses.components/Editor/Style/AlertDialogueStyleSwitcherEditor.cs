using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace ARGlasses.Components
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(AlertDialogueStyleEnforcer))]
    public class AlertDialogueStyleSwitcherEditor : Editor
    {
        private void OnEnable()
        {
            if (!EditorApplication.isPlaying)
            {
                SwitchPrefab();
            }
        }

        public override void OnInspectorGUI()
        {
            AlertDialogueStyleEnforcer styleEnforcer = (AlertDialogueStyleEnforcer)target;
            serializedObject.Update();

            EditorGUI.BeginChangeCheck();
            SerializedProperty currentStyleProperty = serializedObject.FindProperty("_style");
            EditorGUILayout.PropertyField(currentStyleProperty, new GUIContent("Style"));
            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
                SwitchPrefab();
            }
        }

        private void SwitchPrefab()
        {
            AlertDialogueStyleEnforcer styleEnforcer = (AlertDialogueStyleEnforcer)target;

            if (EditorUtilities.IsInSceneAndNotInPrefabEditMode(styleEnforcer.gameObject))
            {
                // target is in the scene (not persistent) and not in prefab edit mode, execute the code
                styleEnforcer.PopulatePrefab();
            }
            else //or target is in prefab mode, but it's a nested prefab
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
