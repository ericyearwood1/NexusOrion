using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace ARGlasses.Components
{
    [CustomEditor(typeof(TileButtonStyleEnforcer))]
    public class TileButtonStyleSwitcherEditor : Editor
    {
        private void OnEnable()
        {
            if (!EditorApplication.isPlayingOrWillChangePlaymode && !EditorApplication.isPlaying)
            {
                SwitchPrefab();
            }
        }

        public override void OnInspectorGUI()
        {
            TileButtonStyleEnforcer styleEnforcer = (TileButtonStyleEnforcer)target;
            serializedObject.Update();

            // Draw other serialized properties if needed
            // EditorGUILayout.PropertyField(serializedObject.FindProperty("yourPropertyName"));

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
            TileButtonStyleEnforcer styleEnforcer = (TileButtonStyleEnforcer)target;

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
