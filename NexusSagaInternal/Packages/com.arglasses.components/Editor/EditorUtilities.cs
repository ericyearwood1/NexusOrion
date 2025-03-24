#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace ARGlasses.Components
{
    public static class EditorUtilities
    {
        public static bool IsInSceneAndNotInPrefabEditMode(GameObject target)
        {
            if (target == null) return false;

            if (!EditorUtility.IsPersistent(target))
            {
                PrefabStage prefabStage = PrefabStageUtility.GetCurrentPrefabStage();
                if (prefabStage == null || (prefabStage.prefabContentsRoot != target.gameObject &&
                                            !target.transform.IsChildOf(prefabStage.prefabContentsRoot.transform)))
                {
                    return true;
                }
            }

            return false;
        }
    }
}
#endif
