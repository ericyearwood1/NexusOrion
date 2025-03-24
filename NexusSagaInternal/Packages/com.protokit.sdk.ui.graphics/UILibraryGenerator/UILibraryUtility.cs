using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;

public static class UILibraryUtility {

    public static GameObject Instantiate(string guid, MenuCommand menuCommand, bool worldSpace = false) {
        var prefabPath = AssetDatabase.GUIDToAssetPath(guid);
        var o = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);

        GameObject instance;
        if (!CommonComponentsPreferences.InstantiateComponentsAsPrefabs) {
            instance = GameObject.Instantiate(o);
            instance.name = o.name;
        } else {
            instance = (GameObject)PrefabUtility.InstantiatePrefab(o);
        }

        if (instance == null) {
            return instance;
        }

        instance.name = o.name;

        // parent will be set by this method
        CanvasInstancer.PlaceUIElementRoot(instance, menuCommand);

        return instance;
    }
}
#endif
