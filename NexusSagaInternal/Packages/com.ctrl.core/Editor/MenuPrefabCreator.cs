using UnityEngine;
using UnityEditor;

namespace CTRL.Inspector
{
  public static class PrefabUtil
  {
    public static void CreatePrefab(string path, GameObject context)
    {
      GameObject prefab = AssetDatabase.LoadAssetAtPath(path, typeof(GameObject)) as GameObject;
      GameObject go = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
      GameObjectUtility.SetParentAndAlign(go, context);
      Selection.activeObject = go;
    }
  }

}
