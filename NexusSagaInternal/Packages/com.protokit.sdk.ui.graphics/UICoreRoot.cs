#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.IO;

public class UICoreRoot : ScriptableObject {

    public string RootPath() {
        var ms = MonoScript.FromScriptableObject(this);
        return Path.GetDirectoryName(AssetDatabase.GetAssetPath(ms));
    }
}
#endif
