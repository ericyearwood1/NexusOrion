using System;
using UnityEditor;

namespace ARGlasses.Interaction
{
    public static class ExtensionsUnityEditor
    {
        public static string ToStringName(this BuildTarget target)
        {
            return Enum.GetName(typeof(BuildTarget), target);
        }

        public static bool EditorGetBool(string key, bool defaultValue = false)
        {
#if UNITY_EDITOR
            return EditorPrefs.GetBool(key, defaultValue);
#else
            return defaultValue;
#endif
        }

        public static void EditorSetBool(string key, bool value)
        {
#if UNITY_EDITOR
            EditorPrefs.SetBool(key, value);
#endif
        }

    }
}
