using UnityEngine;
using UnityEditor;

namespace ARGlasses.Components
{
    public class ViewCollectionManagerEditor
    {
        [MenuItem("ARDS/Refresh Resource Assets")]
        public static void ReloadViewCollectionManagerResources()
        {
            AssetDatabase.Refresh();
            ViewCollectionManager.ResetResources();
            Debug.Log("View Collection Manager resources reset.");
        }
    }
}
