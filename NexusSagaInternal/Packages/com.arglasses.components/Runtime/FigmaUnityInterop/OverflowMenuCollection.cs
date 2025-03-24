using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "OverflowMenuPrefabs",
        menuName = "FigmaUnity/ARDS/Overflow Menu Collection", order = 0)]
    public class OverflowMenuCollection : ScriptableObject
    {
        public static readonly string ResourcesPath = "OverflowMenuCollection";

        public GenericDictionary<ArgOverflowMenu, Skin> OverflowMenuSettings;
    }
}
