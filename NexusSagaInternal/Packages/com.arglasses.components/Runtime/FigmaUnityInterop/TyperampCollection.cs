using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "FigmaUnityComponentPrefabs",
        menuName = "FigmaUnity/ARDS/Typeramp Collection", order = 0)]
    public class TyperampCollection : ScriptableObject
    {
        public static readonly string ResourcesPath = "TyperampCollection";

        public GenericDictionary<Typeramp, TypeSettings> TyperampSettings;
    }
}
