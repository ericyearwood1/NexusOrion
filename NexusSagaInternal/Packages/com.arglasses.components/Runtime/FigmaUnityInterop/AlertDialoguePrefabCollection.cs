using System.Linq;
using FigmaUnity.FigmaComponents;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "FigmaUnityComponentPrefabs", menuName = "FigmaUnity/ARDS/Alert Dialogue Component Collection", order = 0)]
    public class AlertDialoguePrefabCollection : ComponentPrefabCollection<AlertDialogueStyle>
    {
        public static readonly string ResourcesPath = "AlertDialoguePrefabs";

        [SerializeField] private GenericDictionary<AlertDialogueStyle, GameObject> _componentStyleToPrefabs;

        public override void AddComponentStyleFromString(string styleString)
        {
            var styleKey = new AlertDialogueStyle();
            styleKey.FromString(styleString);
            _componentStyleToPrefabs.Add(styleKey, null);
        }

        public override void AssignPrefabToStyle(string styleString, GameObject prefab)
        {
            var styleKey = new AlertDialogueStyle();
            styleKey.FromString(styleString);
            if (_componentStyleToPrefabs.ContainsKey(styleKey)) _componentStyleToPrefabs[styleKey] = prefab;
            else _componentStyleToPrefabs.Add(styleKey, prefab);
        }

        public override bool TryGetPrefabForStyle(AlertDialogueStyle style, out GameObject prefab)
        {
            prefab = _componentStyleToPrefabs.FirstOrDefault(x => x.Key.Layout == style.Layout).Value;
            return prefab != null;
        }

        public override bool ContainsKey(string styleString)
        {
            var styleKey = new AlertDialogueStyle();
            styleKey.FromString(styleString);
            return _componentStyleToPrefabs.ContainsKey(styleKey);
        }
    }
}
