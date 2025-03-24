using System.Linq;
using FigmaUnity.FigmaComponents;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "FigmaUnityComponentPrefabs", menuName = "FigmaUnity/ARDS/Tile Button Component Collection", order = 0)]
    public class TileButtonStylePrefabCollection : ComponentPrefabCollection<TileButtonStyle>
    {
        public static readonly string ResourcesPath = "TileButtonComponentPrefabs";

        [SerializeField] private GenericDictionary<TileButtonStyle, GameObject> _componentStyleToPrefabs;

        public override void AddComponentStyleFromString(string styleString)
        {
            var styleKey = new TileButtonStyle();
            styleKey.FromString(styleString);
            _componentStyleToPrefabs.Add(styleKey, null);
        }

        public override void AssignPrefabToStyle(string styleString, GameObject prefab)
        {
            var styleKey = new TileButtonStyle();
            styleKey.FromString(styleString);
            if (_componentStyleToPrefabs.ContainsKey(styleKey)) _componentStyleToPrefabs[styleKey] = prefab;
            else _componentStyleToPrefabs.Add(styleKey, prefab);
        }

        public override bool TryGetPrefabForStyle(TileButtonStyle style, out GameObject prefab)
        {
            if (!_componentStyleToPrefabs.ContainsKey(style))
            {
                prefab = null;
                return false;
            }

            prefab = _componentStyleToPrefabs[style];
            return true;
        }

        public override bool ContainsKey(string styleString)
        {
            var styleKey = new TileButtonStyle();
            styleKey.FromString(styleString);
            return _componentStyleToPrefabs.ContainsKey(styleKey);
        }
    }
}
