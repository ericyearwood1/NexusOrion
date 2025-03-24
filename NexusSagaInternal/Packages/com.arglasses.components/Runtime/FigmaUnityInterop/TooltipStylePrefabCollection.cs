using System.Linq;
using FigmaUnity.FigmaComponents;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "TooltipComponentPrefabs", menuName = "FigmaUnity/ARDS/Tooltip Component Collection", order = 0)]
    public class TooltipStylePrefabCollection : ComponentPrefabCollection<TooltipStyle>
    {
        [SerializeField] private GenericDictionary<TooltipStyle, GameObject> _componentStyleToPrefabs;
        public static string ResourcesPath = "TooltipComponentPrefabs";


        public override bool TryGetPrefabForStyle(TooltipStyle style, out GameObject prefab)
        {
            if (_componentStyleToPrefabs.ContainsKey(style))
            {
                prefab = _componentStyleToPrefabs[style];
                return true;
            }

            prefab = null;
            return false;
        }

        public override void AddComponentStyleFromString(string styleString)
        {
            var styleKey = new TooltipStyle();
            styleKey.FromString(styleString);
            _componentStyleToPrefabs.Add(styleKey, null);
        }

        public override bool ContainsKey(string styleString)
        {
            var styleKey = new TooltipStyle();
            styleKey.FromString(styleString);
            return _componentStyleToPrefabs.ContainsKey(styleKey);
        }

        public override void AssignPrefabToStyle(string styleString, GameObject prefab)
        {
            var styleKey = new TooltipStyle();
            styleKey.FromString(styleString);
            if (_componentStyleToPrefabs.ContainsKey(styleKey)) _componentStyleToPrefabs[styleKey] = prefab;
            else _componentStyleToPrefabs.Add(styleKey, prefab);
        }
    }
}