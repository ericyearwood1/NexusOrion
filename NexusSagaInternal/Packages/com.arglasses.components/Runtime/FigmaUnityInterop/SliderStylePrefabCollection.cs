using System.Linq;
using FigmaUnity.FigmaComponents;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "FigmaUnityComponentPrefabs", menuName = "FigmaUnity/ARDS/Slider Component Collection", order = 0)]
    public class SliderStylePrefabCollection : ComponentPrefabCollection<SliderStyle>
    {
        [SerializeField] private GenericDictionary<SliderStyle, GameObject> _componentStyleToPrefabs;
        public static readonly string ResourcesPath = "SliderComponentPrefabs";

        public override void AddComponentStyleFromString(string styleString)
        {
            var styleKey = new SliderStyle();
            styleKey.FromString(styleString);
            _componentStyleToPrefabs.Add(styleKey, null);
        }

        public override void AssignPrefabToStyle(string styleString, GameObject prefab)
        {
            var styleKey = new SliderStyle();
            styleKey.FromString(styleString);
            if (_componentStyleToPrefabs.ContainsKey(styleKey)) _componentStyleToPrefabs[styleKey] = prefab;
            else _componentStyleToPrefabs.Add(styleKey, prefab);
        }

        public override bool TryGetPrefabForStyle(SliderStyle style, out GameObject prefab)
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
            var styleKey = new SliderStyle();
            styleKey.FromString(styleString);
            return _componentStyleToPrefabs.ContainsKey(styleKey);
        }
    }
}
