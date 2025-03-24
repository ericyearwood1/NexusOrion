using System.Linq;
using FigmaUnity.FigmaComponents;
using UnityEngine;

namespace ARGlasses.Components
{
    [CreateAssetMenu(fileName = "FigmaUnityComponentPrefabs", menuName = "FigmaUnity/ARDS/Button Component Collection",
        order = 0)]
    public class ButtonStylePrefabCollection : ComponentPrefabCollection<ButtonStyle>
    {
        public static readonly string ResourcesPath = "ButtonComponentPrefabs";
        [SerializeField] private GenericDictionary<ButtonStyle, GameObject> _componentStyleToPrefabs;

        public override void AddComponentStyleFromString(string styleString)
        {
            var styleKey = new ButtonStyle();
            styleKey.FromString(styleString);
            _componentStyleToPrefabs.Add(styleKey, null);
        }

        public override void AssignPrefabToStyle(string styleString, GameObject prefab)
        {
            var styleKey = new ButtonStyle();
            styleKey.FromString(styleString);
            _componentStyleToPrefabs[styleKey] = prefab;
        }

        public override bool TryGetPrefabForStyle(ButtonStyle style, out GameObject prefab)
        {
            bool MatchCondition(ButtonStyle buttonStyle)
            {
                if (style.ButtonType == ButtonType.Text)
                {
                    return buttonStyle.ButtonType == style.ButtonType && buttonStyle.Skin == style.Skin &&
                           buttonStyle.Icon == style.Icon;
                }

                //app and round button type
                return buttonStyle.ButtonType == style.ButtonType && buttonStyle.Skin == style.Skin &&
                       buttonStyle.HideLabel == style.HideLabel;
            }

            var firstMatchingButtonStyle = _componentStyleToPrefabs.Keys.FirstOrDefault(MatchCondition);
            return _componentStyleToPrefabs.TryGetValue(firstMatchingButtonStyle, out prefab);
        }

        public override bool ContainsKey(string styleString)
        {
            var styleKey = new ButtonStyle();
            styleKey.FromString(styleString);
            return _componentStyleToPrefabs.ContainsKey(styleKey);
        }

        public bool TryGetPrefabForStyle(ButtonStyle style, Use use, out GameObject prefab)
        {
            var stateFul = style.Stateful || use is Use.Segmented or Use.FlatSegmented or Use.Toggle;

            bool MatchCondition(ButtonStyle collectionStyle)
            {
                if (style.ButtonType == ButtonType.Text)
                {
                    return collectionStyle.ButtonType == style.ButtonType && collectionStyle.Skin == style.Skin &&
                           collectionStyle.Icon == style.Icon && collectionStyle.Stateful == stateFul;
                }

                //app and round button type
                return collectionStyle.ButtonType == style.ButtonType && collectionStyle.Skin == style.Skin &&
                       collectionStyle.HideLabel == style.HideLabel && collectionStyle.Stateful == stateFul;
            }


            var firstMatchingButtonStyle = _componentStyleToPrefabs.Keys.FirstOrDefault(MatchCondition);
            return _componentStyleToPrefabs.TryGetValue(firstMatchingButtonStyle, out prefab);
        }
    }
}
