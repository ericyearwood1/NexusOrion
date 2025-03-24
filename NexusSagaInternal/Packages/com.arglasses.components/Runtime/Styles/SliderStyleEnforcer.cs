using UnityEngine;

namespace ARGlasses.Components
{
    [RequireComponent(typeof(ArgSlider))]
    public class SliderStyleEnforcer : StyleEnforcerBase<ArgSlider, SliderViewModel>
    {
        [SerializeField] private SliderStyle _style;

        protected override GameObject GetPrefab()
        {
            return ViewCollectionManager.SliderStyleCollection.TryGetPrefabForStyle(_style, out var prefab)
                ? prefab
                : null;
        }

        protected override void PopulateUse()
        {
        }
    }
}
