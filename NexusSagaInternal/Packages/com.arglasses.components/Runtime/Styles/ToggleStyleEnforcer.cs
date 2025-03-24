using UnityEngine;

namespace ARGlasses.Components
{
    [RequireComponent(typeof(ArgToggle))]
    public class ToggleStyleEnforcer : StyleEnforcerBase<ArgToggle, ToggleViewModel>
    {
        [SerializeField] private ToggleStyle _style;

        protected override GameObject GetPrefab()
        {
            return ViewCollectionManager.ToggleStyleCollection.TryGetPrefabForStyle(_style, out var prefab)
                ? prefab
                : null;
        }

        protected override void PopulateUse()
        {
        }
    }
}
