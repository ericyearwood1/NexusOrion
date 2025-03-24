using UnityEngine;

namespace ARGlasses.Components
{
    [RequireComponent(typeof(ArgTileButton))]
    public class TileButtonStyleEnforcer : StyleEnforcerBase<ArgTileButton, TileButtonViewModel>
    {
        [SerializeField] private TileButtonStyle _style;

        protected override void PopulateUse()
        {
        }

        protected override GameObject GetPrefab()
        {
            return ViewCollectionManager.TileButtonCollection.TryGetPrefabForStyle(_style, out var prefab)
                ? prefab
                : null;
        }
    }
}
