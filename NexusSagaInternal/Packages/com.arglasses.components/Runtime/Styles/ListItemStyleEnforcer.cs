using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace ARGlasses.Components
{
    [RequireComponent(typeof(ArgListItem))]
    public class ListItemStyleEnforcer : StyleEnforcerBase<ArgListItem, ListItemViewModel>
    {
        [SerializeField] private ListItemStyle _style;
        public ListItemStyle Style => _style;

        public void SetStyleAndPopulate(ListItemStyle style)
        {
            _style = style;
            PopulatePrefab();
        }

        protected override GameObject GetPrefab()
        {
            return ViewCollectionManager.ListItemCollection.TryGetPrefabForStyle(_style, out var prefab)
                ? prefab
                : null;
        }

        protected override void PopulateUse()
        {
        }
    }
}
