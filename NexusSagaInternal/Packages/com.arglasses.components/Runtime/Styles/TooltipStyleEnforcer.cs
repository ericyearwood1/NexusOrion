using TMPro;
using UnityEngine;

namespace ARGlasses.Components
{
    [RequireComponent(typeof(ArgTooltip))]
    public class TooltipStyleEnforcer : StyleEnforcerBase<ArgTooltip, TooltipViewModel>
    {
        [SerializeField] private TooltipStyle _style;

        protected override GameObject GetPrefab()
        {
            return ViewCollectionManager.TooltipStyleCollection.TryGetPrefabForStyle(_style, out var prefab)
                ? prefab
                : null;
        }

        public void SetStyleAndPopulate(TooltipStyle style)
        {
            _style = style;
            PopulatePrefab();
        }
        
        protected override void PopulateUse()
        {
        }
    }
}