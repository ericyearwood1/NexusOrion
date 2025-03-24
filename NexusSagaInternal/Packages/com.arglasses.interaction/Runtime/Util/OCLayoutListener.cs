using System;
using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using UnityEngine;

// stupid hack to prevent builds from breaking when Utility namespace is included
namespace OSIG.Tools.Utility
{
}

namespace ARGlasses.Interaction
{
    public class OCLayoutListener : MonoBehaviour, IOCLayoutListener, IOCUnitsContextCache
    {
        [SerializeField, ReadOnly] private IOCLayoutComponent _ocLayoutComponent;
        public IOCLayoutComponent OCLayoutComponent => _ocLayoutComponent;

        public event Action<IOCLayoutComponent> WhenApplyLayout = delegate {  };
        public OCLayoutComponentBase _ocLayout;

        public void OnApplyLayout(IOCLayoutComponent layoutComponent)
        {
            _ocLayoutComponent = layoutComponent;
            WhenApplyLayout(_ocLayoutComponent);
        }

        public void OnUnitsContextDirtied()
        {
            // ignored for now
        }

        private void Awake()
        {
            this.Ensure(ref _ocLayout, defaultType: typeof(OCLayoutInset)).SetCenterFill();
            EnsureOCLayoutChainToNearestAncestor(this);
        }

        private static void EnsureOCLayoutChainToNearestAncestor(Component component)
        {

            var current = component.transform.parent;
            while (current != null)
            {
                if (current.TryGetComponent<OCLayoutComponentBase>(out _)) return;

                current.Ensure<OCLayoutComponentBase>(defaultType: typeof(OCLayoutInset)).SetCenterFill();
                current = current.parent;
            }
        }

        // protected virtual void OnEnable() => WhenApplyLayout();
        // public void OnUnitsContextDirtied() => WhenApplyLayout();
        // protected virtual void OnTransformParentChanged() => WhenApplyLayout();
        // protected virtual void OnRectTransformDimensionsChange() => WhenApplyLayout();
    }
}
