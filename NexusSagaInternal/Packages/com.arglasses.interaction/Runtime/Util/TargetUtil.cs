using System.Collections.Generic;
using System.Linq;
using ARGlasses.Interaction;
using UnityEngine;

namespace ARGlasses.R7
{
    public static class TargetUtil
    {
        public static void SetTargetsActive(Component suppressor, Transform parent, bool setEnabled, IEnumerable<Transform> ignore = null, Relation relation = Relation.Descendant)
        {
            bool ShouldIgnore(Target target)
            {
                return ignore != null && ignore.Any(node => target.transform.IsChildOf(node));
            }

            if (relation != Relation.Descendant)
            {
                Debug.LogError("ToggleTargets not implmeneted for non-Descendant relation type, let naschenbach know you want it");
                return;
            }

            foreach (Target target in parent.GetComponentsInChildren<Target>(includeInactive: true))
            {
                if (ShouldIgnore(target)) continue;

                if (setEnabled) target.RemoveSuppressor(suppressor);
                else target.AddSuppressor(suppressor);
            }

            // foreach (var context in parent.GetComponentsInChildren<TargetContext>(includeInactive: true)) context.enabled = setEnabled;
        }

    }
}
