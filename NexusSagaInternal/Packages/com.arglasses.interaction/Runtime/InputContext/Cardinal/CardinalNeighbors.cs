using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CardinalNeighbors : MonoBehaviour
    {
        public Component _up;
        public Component _down;
        public Component _left;
        public Component _right;
        [SerializeField, ReadOnly] private Target _foundTarget;

        public static bool TryGetExplicitNeighborFromParent(Transform child, DPad dPad, out Target found)
        {
            found = null;
            var explicitNeighbors = child.parent.GetComponentInParent<CardinalNeighbors>();
            if (!explicitNeighbors) return false;

            return explicitNeighbors.TryGetNeighbor(dPad, out found);
        }

        public bool TryGetNeighbor(DPad dPad, out Target foundTarget)
        {
            foundTarget = null;
            Component neighbor = null;
            if (dPad.IsUp()) neighbor = _up;
            if (dPad.IsDown()) neighbor = _down;
            if (dPad.IsLeft()) neighbor = _left;
            if (dPad.IsRight()) neighbor = _right;
            if (!neighbor) return TryGetExplicitNeighborFromParent(transform, dPad, out foundTarget);

            // neighbor.Descendant(ref foundTarget, optional: true);
            foundTarget = neighbor.GetComponentInChildren<Target>();
            _foundTarget = foundTarget;
            if (!foundTarget) return false;
            return true;
        }
    }
}
