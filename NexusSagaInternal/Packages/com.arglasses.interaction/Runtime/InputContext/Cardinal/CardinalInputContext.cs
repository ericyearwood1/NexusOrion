using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CardinalInputContext : InputContext
    {
        [Serializable]
        public struct NeighborRaycastHit
        {
            [SerializeField, ReadOnly] internal Target _source;
            [SerializeField, ReadOnly] internal Target _destination;
            [SerializeField, ReadOnly] internal Ray _ray;
            [SerializeField, ReadOnly] internal RaycastHit _hitInfo;
            public Target Source => _source;
            public Target Destination => _destination;
            public bool HasDestination => _destination != null;
            public Ray Ray => _ray;
            public RaycastHit HitInfo => _hitInfo;
        }

        public override InputCategory Category => InputCategory.Cardinal;

        [SerializeField, ReadOnly] public TargetContext _targetContext;
        public override TargetContext TargetContext => _targetContext;

        private FreeMovingCardinalVisual _cardinalFocusVisual;
        public override Vector3 CursorWorld => _targetContext.ToWorldSpace(_lastFocusPositionLocal);

        [SerializeField, ReadOnly] private Target _focused;

        private void SetFocus(Target focused)
        {
            _focused = focused;
            _lastFocusPositionLocal = _targetContext.transform.InverseTransformPoint(_focused.transform.position);
        }

        [SerializeField, ReadOnly] private NeighborRaycastHit _lastNeighborRaycastHit;
        public NeighborRaycastHit LastNeighborRaycastHit => _lastNeighborRaycastHit;

        [SerializeField, ReadOnly] private Vector3 _lastFocusPositionLocal;

        private void Awake()
        {
            this.Ensure(ref _cardinalFocusVisual);
            _cardinalFocusVisual.enabled = false;
        }

        public override void Open(in Snapshot.Rig rigSnapshot, TargetContext targetContext)
        {
            _cardinalFocusVisual.enabled = true;
            _targetContext = targetContext;
            _focused = null;
        }

        public override void Close(in Snapshot.Rig rigSnapshot, TargetContext closingTargetContext)
        {
            _cardinalFocusVisual.enabled = false;
            _targetContext = null;
            _focused = null;
            _lastNeighborRaycastHit = default;
        }

        public override Snapshot.Focus GetFocus(in Snapshot.Rig rigSnapshot)
        {
            var rightSideOnly = Side.Right;
            var isPressingPrimary = IsPinching(rigSnapshot, rightSideOnly, HandFinger.Index);
            var isPressingSecondary = IsPinching(rigSnapshot, rightSideOnly, HandFinger.Middle);
            var empty = Snapshot.Focus.Empty(rightSideOnly, isPressingPrimary, isPressingSecondary);

            var dPad = rigSnapshot.Wristband.DPad;
            var surfaceTargets = _targetContext.ActiveTargets;

            if (_focused == null || !surfaceTargets.Contains(_focused))
            {
                if (_targetContext.FindNearestTarget(_lastFocusPositionLocal, out var found)) SetFocus(found);
            }

            if (_focused == null) return empty;


            if (dPad.IsNone()) return new Snapshot.Focus(_focused, rightSideOnly, isPressingPrimary: isPressingPrimary, isPressingSecondary: isPressingSecondary);

            if (CardinalNeighbors.TryGetExplicitNeighborFromParent(_focused.transform, dPad, out var neighbor))
            {
                SetFocus(neighbor);
            }
            else if (TryRaycastNeighbor(_focused, dPad, surfaceTargets, out var neighborRaycastHit))
            {
                SetFocus(neighborRaycastHit.Destination);
                _lastNeighborRaycastHit = neighborRaycastHit;
            }

            return !_focused ? empty : new Snapshot.Focus(_focused, rightSideOnly, isPressingPrimary: isPressingPrimary, isPressingSecondary: isPressingSecondary);
        }

        private bool TryRaycastNeighbor(Target interactable, DPad dPad, IEnumerable<Target> targets, out NeighborRaycastHit newFocusState)
        {
            newFocusState = default;
            var t = interactable.transform;
            var nudge = t.forward * 0.005f;
            var neighborRay = new Ray(t.position + nudge, t.TransformVector(dPad.ToVector2()));
            Debug.DrawRay(neighborRay.origin, neighborRay.direction);
            float minDist = float.MaxValue;
            foreach (var neighbor in targets)
            {
                if (neighbor == interactable) continue;

                foreach (var col in neighbor.Colliders)
                {
                    if (col.Raycast(neighborRay, out var neighborHitInfo, float.MaxValue) && minDist > neighborHitInfo.distance)
                    {
                        minDist = neighborHitInfo.distance;
                        newFocusState._destination = neighbor;
                        newFocusState._hitInfo = neighborHitInfo;
                        newFocusState._ray = neighborRay;
                    }
                }
            }

            return newFocusState.HasDestination;
        }
    }
}
