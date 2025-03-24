using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public abstract class InputContext : MonoBehaviour
    {
        public virtual bool SuppressCv => false;
        public virtual bool SuppressWristband => false;

        public abstract void Open(in Snapshot.Rig rigSnapshot, TargetContext targetContext);
        public abstract Snapshot.Focus GetFocus(in Snapshot.Rig rigSnapshot);
        public abstract void Close(in Snapshot.Rig rigSnapshot, TargetContext closingTargetContext);
        public abstract Vector3 CursorWorld { get; }
        public abstract TargetContext TargetContext { get; }
        public string Name => Category.ToString();
        public abstract InputCategory Category { get; }

        public virtual Drag CreateDrag(InteractionState latest, InteractionState begin)
        {
            var useImu = !SuppressWristband && latest.IsWristbandConnected;
            return useImu ? new ImuDrag(latest, begin) : new Drag(latest, begin);
        }

        public virtual TargetContext BestSurface(in Snapshot.Rig rigSnapshot) => ConecastTargetContext(rigSnapshot.Gaze.Head.ToRay()).Context;

        [SerializeField, ReadOnly] private List<TargetContext.RaycastHit> _targetContextHits = new();

        protected TargetContext.RaycastHit ConecastTargetContext(Ray ray, IReadOnlyList<TargetContext> targetContexts = null, float coneAngleDegrees = 30)
        {
            if (targetContexts == null) targetContexts = TargetContext.All;

            _targetContextHits.Clear();
            foreach (var targetContext in targetContexts)
            {
                if (!targetContext.isActiveAndEnabled) continue;
                var hit = targetContext.RaycastWorldSpace(ray);
                if (hit.IsPlaneHit && hit.SnapAngle <= coneAngleDegrees) _targetContextHits.Add(hit);
            }

            _targetContextHits.Sort((a, b) =>
            {
                var angleCompare = a.SnapAngle.CompareTo(b.SnapAngle);
                if (angleCompare != 0) return angleCompare;

                return a.RayToPlaneHitDistance.CompareTo(b.RayToPlaneHitDistance);
            });

            return _targetContextHits.FirstOrDefault();
        }


        public bool IsPinching(in Snapshot.Rig snapshot, in Side side, in HandFinger finger)
        {
            var cvHand = snapshot.GetHand(side);
            var wristband = snapshot.Wristband;
            var suppressCv = wristband.IsConnected || SuppressCv;
            var suppressWristband = !wristband.IsConnected || SuppressWristband;
            var cvPinching = !suppressCv && cvHand.IsPinching(finger);
            var wristbandPinching = !suppressWristband && wristband.IsPinching(side, finger);
            return cvPinching || wristbandPinching;
        }
    }
}
