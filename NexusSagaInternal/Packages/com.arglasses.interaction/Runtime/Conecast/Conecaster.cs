using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class Conecaster : MonoBehaviour
    {
        [SerializeField] private float _coneRadiusDegrees = 2f;

        public float ConeRadiusDegrees
        {
            get
            {
                if (!isActiveAndEnabled) return 0;
                return _coneRadiusDegrees;
            }
            set => _coneRadiusDegrees = value;
        }

        [SerializeField] private float _maxRayLength = 10f;
        public float MaxRayLength => _maxRayLength;

        [SerializeField, ReadOnly] private ConecastPrioritizer _prioritizer;
        public ConecastPrioritizer Prioritizer => _prioritizer;

        [SerializeField, ReadOnly] private List<ConecastResult> _orderedHits = new();
        public IReadOnlyList<ConecastResult> OrderedHits => _orderedHits;

        [SerializeField, ReadOnly] private ConecastResult _lastHit;
        public ConecastResult LastHit => _lastHit;

        public float RayDirectionOffset { get; set; }

        private void Awake()
        {
            this.Ensure(ref _prioritizer, relation: Relation.Descendant);
        }

        private void Update()
        {
        }

        public bool PrioritizeBestHit(Ray ray, IEnumerable<Target> targets, out ConecastResult bestHit)
        {
            if (RayDirectionOffset != 0)
            {
                ray.direction = Quaternion.AngleAxis(RayDirectionOffset, Vector3.up) * ray.direction;
                //does not reset itself?
                //RayDirectionOffset = 0;
            }

            Cone cone = new Cone(ray, ConeRadiusDegrees, MaxRayLength);
            _orderedHits.Clear();

            foreach (var target in targets)
            {
                if (target.HasSuppressors)
                {
                    Debug.LogError($"Suppressed Target attempting Conecast: {target.GetPath()}");
                    continue;
                }

                ConecastResult result = Conecast(cone, target);
                if (result.HasTarget) _orderedHits.Add(result);
            }

            _lastHit = bestHit = _prioritizer.FindBestResultInfo(_orderedHits, cone);
            return bestHit.HasTarget;
        }

        public Vector3 ClosestPoint(Vector3 point, Target target)
        {
            Vector3 closestPoint = default;
            var closestSqrMagnitude = float.MaxValue;

            foreach (var col in target.Colliders)
            {
                var candidatePoint = col.ClosestPoint(point);
                var sqrMagnitude = (candidatePoint - point).sqrMagnitude;
                if (closestPoint != default && sqrMagnitude > closestSqrMagnitude) continue;

                closestSqrMagnitude = sqrMagnitude;
                closestPoint = candidatePoint;
            }

            return closestPoint;
        }

        public ConecastResult Conecast(Cone cone, Target target)
        {
            for (var i = 0; i < target.Colliders.Count; i++)
            {
                var result = ConecastCollider(cone, target.Colliders[i], target);
                // todo need to pick the best result, not return whichever is first
                if (result.HasTarget) return result;
            }

            return ConecastResult.Empty(cone);
        }

        private static ConecastResult ConecastCollider(Cone cone, Collider collider, Target candidate)
        {
            if (!collider.gameObject.activeInHierarchy) return ConecastResult.Empty(cone);
            var surfacePlane = GetSurfacePlane(cone, collider);

            if (collider.Raycast(cone.ray, out var colliderHit, cone.maxDistance))
            {
                return CreateResult(cone, colliderHit.distance, collider, candidate, surfacePlane.normal);
            }

            if (surfacePlane.Raycast(cone.ray, out var hitDistance))
            {
                var result = CreateResult(cone, hitDistance, collider, candidate, surfacePlane.normal);
                if (result.gazeToColliderAngleDegrees <= cone.radiusDegrees) return result;
            }

            return ConecastResult.Empty(cone);
        }

        private const bool PerformRaycastForSurfacePlane = false;

        // todo we should be more explicit about the surface plane when authoring prefabs
        private static Plane GetSurfacePlane(Cone cone, Collider collider)
        {
            if (!PerformRaycastForSurfacePlane) return collider.transform.ToPlane();

            var toColliderCenter = new Ray(origin: cone.ray.origin, direction: (collider.transform.position - cone.ray.origin).normalized);
            // try to raycast the center of the Collider and build a plane from the resulting hit
            if (collider.Raycast(toColliderCenter, out var centerHit, float.MaxValue)) return new Plane(centerHit.normal, centerHit.point);

            // if not, create one from the transform
            var inPoint = collider.transform.position;
            var inNormal = (cone.ray.origin - inPoint).normalized;
            return new Plane(inNormal, inPoint);
        }

        private static ConecastResult CreateResult(Cone cone, float hitDistance, Collider collider, Target target, Vector3 hitNormal)
        {
            var conePoint = cone.ray.GetPoint(hitDistance);
            var nearestHit = collider.ClosestPointOnBounds(conePoint);
            var gazeToNearestHit = nearestHit - cone.ray.origin;
            var coneDistanceDegrees = Vector3.Angle(cone.ray.direction, gazeToNearestHit.normalized);

            return new()
            {
                target = target,
                component = collider,
                hitPoint = nearestHit,
                hitNormal = hitNormal,
                hitPointDistance = hitDistance,
                gazePoint = conePoint,
                gazeToColliderAngleDegrees = coneDistanceDegrees,
                sortOrder = target.SortOrder,
                cone = cone,
            };
        }
    }
}
