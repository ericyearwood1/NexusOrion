using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public struct Cone
    {
        public Ray ray;
        public float radiusDegrees;
        public float maxDistance;

        public Cone(Ray ray, float radiusDegrees, float maxDistance)
        {
            this.ray = ray;
            this.radiusDegrees = radiusDegrees;
            this.maxDistance = maxDistance;
        }

        public Pose EmptyHit => new Pose(ray.GetPoint(maxDistance), Quaternion.LookRotation(ray.direction * -1));
    }

    [Serializable]
    public struct ConecastResult
    {
        public enum SortLayer
        {
            Empty = -1000,
            Background = -10,
            Neutral = 0,
            Foreground = 10,
        }

        public Target target;
        public Component component;

        // gaze hit point on the hitbox in worldspace.  Guaranteed to always be on the surface of the hitbox.
        public Vector3 hitPoint;
        public Vector3 hitNormal;

        // distance from gaze origin to the hit point
        public float hitPointDistance;

        // the "true gaze" point.  For Direct hits, this will be the same as 'point'
        // For hits that are not direct but are within the cone, gazePoint will _not_ be on the hitbox surface.
        // It will be off to the side by angularDistanceToHitBox
        public Vector3 gazePoint;
        public float gazePointDistance => (gazePoint - cone.ray.origin).magnitude;

        // Distance in screenspace angular degrees between the "true gaze" and nearest point on the hitbox
        public float gazeToColliderAngleDegrees;

        // Force sorting, smaller values are lower priority
        public float sortOrder;

        public Cone cone;
        public bool HasTarget => target != null;

        // HitPose is the conecast-influenced hitpoint on the Collider
        public Pose HitPose => new Pose(hitPoint, Quaternion.LookRotation(hitNormal == default ? Vector3.forward : hitNormal));

        // GazeEndPose is the "true gaze" end of the Gaze ray
        public Pose GazeEndPose => new Pose(gazePoint, Quaternion.LookRotation(hitNormal == default ? Vector3.forward : hitNormal));
        public Cone Cone => cone;
        public bool IsDirectHit => gazeToColliderAngleDegrees <= 0;
        public string TargetPath => HasTarget ? target.transform.GetPath() : string.Empty;

        public static ConecastResult Empty(Cone cone)
        {
            var emptyHit = cone.EmptyHit;
            return new ConecastResult
            {
                target = default,
                component = default,
                hitPoint = emptyHit.position,
                hitNormal = emptyHit.forward,
                hitPointDistance = cone.maxDistance,
                gazePoint = emptyHit.position,
                gazeToColliderAngleDegrees = 0f,
                sortOrder = (int)SortLayer.Empty,
                cone = cone
            };
        }
    }
}
