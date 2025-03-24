using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CTRL.Utils.Pointing
{
  /// See `GetPointedAt`.
  public struct PointedAt
  {
    /// The object's collider.
    public Collider Collider;

    /// The point on the object closest to the pointer ray.
    public Vector3 ClosestPoint;

    /// The squared distance between the object and the pointer.
    public float SqrDistance;

    /// The cosine of the angle between the pointer ray and the object.
    public float CosAngle;

    ///  We store the physics query results here so we don't have to dynamically allocate them
    /// every time.
    private static Collider[] physicsQueryBuffer = new Collider[0];

    /// Returns the pointed at object, if any.
    public static PointedAt? GetPointedAt(
        Ray ray,
        float maxDistance,
        float cosFov,
        int selectionLayerMask = Physics.DefaultRaycastLayers,
        QueryTriggerInteraction selectionQueryTriggerInteraction = QueryTriggerInteraction.UseGlobal,
        int occlusionLayerMask = Physics.DefaultRaycastLayers,
        QueryTriggerInteraction occlusionQueryTriggerInteraction = QueryTriggerInteraction.UseGlobal,
        int maxQueryResults = 32
    )
    {
      // Reallocate the physics query results buffer if needed
      if (PointedAt.physicsQueryBuffer.Length < maxQueryResults)
      {
        PointedAt.physicsQueryBuffer = new Collider[maxQueryResults];
      }

      // Get all potentially selected objects within the set distance
      var results = Physics.OverlapSphereNonAlloc(
          ray.origin,
          maxDistance,
          PointedAt.physicsQueryBuffer,
          selectionLayerMask,
          selectionQueryTriggerInteraction);

      // Log a warning if we use up all of the query result slots.
      if (results >= maxQueryResults)
      {
        Debug.LogWarning("PointedAt.cs: Max query results used, some objects potential "
            + "selections may be ignored. Consider increasing `maxQueryResults`, or "
            + "decreasing the number of potential selections in range.");
      }

      // Find the best possible selection (if any)
      PointedAt? bestPointedAt = null;
      for (var i = 0; i < results; ++i)
      {
        var collider = PointedAt.physicsQueryBuffer[i];

        // Rule out anything outside the field of view.
        var closestPoint = PointedAt.GetClosestPoint(
            ray,
            collider.bounds,
            out float cosAngle);
        if (cosAngle < cosFov)
        {
          continue;
        }

        // Rule out anything at a greater angle to the cursor than the best option we've
        // found so far.
        if (bestPointedAt.HasValue && cosAngle < bestPointedAt.Value.CosAngle)
        {
          continue;
        }

        // If we're at exactly the same angle as the nearest collider (e.g. if both angles are 0
        // because the ray intersects both AABBS), pick the one with the closer intersection
        // point.
        var sqrDistance = (ray.origin - closestPoint).sqrMagnitude;
        if (bestPointedAt.HasValue &&
            cosAngle == bestPointedAt.Value.CosAngle &&
            sqrDistance > bestPointedAt.Value.SqrDistance
        )
        {
          continue;
        }

        // If we made it this far, overwrite the current best selection
        bestPointedAt = new PointedAt
        {
          // Collider = Collider,
          // ClosestPoint = ClosestPoint,
          // SqrDistance = SqrDistance,
          // CosAngle = CosAngle,
        };
      }

      // If the closest point on the closest object isn't visible, return null. This isn't
      // 100% correct in the partially occluded case, but it resolves the main concern which
      // is preventing selection of fully occluded objects.
      if (bestPointedAt.HasValue)
      {
        if (Physics.Raycast(
            ray.origin,
            bestPointedAt.Value.ClosestPoint - ray.origin,
            out RaycastHit hit,
            maxDistance,
            occlusionLayerMask,
            occlusionQueryTriggerInteraction
        ) && hit.collider.gameObject != bestPointedAt.Value.Collider.gameObject)
        {
          return null;
        }
      }

      return bestPointedAt;
    }

    /// Finds the point on the given AABB closest to the given ray. Also sets the out param
    /// `cosAng` to the cosine of the angle between the given ray and the given AABB.
    private static Vector3 GetClosestPoint(
        Ray ray,
        Bounds bounds,
        out float cosAng
    )
    {
      // If the ray directly intersects with the AABB, the angle is 0 and the closest point to
      // the ray is the intersection point.
      if (bounds.IntersectRay(ray, out float intersectDistance))
      {
        cosAng = 1.0f;
        return ray.origin + intersectDistance * ray.direction;
      }

      /// Otherwise, the closest point to the ray will be at least tied with one of the
      /// corners so just check each one.
      var cosMinAng = -1.0f;
      Vector3? closestPoint = null;
      var points = new Vector3[] {
                new Vector3(bounds.min.x, bounds.min.y, bounds.min.z),
                new Vector3(bounds.max.x, bounds.min.y, bounds.min.z),
                new Vector3(bounds.min.x, bounds.max.y, bounds.min.z),
                new Vector3(bounds.min.x, bounds.min.y, bounds.max.z),
                new Vector3(bounds.max.x, bounds.max.y, bounds.min.z),
                new Vector3(bounds.min.x, bounds.min.y, bounds.max.z),
                new Vector3(bounds.min.x, bounds.max.y, bounds.max.z),
                new Vector3(bounds.max.x, bounds.max.y, bounds.max.z),
            };
      foreach (var point in points)
      {
        var currentCosAng = PointedAt.CosAngRayPoint(ray, point);
        if (currentCosAng > cosMinAng)
        {
          cosMinAng = currentCosAng;
          closestPoint = point;
        }
      }
      cosAng = cosMinAng;
      return closestPoint.Value;
    }

    /// Returns the cosine of the angle between the given ray and the given point.
    private static float CosAngRayPoint(Ray ray, Vector3 point)
    {
      return Vector3.Dot(ray.direction, (point - ray.origin).normalized);
    }
  }
}
