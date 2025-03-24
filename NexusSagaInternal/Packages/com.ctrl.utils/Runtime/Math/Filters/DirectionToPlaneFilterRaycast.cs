using UnityEngine;
using System;

namespace CTRL.Math.Filters
{
  [Serializable]
  public class DirectionToPlaneFilterRaycast : IDirectionToPlaneFilter
  {
    // The current "zero" center of the screen
    protected Vector3? _referenceDirection;
    public Vector3? ReferenceDirection { get => _referenceDirection; }
    private Quaternion _inverseReferenceOrientation;

    public void Reset()
    {
      _referenceDirection = null;
      _inverseReferenceOrientation = Quaternion.identity;
    }

    public void Reset(Vector3 newReference)
    {
      _referenceDirection = newReference;

      // Build reference frame
      // Z-axis is just the reference direction
      // X-axis
      Vector3 refRight = Vector3.Cross(Vector3.up, newReference);
      if (refRight.sqrMagnitude < Vector3.kEpsilon)
      {
        refRight = Vector3.right;
      }
      // Y-axis
      Vector3 refUp = Vector3.Cross(newReference, refRight);
      Quaternion refOrientation = Quaternion.LookRotation(newReference, refUp);
      _inverseReferenceOrientation = Quaternion.Inverse(refOrientation);
    }

    /// <summary>
    /// This computes the coordinates of curDirection w.r.t. the reference orientation by raycasting from the origin of a sphere to a tangent plane at (0, 0, 1). This is called a gnomonic projection: https://en.wikipedia.org/wiki/Gnomonic_projection. This has one nice property:
    /// a) the directionality of the forearm motion is reflected accurately in the pointer's motion on the plane.
    /// </summary>
    /// <param name="curDirection">The current direction.</param>
    /// <returns>
    /// Planar projection of the current direction w.r.t. the reference direction being the origin (0, 0).
    /// </returns>
    public Vector2 Evaluate(Vector3 curDirection)
    {
      // Ensure reference direction is set
      if (!_referenceDirection.HasValue)
      {
        Reset(curDirection);
        return new Vector2();
      }

      // Rotate current vector to identity frame
      Vector3 curInRef = _inverseReferenceOrientation * curDirection;
      curInRef.Normalize();

      // Get Spherical coordinates w.r.t. +Z being the zenith and +X the azimuthal zero
      float inclination = Mathf.Acos(Mathf.Clamp(curInRef.z, -1, 1));
      float azimuth = Mathf.Atan2(curInRef.y, curInRef.x);

      float magnitude = Mathf.Tan(Mathf.Min(inclination, Mathf.PI/2));

      float xPlane = magnitude * Mathf.Cos(azimuth);
      float yPlane = magnitude * Mathf.Sin(azimuth);

      //Debug.Log($"{curDirection:F6} {curInRef:F6} {inclination} {azimuth} {xPlane} {yPlane}");

      return new Vector2(
        xPlane,
        yPlane);
    }
  }
}
