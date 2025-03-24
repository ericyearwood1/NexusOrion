using UnityEngine;
using System;
using CTRL.Math;

namespace CTRL.Math.Filters
{

  [Serializable]
  public class DirectionToPlaneFilterSpherical : IDirectionToPlaneFilter
  {
    // The current "zero" center of the screen
    protected Vector3? _referenceDirection;
    public Vector3? ReferenceDirection { get => _referenceDirection; }
    private Vector2 _referenceHorizontalProjection;
    private Quaternion _inverseReferenceOrientation;

    public void Reset()
    {
      _referenceDirection = null;
      _inverseReferenceOrientation = Quaternion.identity;
    }

    public void Reset(Vector3 newReference)
    {
      _referenceDirection = newReference;

      // Compute the horizontal projection of the reference direction
      Vector3 _referenceDirectionHorizontal = Vector3.ProjectOnPlane(_referenceDirection.Value, Vector3.up).normalized;

      // Compute the quaternion that rotates the reference direction to its horizontal projection
      _inverseReferenceOrientation = Quaternion.FromToRotation(_referenceDirection.Value, _referenceDirectionHorizontal);
      var sph = new Spherical().SetFromCartesian(_referenceDirectionHorizontal);
      _referenceHorizontalProjection = new Vector2(sph.Azimuth, sph.Inclination);
    }

    /// This computes the coordinates of curDirection w.r.t. the reference orientation using an azimuthal equidistant projection: https://en.wikipedia.org/wiki/Azimuthal_equidistant_projection.
    public Vector2 Evaluate(Vector3 curDirection)
    {
      // Ensure reference direction is set
      if (!_referenceDirection.HasValue)
      {
        Reset(curDirection);
        return new Vector2();
      }

      Vector3 curInRef = _inverseReferenceOrientation * curDirection;

      // Move to spherical coordinates for difference calculations
      var sphCurr = new Spherical().SetFromCartesian(curInRef);

      return new Vector2(
        MathUtil.DeltaAngleRadian(sphCurr.Azimuth, _referenceHorizontalProjection.x),
        MathUtil.DeltaAngleRadian(sphCurr.Inclination, _referenceHorizontalProjection.y));
    }
  }
}
