using UnityEngine;
using System;

namespace CTRL.Math.Filters
{
  [Serializable]
  public class DirectionToPlaneFilterEuler: IDirectionToPlaneFilter
  {
    // The current "zero" center of the screen
    protected Vector3? _referenceDirection;
    public Vector3? ReferenceDirection { get => _referenceDirection; }
    public Quaternion _referenceOrientation = Quaternion.identity;
    
    public void Reset()
    {
      _referenceDirection = null;
      _referenceOrientation = Quaternion.identity;
    }

    public void Reset(Vector3 newReference)
    {
      _referenceDirection = newReference;
      _referenceOrientation = Quaternion.LookRotation(newReference);
    }

    /// <summary>
    /// Computes the difference between the reference and current orientation as the difference in Y and X Euler Angles between the two. The orientations are defined using reference and current direction as the forward vector, respectively, and +Y as the up vector.
    /// *NOTE.* This class is purely for demonstration purposes. Usage in actual applications is not recommended.
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
      Quaternion curOrientation = Quaternion.LookRotation(curDirection);

      // Euler approach
      var curEuler = curOrientation.eulerAngles;
      var refEuler = _referenceOrientation.eulerAngles;

      float xPlane = Mathf.Deg2Rad * Mathf.DeltaAngle(curEuler.y, refEuler.y);
      float yPlane = Mathf.Deg2Rad * Mathf.DeltaAngle(curEuler.x, refEuler.x);

      //Debug.Log($"{curDirection:F6} {curInRef:F6} {inclination} {azimuth} {xPlane} {yPlane}");

      return new Vector2(
        xPlane,
        yPlane);
    }
  }
}
