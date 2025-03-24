using UnityEngine;

namespace CTRL.Math.Filters
{
  public interface IDirectionToPlaneFilter : IFilter<Vector3, Vector2>
  {
    public enum ProjectionType
    {
      AzimuthalEquidistant,
      Stereographic,
      SphericalCoordinates,
      Raycast,
      Euler
    }

    public Vector3? ReferenceDirection { get; }

    public void Reset(Vector3 newReferenceDirection);
  }
}
