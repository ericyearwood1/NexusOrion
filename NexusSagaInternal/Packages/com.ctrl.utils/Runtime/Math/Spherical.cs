using System;
using UnityEngine;

namespace CTRL.Math
{
  [Serializable]
  public struct Spherical
  {
    /// Distance from origin
    public float Radius;

    /// -PI/2 (north pole) to PI/2 (south pole)
    public float Inclination;

    /// -PI (west) to PI (east)
    public float Azimuth;

    public Spherical(float radius = 1, float azimuth = 0, float inclination = 0)
    {
      this.Radius = radius;
      this.Azimuth = azimuth;
      this.Inclination = inclination;
    }

    public Spherical SetFromCartesian(Vector3 v)
    {
      Radius = v.magnitude;
      Azimuth = Mathf.Atan2(v.z, v.x);
      Inclination = Mathf.Acos(v.y / Radius) - Mathf.PI / 2;
      return this;
    }

    public static Spherical FromCartesian(Vector3 v)
    {
      return new Spherical().SetFromCartesian(v);
    }

    public Vector3 ToCartesian()
    {
      float a = Radius * Mathf.Sin(Inclination);
      return new Vector3(
        a * Mathf.Cos(Azimuth),
        Radius * Mathf.Cos(Inclination),
        a * Mathf.Sin(Azimuth)
      );
    }

    public override string ToString()
    {
      return ToString("F2");
    }

    public string ToString(string format)
    {
      return $"s[{Radius.ToString(format)},{Inclination.ToString(format)},{Azimuth.ToString(format)}]";
    }
  }
}
