using UnityEngine;

namespace CTRL.Math
{
  public static class MathUtil
  {
    public static float Map(float x, float inMin, float inMax, float outMin, float outMax, bool clamp = false)
    {
      if (clamp)
      {
        x = System.Math.Max(inMin, System.Math.Min(x, inMax));
      }

      return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double Map(double x, double inMin, double inMax, double outMin, double outMax, bool clamp = false)
    {
      if (clamp)
      {
        x = System.Math.Max(inMin, System.Math.Min(x, inMax));
      }

      return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static float DeltaAngleRadian(float current, float target)
    {
      const float PI_2 = 2.0f * Mathf.PI;

      float num = Mathf.Repeat(target - current, PI_2);
      if (num > Mathf.PI)
      {
        num -= PI_2;
      }

      return num;
    }
  }
}
