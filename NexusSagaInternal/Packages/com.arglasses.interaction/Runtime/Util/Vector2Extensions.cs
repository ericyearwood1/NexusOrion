using UnityEngine;

// todo Lifted from ITK.Util during GazeIX -> ISDK transition, need to find the right home
namespace ARGlasses.Interaction
{
    public static class Vector2Extensions
    {
        /// <summary>
        /// returns 1/v
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector2 Inverse(this Vector2 v)
        {
            return new Vector2(1.0f / v.x, 1.0f / v.y);
        }

        /// <summary>
        /// Component wise division of two Vectors
        /// </summary>
        /// <param name="numerator"></param>
        /// <param name="denominator"></param>
        /// <returns></returns>
        public static Vector2 Divide(this Vector2 numerator, Vector2 denominator)
        {
            return new Vector2(numerator.x / denominator.x, numerator.y / denominator.y);
        }

        public static Vector2 WithX(this Vector2 v, float x)
        {
            v.x = x;
            return v;
        }

        public static Vector2 WithY(this Vector2 v, float y)
        {
            v.y = y;
            return v;
        }
        public static Vector3 WithZ(this Vector2 v, float z)
        {
            return new Vector3(v.x, v.y, z);
        }

        public static string ToDebugString(this Vector2 v)
        {
            return $"x: {v.x:F3} y: {v.y:F3}";
        }
    }
}
