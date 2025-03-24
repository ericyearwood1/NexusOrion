using UnityEngine;

// todo Lifted from ITK.Util during GazeIX -> ISDK transition, need to find the right home
namespace ARGlasses.Interaction
{
    public static class Vector3Extensions
    {
        /// <summary>
        /// returns 1/v
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public static Vector3 Inverse(this Vector3 v)
        {
            return new Vector3(1.0f / v.x, 1.0f / v.y, 1.0f / v.z);
        }

        /// <summary>
        /// Component wise division of two Vectors
        /// </summary>
        /// <param name="numerator"></param>
        /// <param name="denominator"></param>
        /// <returns></returns>
        public static Vector3 Divide(this Vector3 numerator, Vector3 denominator)
        {
            return new Vector3(numerator.x / denominator.x, numerator.y / denominator.y, numerator.z / denominator.z);
        }

        public static Vector3 FlipX(this Vector3 v)
        {
            v.x = -v.x;
            return v;
        }

        public static Vector3 FlipY(this Vector3 v)
        {
            v.y = -v.y;
            return v;
        }

        public static Vector3 FlipZ(this Vector3 v)
        {
            v.z = -v.z;
            return v;
        }

        public static Vector3 WithX(this Vector3 v, float x)
        {
            v.x = x;
            return v;
        }

        public static Vector3 WithY(this Vector3 v, float y)
        {
            v.y = y;
            return v;
        }

        public static Vector3 WithZ(this Vector3 v, float z)
        {
            v.z = z;
            return v;
        }

        public static Vector2 XY(this Vector3 v)
        {
            return new Vector2(v.x, v.y);
        }
        public static Vector2 ZY(this Vector3 v)
        {
            return new Vector2(v.z, v.y);
        }
        public static Vector2 XZ(this Vector3 v)
        {
            return new Vector2(v.x, v.z);
        }

        public static Vector3 XYto3(this Vector2 v)
        {
            return new Vector3(v.x, v.y, 0);
        }

        public static Vector3 ZYto3(this Vector2 v)
        {
            return new Vector3(0, v.y, v.x);
        }

        public static Vector3 XZto3(this Vector2 v)
        {
            return new Vector3(v.x, 0, v.y);
        }

        public static Vector3 NormalToUp(this Vector3 n)
        {
            var upDot = Mathf.Abs(Vector3.Dot(n, Vector3.up));
            var rightDot = Mathf.Abs(Vector3.Dot(n, Vector3.right));
            var fwdDot = Mathf.Abs(Vector3.Dot(n, Vector3.forward));

            var best = Vector3.up;
            var bestDot = upDot;
            if (rightDot < bestDot)
            {
                best = Vector3.right;
                bestDot = rightDot;
            }

            if (fwdDot < bestDot)
            {
                best = Vector3.forward;
            }

            return best;
        }

        public static Vector3 RotateAround(this Vector3 point, Vector3 pivot, Quaternion rotation)
        {
            var diff = point - pivot;
            diff = rotation * diff;
            return pivot + diff;
        }

        public static Vector3 Min(this Vector3 a, Vector3 b) => Vector3.Min(a, b);
        public static Vector3 Max(this Vector3 a, Vector3 b) => Vector3.Max(a, b);

        public static float MaxComponent(this Vector3 v)
        {
            var max = v.x;
            if (v.y > max) max = v.y;
            if (v.z > max) max = v.z;
            return max;
        }


        public static float MinComponent(this Vector3 v)
        {
            var min = v.x;
            if (v.y < min) min = v.y;
            if (v.z < min) min = v.z;
            return min;
        }

        public static string ToDebugString(this Vector3 v)
        {
            return $"x: {v.x:F3} y: {v.y:F3}, z: {v.z:F3}";
        }
    }
}
