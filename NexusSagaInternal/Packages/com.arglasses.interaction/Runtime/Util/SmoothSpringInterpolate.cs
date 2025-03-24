using UnityEngine;

namespace ARGlasses.R7
{
    public static class SmoothSpringInterpolate
    {
        public static bool Interpolate(ref float x, float target, ref float velocity, float smooth, float epsilon)
        {
            if (x == target) return false;

            float result = Mathf.SmoothDamp(x, target, ref velocity, smooth);
            if (float.IsNaN(result) || float.IsInfinity(result))
            {
                x = target;
                velocity = 0;
                return true;
            }

            if (Mathf.Abs(target - result) < epsilon)
            {
                x = target;
                velocity = 0;
                return true;
            }

            x = result;
            return true;
        }

        public static bool InterpolateAngle(ref float x, float target, ref float velocity, float smooth, float epsilon)
        {
            if (x == target) return false;

            float result = Mathf.SmoothDampAngle(x, target, ref velocity, smooth);
            if (float.IsNaN(result) || float.IsInfinity(result))
            {
                x = target;
                velocity = 0;
                return true;
            }

            if (Mathf.Abs(target - result) < epsilon)
            {
                x = target;
                velocity = 0;
                return true;
            }

            x = result;
            return true;
        }

        public static bool Interpolate(ref Vector3 v, Vector3 target, ref Vector3 velocity, float smooth, float epsilon)
        {
            if (v == target) return false;

            Vector3 result;
            result.x = Mathf.SmoothDamp(v.x, target.x, ref velocity.x, smooth);
            result.y = Mathf.SmoothDamp(v.y, target.y, ref velocity.y, smooth);
            result.z = Mathf.SmoothDamp(v.z, target.z, ref velocity.z, smooth);
            if (float.IsNaN(result.x) || float.IsInfinity(result.x) ||
                float.IsNaN(result.y) || float.IsInfinity(result.y) ||
                float.IsNaN(result.z) || float.IsInfinity(result.z))
            {
                v = target;
                velocity = Vector3.zero;
                return true;
            }

            if ((target - result).sqrMagnitude < epsilon)
            {
                v = target;
                velocity = Vector3.zero;
                return true;
            }

            v = result;
            return true;
        }
    }
}
