using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class TrackingCone : MonoBehaviour
    {
        [SerializeField] Color _debugColor = Color.yellow;
        [SerializeField] private bool _drawDebug;
        public bool DrawDebug
        {
            get => _drawDebug;
            set => _drawDebug = value;
        }

        public Color DebugColor
        {
            get => _debugColor;
            set => _debugColor = value;
        }

        private Cone _cone = Cone.Default;

        public void SetAngles(float x, float y) => _cone = new Cone(x, y);
        public bool Contains(Vector3 point) => _cone.Contains(point);

        void Update()
        {
            _cone.forward = transform.forward;
            _cone.up = transform.up;
            _cone.right = transform.right;
            _cone.origin = transform.position;
            if(DrawDebug) _cone.DrawDebug(2f, DebugColor.WithAlpha(0.1f));
        }

        [Serializable]
        public struct Cone
        {
            public Vector3 origin;
            public Vector3 forward;
            public Vector3 up;
            public Vector3 right;

            private float xAngle;
            private float yAngle;

            private float cosYAngle;
            private float cosXAngle;

            public Cone(float xAngle, float yAngle)
            {
                this.xAngle = 0.5f * xAngle;
                this.yAngle = 0.5f * yAngle;
                cosXAngle = Mathf.Cos(this.xAngle * Mathf.Deg2Rad);
                cosYAngle = Mathf.Cos(this.yAngle * Mathf.Deg2Rad);
                origin = Vector3.zero;
                forward = Vector3.forward;
                up = Vector3.up;
                right = Vector3.right;
            }


            public static Cone Default => new Cone(60, 60);

            public void DrawDebug(float length, Color c)
            {
                // Debug.DrawLine(origin, origin + length * forward, Color.green);

                float upLen = Mathf.Tan(Mathf.Deg2Rad * yAngle);
                Vector3 yVec = (forward + up * upLen).normalized;
                Vector3 yDownVec = (forward - up * upLen).normalized;
                float rightLen = Mathf.Tan(Mathf.Deg2Rad * xAngle);
                Vector3 xVec = (forward + right * rightLen).normalized;
                Vector3 xDownVec = (forward - right * rightLen).normalized;

                Vector3 xyVec0 = (forward + 0.707f * right * rightLen + 0.707f * up * upLen).normalized;
                Vector3 xyVec1 = (forward - 0.707f * right * rightLen + 0.707f * up * upLen).normalized;
                Vector3 xyVec2 = (forward - 0.707f * right * rightLen - 0.707f * up * upLen).normalized;
                Vector3 xyVec3 = (forward + 0.707f * right * rightLen - 0.707f * up * upLen).normalized;

                Vector3 xyVec4 = (forward + 0.5f * right * rightLen + 0.866f * up * upLen).normalized;
                Vector3 xyVec5 = (forward - 0.5f * right * rightLen + 0.866f * up * upLen).normalized;
                Vector3 xyVec6 = (forward - 0.5f * right * rightLen - 0.866f * up * upLen).normalized;
                Vector3 xyVec7 = (forward + 0.5f * right * rightLen - 0.866f * up * upLen).normalized;

                Vector3 xyVec8 = (forward + 0.866f * right * rightLen + 0.5f * up * upLen).normalized;
                Vector3 xyVec9 = (forward - 0.866f * right * rightLen + 0.5f * up * upLen).normalized;
                Vector3 xyVec10 = (forward - 0.866f * right * rightLen - 0.5f * up * upLen).normalized;
                Vector3 xyVec11 = (forward + 0.866f * right * rightLen - 0.5f * up * upLen).normalized;

                Debug.DrawLine(origin, origin + length * yVec, c);
                Debug.DrawLine(origin, origin + length * yDownVec, c);

                Debug.DrawLine(origin, origin + length * xVec, c);
                Debug.DrawLine(origin, origin + length * xDownVec, c);

                Debug.DrawLine(origin, origin + length * xyVec0, c);
                Debug.DrawLine(origin, origin + length * xyVec1, c);
                Debug.DrawLine(origin, origin + length * xyVec2, c);
                Debug.DrawLine(origin, origin + length * xyVec3, c);

                Debug.DrawLine(origin, origin + length * xyVec4, c);
                Debug.DrawLine(origin, origin + length * xyVec5, c);
                Debug.DrawLine(origin, origin + length * xyVec6, c);
                Debug.DrawLine(origin, origin + length * xyVec7, c);

                Debug.DrawLine(origin, origin + length * xyVec8, c);
                Debug.DrawLine(origin, origin + length * xyVec9, c);
                Debug.DrawLine(origin, origin + length * xyVec10, c);
                Debug.DrawLine(origin, origin + length * xyVec11, c);
            }

            public bool Contains(Vector3 p)
            {
                Vector3 ray = p - origin;
                Vector3 rayNorm = ray.normalized;

                float cosAngle = Vector3.Dot(rayNorm, forward);
                if (cosAngle < cosYAngle && cosAngle < cosXAngle)
                {
                    Debug.DrawLine(origin, p, Color.red);
                    return false;
                }

                if (cosAngle >= cosYAngle && cosAngle >= cosXAngle)
                {
                    Debug.DrawLine(origin, p, Color.green);
                    return true;
                }


                // Project the ray onto the circle
                Vector3 tmp = Vector3.Cross(forward, rayNorm).normalized;
                Vector3 proj = Vector3.Cross(tmp, forward);


                float cosAngleUp = Mathf.Abs(Vector3.Dot(up, proj));
                float cosAngleRight = Mathf.Abs(Vector3.Dot(right, proj));

                // Compute the threshold at an orientation (cos^2 + sin^2 = 1).  Two cos here because we're comparing up and right.
                float threshold = cosAngleUp * cosAngleUp * cosYAngle + cosAngleRight * cosAngleRight * cosXAngle;

                bool result = cosAngle >= threshold;
                Debug.DrawLine(origin, p, result ? Color.blue : Color.red);
                return result;
            }
        }
    }
}
