using UnityEngine;

namespace ARGlasses.Interaction
{
    public class FixationRectangle : MonoBehaviour
    {
        [SerializeField, ReadOnly] private Vector2 _localSize;
        public Vector2 LocalSize => _localSize;
        [SerializeField, ReadOnly] private float _xMin, _xMax, _yMin, _yMax;
        [SerializeField, ReadOnly] protected RectTransform _localSizeProvider;

        protected void OnEnable()
        {
            this.Sibling(ref _localSizeProvider);

            Vector3[] corners = new Vector3[4];
            _localSizeProvider.GetWorldCorners(corners);
            Vector2 worldSize = new(Mathf.Abs(corners[2].x - corners[0].x), Mathf.Abs(corners[2].y - corners[0].y));

            var localWidth = transform.InverseTransformVector(Vector3.right * worldSize.x).magnitude;
            var localHeight = transform.InverseTransformVector(Vector3.up * worldSize.y).magnitude;

            _localSize = new Vector2(localWidth, localHeight);
            _xMin = transform.localPosition.x - 0.5f * localWidth;
            _xMax = transform.localPosition.x + 0.5f * localWidth;
            _yMin = transform.localPosition.y - 0.5f * localHeight;
            _yMax = transform.localPosition.y + 0.5f * localHeight;
        }

        public bool ContainsXY(Vector3 point)
        {
            point = transform.InverseTransformPoint(point);
            return point.x >= _xMin && point.x <= _xMax && point.y >= _yMin && point.y <= _yMax;
        }

        private Vector3 LocalOffset(Vector3 point, bool clampZ)
        {
            var local = transform.InverseTransformPoint(point);
            var offset = new Vector3()
            {
                x = AxisDistance(local.x, _xMin, _xMax),
                y = AxisDistance(local.y, _yMin, _yMax),
                z = clampZ ? 0.0f : local.z
            };
            return offset;
        }

        public float Distance(Vector3 point, bool clampZ = true) => LocalOffset(point, clampZ).magnitude;

        private static float AxisDistance(float val, float min, float max)
        {
            if (val < min) return val - min;
            if (val > max) return val - max;
            return 0.0f;
        }
    }
}
