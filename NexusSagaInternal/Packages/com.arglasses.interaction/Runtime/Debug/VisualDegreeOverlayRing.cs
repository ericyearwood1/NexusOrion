using UnityEngine;

namespace ARGlasses.Interaction
{
    public class VisualDegreeOverlayRing : MonoBehaviour
    {
        [SerializeField] private bool _showRing;
        [SerializeField] private float _radiusDegrees = 2f;
        [SerializeField] private Color _fillColor = Color.red;
        [SerializeField] private float _borderWidth = 0.025f;
        [SerializeField] private Color _borderColor = Color.red;

        public bool ShowRing
        {
            get => _showRing;
            set => _showRing = value;
        }

        public float RadiusDegrees
        {
            get => _radiusDegrees;
            set => _radiusDegrees = value;
        }

        public Color FillColor => _fillColor;
        public float BorderWidth => _borderWidth;
        public Color BorderColor => _borderColor;

        public void Init(Color color, float radiusDegrees, float borderWidth)
        {
            _showRing = true;
            _borderWidth = borderWidth;
            _radiusDegrees = radiusDegrees;
            _fillColor = color;
            _borderColor = color;
            transform.position = Vector3.forward;
        }
    }
}
