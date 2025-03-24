using UnityEngine;
using UnityEngine.UI;

namespace ARGlasses.Components
{
    public class RadialProgressBar : MonoBehaviour
    {
        [SerializeField] private Image fillImage;

        [SerializeField] private RectTransform endCapPivot;

        [SerializeField] private RectTransform fillPivot;

        [SerializeField] private CanvasGroup _canvasGroup;

        [SerializeField, Range(0f, 1f)] private float _fillPercentage;

        public float RotationDegPerSecond;

        public float CanvasGroupAlpha
        {
            get => _canvasGroup.alpha;
            set => _canvasGroup.alpha = value;
        }

        public float FillPercentage
        {
            get => _fillPercentage;
            set
            {
                _fillPercentage = value;
                // Clamp fill value between 0 and 1
                _fillPercentage = Mathf.Clamp01(_fillPercentage);

                // Set fill amount for the fill image
                fillImage.fillAmount = _fillPercentage;

                // Calculate the rotation angle
                float rotationAngle = 360f * _fillPercentage;

                // Set the end cap rotation
                endCapPivot.localRotation = Quaternion.Euler(0f, 0f, -rotationAngle);
            }
        }

        [SerializeField, Range(0f, 1f)] private float _startPercentage;


        private void Update()
        {
            // Rotate the fillPivot
            float rotationAmount = RotationDegPerSecond * Time.deltaTime;
            fillPivot.Rotate(0f, 0f, -rotationAmount);
        }

        public float StartPercentage
        {
            get => _startPercentage;
            set
            {
                _startPercentage = value;
                // Clamp fill value between 0 and 1
                _startPercentage = Mathf.Clamp01(_startPercentage);

                // Calculate the rotation angle
                float rotationAngle = 360f * _startPercentage;

                // Set the end cap rotation
                fillPivot.localRotation = Quaternion.Euler(0f, 0f, -rotationAngle);
            }
        }
    }
}
