using Robot.Runtime.View;
using UnityEngine;

namespace Multiplayer.Runtime.View
{
    public class UserNameDisplay : LabelHighlightView
    {
        private Vector3 _velocity;
        [SerializeField] private float _trackSpeed = 40;
        
        private UserEntityView _trackedEntity;

        private void Awake()
        {
            var camera = Camera.main;
            if (camera == null)
            {
                Debug.LogError("Could not find main camera");
                return;
            }
            _cameraTransform = camera.transform;
        }
        
        
        protected override void Update()
        {
            if (_trackedEntity != null)
            {
                var targetPosition = _trackedEntity.transform.localPosition;
                targetPosition.y += 0.3f;
                transform.localPosition = Vector3.SmoothDamp(transform.localPosition, targetPosition, ref _velocity, _trackSpeed * Time.deltaTime);
            }
            base.Update();
        }

        public void UpdateDisplay(string userName, Color colour, Transform cameraTransform)
        {
            _color = colour;
            Debug.Log($"UpdateDisplay : {userName} | {colour}");
            _cameraTransform = cameraTransform;
            _label.text = userName;
            _display.color = _color;
        }

        public void Show()
        {
            StartShow(_cameraTransform);
        }

        public void TrackEntity(UserEntityView view)
        {
            _trackedEntity = view;
        }
    }
}