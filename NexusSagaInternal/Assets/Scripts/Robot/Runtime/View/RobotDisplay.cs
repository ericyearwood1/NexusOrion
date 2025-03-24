using UnityEngine;

namespace Robot.Runtime.View
{
    public class RobotDisplay : MonoBehaviour
    {
        [SerializeField] private MeshRenderer _debugRobotFrameMarker;
        [SerializeField] private MeshRenderer _debugEEMarker;
        [SerializeField] private MeshRenderer _debugGripPositionMarker;
        [SerializeField] private RobotOverlayView _robotUI;
        [SerializeField] private Transform _eePosition;
        [SerializeField] private Transform _eeOrientation;
        [SerializeField] private Transform _eeGripPosition;
        [SerializeField] private Transform _pitch;
        [SerializeField] private Transform _roll;
        [SerializeField] private Transform _yaw;
        public RobotOverlayView RobotUI => _robotUI;
        public Transform Pitch => _pitch;
        public Transform Roll => _roll;
        public Transform Yaw => _yaw;
        public Transform EEPositionTransform => _eePosition;
        public Transform EEOrientationTransform => _eeOrientation;
        public Transform EEGripPosition => _eeGripPosition;
        private bool _isShowDebugObjects;

        public void Initialise(Transform cameraTransform, bool isShowDebugObjects)
        {
            _isShowDebugObjects = isShowDebugObjects;
            _robotUI.Initialise(cameraTransform);
            ConfigureDebugObjects(false);
        }

        public void Show()
        {
            ConfigureDebugObjects(_isShowDebugObjects);
            _robotUI.Show();
        }

        private void ConfigureDebugObjects(bool isShowObjects)
        {
            _debugRobotFrameMarker.enabled = isShowObjects;
            _debugEEMarker.enabled = isShowObjects;
            _debugGripPositionMarker.enabled = isShowObjects;
        }
    }
}