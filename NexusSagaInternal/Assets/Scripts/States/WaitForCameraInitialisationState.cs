using Data;
using Prime31.StateKit;
using UnityEngine;

namespace States
{
    public class WaitForCameraInitialisationState : SKState<AppData>
    {
        private float _targetWaitTime = 1.4f;
        private float _currentWaitTime;
        public override void update(float deltaTime)
        {
            if (Application.isEditor)
            {
                _machine.changeState<RequestMicrophonePermissionState>();
                return;
            }

            _currentWaitTime += deltaTime;
            if (_currentWaitTime < _targetWaitTime) return;
            if(_context.CameraTransform.position.y < 0.3f) return;
            _context.FullFocusCanvasUI.ForceUpdatePose();
            _machine.changeState<RequestMicrophonePermissionState>();
        }
    }
}