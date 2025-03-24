using Data;
using Prime31.StateKit;
using UnityEngine;
using UnityEngine.Android;

namespace States
{
    public class RequestMicrophonePermissionState : SKState<AppData>
    {
        private const string PermissionsDeniedMessage =
            "Microphone permissions must be granted.\nPlease open Settings > Permissions > Microphone\nand enable settings for the Meta Fair Siro application.";

        private PermissionCallbacks _callbacks;
        private bool _isMicrophonePermissionAvailable;

        public override void begin()
        {
            base.begin();
            if (Application.isEditor)
            {
                _machine.changeState<ConnectToSiroServiceState>();
                return;
            }
            Debug.Log($"RequestMicrophonePermissionState::{Permission.HasUserAuthorizedPermission(Permission.Microphone)}");
            if (Permission.HasUserAuthorizedPermission(Permission.Microphone))
            {
                _isMicrophonePermissionAvailable = true;
            }
            else
            {
                _callbacks = new PermissionCallbacks();
                AddCallbackListeners();
                Permission.RequestUserPermission(Permission.Microphone, _callbacks);
            }
        }

        public override void end()
        {
            _isMicrophonePermissionAvailable = false;
            RemoveCallbackListeners();
        }

        private void AddCallbackListeners()
        {
            _callbacks.PermissionDenied += PermissionCallbacks_PermissionDenied;
            _callbacks.PermissionGranted += PermissionCallbacks_PermissionGranted;
            _callbacks.PermissionDeniedAndDontAskAgain += PermissionCallbacks_PermissionDeniedAndDontAskAgain;
        }

        private void RemoveCallbackListeners()
        {
            if (_callbacks == null) return;
            _callbacks.PermissionDenied -= PermissionCallbacks_PermissionDenied;
            _callbacks.PermissionGranted -= PermissionCallbacks_PermissionGranted;
            _callbacks.PermissionDeniedAndDontAskAgain -= PermissionCallbacks_PermissionDeniedAndDontAskAgain;
            _callbacks = null;
        }

        private void PermissionCallbacks_PermissionDeniedAndDontAskAgain(string permissionName)
        {
            Debug.Log($"RequestMicrophonePermissionState::PermissionCallbacks_PermissionDeniedAndDontAskAgain");
            _context.FatalError = PermissionsDeniedMessage;
            _machine.changeState<FatalErrorState>();
        }

        private void PermissionCallbacks_PermissionGranted(string permissionName)
        {
            Debug.Log($"RequestMicrophonePermissionState::PermissionCallbacks_PermissionGranted");
            _isMicrophonePermissionAvailable = true;
        }

        private void PermissionCallbacks_PermissionDenied(string permissionName)
        {
            Debug.Log($"RequestMicrophonePermissionState::PermissionCallbacks_PermissionDenied");
            _context.FatalError = PermissionsDeniedMessage;
            _machine.changeState<FatalErrorState>();
        }

        public override void update(float deltaTime)
        {
            // hack for state machine issue — attempt to go to next state on begin
            // fails as rest of states aren't yet added
            if (!_isMicrophonePermissionAvailable) return;
            _isMicrophonePermissionAvailable = false;
            _machine.changeState<ConnectToSiroServiceState>();
        }
    }
}