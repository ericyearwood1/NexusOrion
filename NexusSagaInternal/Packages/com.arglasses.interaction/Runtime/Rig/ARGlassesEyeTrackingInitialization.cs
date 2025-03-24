using System;
using System.Collections;
using UnityEngine;
using UnityEngine.Android;

namespace ARGlasses.Interaction
{
    public class ARGlassesEyeTrackingInitialization : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesRig _rig;
        [SerializeField, ReadOnly] private bool _isEyeTrackingEnabled;
        [SerializeField, ReadOnly] private bool _isPermissionGranted;
        public event Action<string> WhenErrorMessageChanged = delegate { };

        [SerializeField, ReadOnly] private string _errorMessage;

        public string ErrorMessage
        {
            get => _errorMessage;
            private set
            {
                if (_errorMessage == value) return;
                _errorMessage = value;
                WhenErrorMessageChanged(_errorMessage);
            }
        }

        public static bool IsEyeTrackingEnabled()
        {
            return OVRPlugin.eyeTrackingEnabled;
        }

        const string EyeTrackingPermission = "com.oculus.permission.EYE_TRACKING";

        public static bool IsPermissionGranted()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            return UnityEngine.Android.Permission.HasUserAuthorizedPermission(EyeTrackingPermission);
#else
            return true;
#endif
        }

        private static void RequestPermissions()
        {
            PermissionCallbacks callbacks = new PermissionCallbacks();
            Permission.RequestUserPermissions(new[] { EyeTrackingPermission }, callbacks);
        }

        private void Awake()
        {
            this.Scene(ref _rig);
            _rig.WhenIsUserInHmdChanged += HandleUserHmdChanged;
        }

        private void HandleUserHmdChanged(bool isHmd)
        {
            if (!isHmd) return;
            StartCoroutine(InitializeEyeTracking());
        }

        private IEnumerator InitializeEyeTracking()
        {
            _isPermissionGranted = IsPermissionGranted();
            while (!_isPermissionGranted)
            {
                ErrorMessage = "Missing Eye Tracking permission.\nTry restarting the app.";
                RequestPermissions();
                yield return new WaitForSeconds(1);
                _isPermissionGranted = IsPermissionGranted();
            }

            _isEyeTrackingEnabled = IsEyeTrackingEnabled();
            while (!_isEyeTrackingEnabled)
            {
                ErrorMessage = "Eye Tracking not enabled.\nTry restarting the app.";
                OVRPlugin.StartEyeTracking();
                yield return new WaitForSeconds(1);
                _isEyeTrackingEnabled = IsEyeTrackingEnabled();
            }

            ErrorMessage = string.Empty;
        }
    }
}
