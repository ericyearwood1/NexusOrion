using UnityEngine;
//using Oculus.VR;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.FromOVREyeTracking)]
    public class ARGlassesEyeTrackingRig : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesRig _fullRig;
        public ARGlassesRig FullRig => _fullRig;

        [SerializeField, ReadOnly] private Pose _headPose;
        public Pose HeadPose => _headPose;

        [SerializeField, ReadOnly] private bool _eyesTracked;
        public bool EyesTracked => _eyesTracked;

        [SerializeField, ReadOnly] private Pose _eyesPose;
        public Pose EyesPose => _eyesPose;

        [SerializeField, ReadOnly] private Snapshot.Gaze _snapshot;

        [SerializeField, ReadOnly] private OVRCameraRig _ovrCameraRig;
        [SerializeField, ReadOnly] private bool _useOvrEyeGazeInteraction;

        private Camera _centerEyeCamera;

        public delegate void GetEyeTrackingPoseFn(out Pose pose, out bool eyesTracked);
        private GetEyeTrackingPoseFn GetEyeTrackingPose;

        public void OverrideEyePoseAcquisitionFunction(GetEyeTrackingPoseFn newEyeTrackingPoseFunction)
        {
            GetEyeTrackingPose = newEyeTrackingPoseFunction;
        }

        private void Awake()
        {
            this.Scene(ref _ovrCameraRig);
            this.Scene(ref _fullRig);

            if (!_centerEyeCamera) _centerEyeCamera = Camera.main;
            if (_useOvrEyeGazeInteraction) 
                OverrideEyePoseAcquisitionFunction(UpdateFromInteractionEyes);
            else 
                OverrideEyePoseAcquisitionFunction(UpdateFromSocialEyes);
        }

        // Guaranteed to be called after OVRCameraRig (see DefaultExecutionOrder with ExecutionOrder.FromOVRGaze)
        protected void Update()
        {
        }

        public Snapshot.Gaze Snapshot
        {
            get
            {
                _headPose = GetHead();

                if (_fullRig.IsUserInHmd)
                {
                   // GetEyeTrackingPose(out _eyesPose, out _eyesTracked);
                }
                else
                {
                    UpdateFromEmulator(out _eyesPose, out _eyesTracked);
                }

                _snapshot = new Snapshot.Gaze(_eyesTracked, _eyesPose, _headPose);
                transform.Set(_snapshot.Eyes);
                return _snapshot;
            }
        }

        public static readonly Pose CenterEyeToHeadCenter = new(new Vector3(0, -0.067f, -0.08f), Quaternion.identity);
        [SerializeField, ReadOnly] private OVRPlugin.EyeGazeInteractionsState _eyeGazeInteractionsState;
        [SerializeField, ReadOnly] private OVRPlugin.EyeGazesState _eyeGazeSocialState;
        [SerializeField, ReadOnly] private bool _eyeApiReturnValue;
        [SerializeField, ReadOnly] private Pose _leftEyePose;
        [SerializeField, ReadOnly] private Pose _rightEyePose;

        private Pose GetHead()
        {
            Transform root = _ovrCameraRig.trackingSpace;
            var centerEye = _ovrCameraRig.centerEyeAnchor.ToPose();
            return centerEye.Transform(CenterEyeToHeadCenter);
        }

        // Interaction Filtering
        // https://fb.workplace.com/groups/297609864237104/permalink/1261265567871524/
        private void UpdateFromInteractionEyes(out Pose pose, out bool eyesTracked)
        {
            eyesTracked = false;
            pose = _centerEyeCamera.transform.ToPose();

            _eyeApiReturnValue = OVRPlugin.GetEyeGazeInteractionsState(OVRPlugin.Step.Render, -1, ref _eyeGazeInteractionsState);
            var eyeId = OVREyeGazeInteraction.EyeId.Combined;
            var eyeGaze = _eyeGazeInteractionsState.EyeGazeInteractions[(int)eyeId];

            var isValid = _eyeApiReturnValue && eyeGaze.IsValid;
            if (!isValid) return;

            pose = eyeGaze.ToWorldSpacePose();
            eyesTracked = true;
        }

        private void UpdateFromEmulator(out Pose pose, out bool eyesTracked)
        {
            eyesTracked = true;
            pose = _fullRig.MouseKeyboardEmulator.EmulatedEyeTracking;
        }

        private void UpdateFromSocialEyes(out Pose eyesPose, out bool eyesTracked)
        {
            eyesPose = _centerEyeCamera.transform.ToPose();
            eyesTracked = false;
            _eyeApiReturnValue = OVRPlugin.GetEyeGazesState(OVRPlugin.Step.Render, -1, ref _eyeGazeSocialState);
            var leftState = _eyeGazeSocialState.EyeGazes[0];
            var  rightState = _eyeGazeSocialState.EyeGazes[1];

            _leftEyePose = leftState.IsValid && leftState.Confidence != 0 ? leftState.ToWorldSpacePose() : _leftEyePose;
            _rightEyePose = rightState.IsValid && rightState.Confidence != 0 ? rightState.ToWorldSpacePose() : _rightEyePose;

            var isValid = _eyeApiReturnValue && leftState.IsValid && rightState.IsValid;
            // var isLink = _fullRig.IsUserInLink; // Link always seem to return false for IsValid? :(
            if (!isValid) return;

            var centerPosition = Vector3.Lerp(_leftEyePose.position, _rightEyePose.position, 0.5f);
            var centerRotation = Quaternion.Slerp(_leftEyePose.rotation, _rightEyePose.rotation, 0.5f);
            eyesPose = new Pose(centerPosition, centerRotation);
            eyesTracked = true;
            //Debug.Log("Eye rotation is: " + centerRotation);
        }
    }
}
