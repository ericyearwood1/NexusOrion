using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.ARGlassesRig)]
    public class ARGlassesRig : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesEyeTrackingRig _eyeTrackingRig;
        public ARGlassesEyeTrackingRig EyeTrackingRig => _eyeTrackingRig;

        public Pose EyePose => _eyeTrackingRig.EyesPose;
        public Pose HeadPose => _eyeTrackingRig.HeadPose;

        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;
        public ARGlassesWristband Wristband => _wristband;

        [SerializeField, ReadOnly] private ARGlassesHandRig _leftHandRig;
        public ARGlassesHandRig LeftHandRig => _leftHandRig;
        [SerializeField, ReadOnly] private ARGlassesHandRig _rightHandRig;
        public ARGlassesHandRig RightHandRig => _rightHandRig;

        public IReadOnlyList<ARGlassesHandRig> HandRigs { get; private set; }

        [SerializeField, ReadOnly] private MouseKeyboardEmulator _emulator;
        public MouseKeyboardEmulator MouseKeyboardEmulator => _emulator;

        [SerializeField, ReadOnly] private Snapshot.Gaze _lastGaze;
        [SerializeField, ReadOnly] private Snapshot.Hand _lastLeft;
        public Snapshot.Hand LeftHandSnapshot => _lastLeft;
        [SerializeField, ReadOnly] private Snapshot.Hand _lastRight;
        public Snapshot.Hand RightHandSnapshot => _lastRight;
        public Snapshot.Hand GetHandSnapshot(Side side) => side.IsLeft() ? _lastLeft : _lastRight;

        [SerializeField, ReadOnly] private Snapshot.Wristband _lastWristband;
        [SerializeField, ReadOnly] private bool _isUserInHmd;
        public bool IsUserInHmd => _isUserInHmd;
        public bool IsUserInLink => _isUserInHmd && Application.isEditor;

        public static bool IsHmdPresentAndUserPresent => OVRNodeStateProperties.IsHmdPresent() && OVRManager.instance.isUserPresent;
        public event Action<bool> WhenIsUserInHmdChanged = delegate { };

        private void Awake()
        {
            this.Descendant(ref _eyeTrackingRig);
            this.Descendant(ref _wristband);
            this.Descendant(ref _emulator);
            this.Descendant(ref _leftHandRig, nameContains: "Left");
            this.Descendant(ref _rightHandRig, nameContains: "Right");

            HandRigs = new[] { _leftHandRig, _rightHandRig };
        }

        private ISnapshotRigProvider _rigProvider;
        public void SetSnapshotProvider(ISnapshotRigProvider rigProvider) => _rigProvider = rigProvider;

        [SerializeField, ReadOnly] private int _snapshotFrame;
        [SerializeField, ReadOnly] private Snapshot.Rig _snapshotCache;

        public Snapshot.Rig Snapshot()
        {
            if (_snapshotFrame == Time.frameCount) return _snapshotCache;
            _snapshotFrame = Time.frameCount;
            if (_rigProvider != null) _snapshotCache = _rigProvider.Snapshot;
            else _snapshotCache = GenerateSnapshot();
            return _snapshotCache;
        }

        private Snapshot.Rig GenerateSnapshot()
        {
            _lastGaze = _eyeTrackingRig.Snapshot;
            _lastWristband = _wristband.GetSnapshot();
            _lastLeft = _leftHandRig.GetSnapshot();
            _lastRight = _rightHandRig.GetSnapshot();
            return new Snapshot.Rig(_lastGaze, _lastLeft, _lastRight, _lastWristband, _isUserInHmd);
        }

        // todo move this
        private void Update()
        {
            var isHmd = IsHmdPresentAndUserPresent;
            if (_isUserInHmd == isHmd) return;
            _isUserInHmd = isHmd;
            WhenIsUserInHmdChanged(_isUserInHmd);
        }
    }
}
