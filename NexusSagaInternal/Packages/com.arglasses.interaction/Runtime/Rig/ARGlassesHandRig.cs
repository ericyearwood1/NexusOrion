using UnityEngine;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.FromOVRHandTracking)]
    public class ARGlassesHandRig : MonoBehaviour
    {
        [SerializeField] private OVRHand _ovrHand;
        [SerializeField] private Side _side;
        public Side Side => _side;
        private Snapshot.Hand _snapshot = default;

        [SerializeField, ReadOnly] private OVRSkeleton _skeleton;

        [SerializeField] private bool _allowController = true;
        private readonly Pose _controllerWristOffset = new(new Vector3(0, 0, -0.1f), Quaternion.identity);
        [SerializeField, ReadOnly] private HandTrackingLossSimulation _handTrackingLossSimulation;
        [SerializeField, ReadOnly] private MouseKeyboardEmulator _mouseKeyboardEmulator;

        [SerializeField] private OneEuroFilterPropertyBlock _pinchFilterProperties = new(0.2f, 10f, 1f);
        [SerializeField] private OneEuroFilterPropertyBlock _wristFilterProperties = new(0.1f, 100f, .6f);
        private IOneEuroFilter<Vector3> _pinchPosFilter = OneEuroFilter.CreateVector3();
        private IOneEuroFilter<Vector3> _wristPosFilter = OneEuroFilter.CreateVector3();

        private void Awake()
        {
            _side = this.SideFromName();
            this.Scene(ref _ovrHand, nameContains: _side.ToString());
            this.Scene(ref _mouseKeyboardEmulator);
            this.Scene(ref _handTrackingLossSimulation);

            _skeleton = _ovrHand.GetComponent<OVRSkeleton>();
            _bones = Snapshot.HandBones.Default(_side, Pose.identity);

            // todo reset when tracking lost/found?
            _pinchPosFilter.SetProperties(_pinchFilterProperties);
            _pinchPosFilter.Reset();
            _wristPosFilter.SetProperties(_wristFilterProperties);
            _wristPosFilter.Reset();
        }

        // Will always Update after OVRHands due to ExecutionOrder.FromOVRHand
        private void Update()
        {
        }

        public float Scale => _ovrHand.HandScale;

        private Snapshot.HandBones _bones;
        public Snapshot.HandBones Bones => _bones;

        public Snapshot.Hand GetSnapshot()
        {
            if (UpdateFromMouseKeyboard()) {}
            else if (UpdateFromHand()){}
            else UpdateFromController();


            return _snapshot;
        }

        private bool UpdateFromMouseKeyboard()
        {
            if (ARGlassesRig.IsHmdPresentAndUserPresent) return false;
            var emulatedHand = _mouseKeyboardEmulator.GetEmulation(Side);
            _snapshot = emulatedHand.Snapshot;

            transform.Set(_snapshot.Wrist);
            return true;
        }

        private int _lastHandUpdateFrame;
        private bool UpdateFromHand()
        {
            if (!_ovrHand.IsTracked || _skeleton.Bones == null || _skeleton.Bones.Count == 0)
            {
                _snapshot.IsTracked = false;
                _pinchPosFilter.Reset();
                _wristPosFilter.Reset();
                return false;
            }

            if (_lastHandUpdateFrame == Time.frameCount) return true;
            _lastHandUpdateFrame = Time.frameCount;

            var wristPose = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.ToPose();

            // todo do we still need this?
            if (Side.IsLeft()) wristPose.rotation *= Quaternion.Euler(180f, 0f, 0f);

            _bones.Wrist = wristPose;
            _bones.Forearm = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ForearmStub].Transform.ToPose();

            _bones.IndexProximal = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_Index1].Transform.ToPose();
            _bones.IndexDistal = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_Index3].Transform.ToPose();
            _bones.IndexTip = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_IndexTip].Transform.ToPose().WithRotation(_bones.IndexDistal.rotation);

            _bones.ThumbProximal = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_Thumb1].Transform.ToPose();
            _bones.ThumbDistal = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_Thumb3].Transform.ToPose();
            _bones.ThumbTip = _skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_ThumbTip].Transform.ToPose().WithRotation(_bones.ThumbDistal.rotation);

            _bones.Scale = _ovrHand.HandScale;

            var isTracked = (_ovrHand.IsTracked && _handTrackingLossSimulation.Contained(wristPose.position)) || !ARGlassesRig.IsHmdPresentAndUserPresent;

            var pinchPose = _ovrHand.PointerPose.ToPose();
            pinchPose.position = _pinchPosFilter.Step(pinchPose.position, Time.deltaTime);
            wristPose.position = _wristPosFilter.Step(wristPose.position, Time.deltaTime);

            _snapshot = new Snapshot.Hand(
                _side,
                pinchPose,
                wristPose,
                _ovrHand.PointerPose.ToPose(),
                _ovrHand.IsPointerPoseValid,
                _ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Index),
                _ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Middle),
                _ovrHand.GetFingerIsPinching(OVRHand.HandFinger.Index),
                _ovrHand.GetFingerIsPinching(OVRHand.HandFinger.Middle),
                _bones,
                isTracked);
            return true;
        }

        private bool UpdateFromController()
        {
            var ovrControllerType = _side.IsLeft() ? OVRInput.Controller.LTouch : OVRInput.Controller.RTouch;

            var positionTracked = OVRInput.GetControllerPositionTracked(ovrControllerType);
            var orientationTracked = OVRInput.GetControllerOrientationTracked(ovrControllerType);
            if (!_allowController || !positionTracked || !orientationTracked)
            {
                _snapshot.IsTracked = false;
                return false;
            }

            var indexTrigger = _side.IsLeft() ? OVRInput.Axis1D.PrimaryIndexTrigger : OVRInput.Axis1D.SecondaryIndexTrigger;
            var middleTrigger = _side.IsLeft() ? OVRInput.Axis1D.PrimaryHandTrigger : OVRInput.Axis1D.SecondaryHandTrigger;

            var indexButton = _side.IsLeft() ? OVRInput.RawButton.LIndexTrigger : OVRInput.RawButton.RIndexTrigger;
            var middleButton = _side.IsLeft() ? OVRInput.RawButton.LHandTrigger : OVRInput.RawButton.RHandTrigger;

            var pinchPose = new Pose(OVRInput.GetLocalControllerPosition(ovrControllerType), OVRInput.GetLocalControllerRotation(ovrControllerType));
            var wristPose = pinchPose.Transform(_controllerWristOffset);
            var indexStrength = OVRInput.Get(indexTrigger);
            var middleStrength = OVRInput.Get(middleTrigger);
            var indexPinch = OVRInput.Get(indexButton);
            var middlePinch = OVRInput.Get(middleButton);

            _bones = Snapshot.HandBones.Default(_side, wristPose);

            Pose systemRay = pinchPose;
            bool rayActive = true;
            var isTracked = (positionTracked && orientationTracked && _handTrackingLossSimulation.Contained(wristPose.position)) || !ARGlassesRig.IsHmdPresentAndUserPresent;
            _snapshot = new Snapshot.Hand(_side, pinchPose, wristPose, systemRay, rayActive, indexStrength, middleStrength, indexPinch, middlePinch, _bones, isTracked);
            return true;
        }
    }
}
