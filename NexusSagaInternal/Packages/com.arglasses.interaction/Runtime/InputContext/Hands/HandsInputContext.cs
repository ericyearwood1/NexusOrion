using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class HandsInputContext : InputContext
    {
        public const float DefaultConeRadiusDegrees = 1.5f;
        public override bool SuppressWristband => true;

        public override InputCategory Category => InputCategory.Hands;
        [SerializeField] public TargetContextVisual.Style _style = TargetContextVisual.Style.Outline;
        [SerializeField] private bool _forceDirectRay = false;
        public void CycleCursorStyle() => _cursorVisual.CycleMode();

        [SerializeField, ReadOnly] public TargetContext _targetContext;
        public override TargetContext TargetContext => _targetContext;

        [SerializeField, ReadOnly] private Conecaster _conecaster;
        public Conecaster Conecaster => _conecaster;

        [SerializeField, ReadOnly] private ConecastResult _conecastResult;
        public ConecastResult ConecastResult => _conecastResult;

        [SerializeField, ReadOnly] private Snapshot.Focus _lastFocus;
        [SerializeField, ReadOnly] private Vector3 _cursorWorld;
        public override Vector3 CursorWorld => _cursorWorld;

        private CursorVisual _cursorVisual;
        public CursorVisual CursorVisual => _cursorVisual;

        public const float DefaultRayPitch = 30;
        private static float _rayPitch = DefaultRayPitch;

        public static readonly Vector3 DefaultRayOffset = new Vector3(0f, 0.28f, 0.075f);
        private static Vector3 _rayOffset = DefaultRayOffset;

        public const float DefaultRayPitchAccelerationBegin = 0;
        private static float _rayPitchAccelerationBegin = DefaultRayPitchAccelerationBegin;

        private static float _rayPitchAccelerationPow = 1;

        public GameManager gameManager;

        public static void DisableRayPitch()
        {
            RayPitch = 0;
            RayOffset = Vector3.zero;
            RayPitchAccelerationBegin = 60;
            _rayPitchAccelerationPow = 1;
        }

        public static void ResetRayPitch()
        {
            RayPitch = DefaultRayPitch;
            RayOffset = DefaultRayOffset;
            RayPitchAccelerationBegin = DefaultRayPitchAccelerationBegin;
            _rayPitchAccelerationPow = 1;
        }

        private static Quaternion _wristRayOffset;

        public static Vector3 RayOffset
        {
            get => _rayOffset;
            set => _rayOffset = value;
        }

        public static float RayPitch
        {
            get => _rayPitch;
            set => _rayPitch = value;
        }

        public static float RayPitchAccelerationBegin
        {
            get => _rayPitchAccelerationBegin;
            set => _rayPitchAccelerationBegin = value;
        }

        public static float RayPitchAccelerationPow
        {
            get => _rayPitchAccelerationPow;
            set => _rayPitchAccelerationPow = value;
        }

        public Pose PitchedRay => _pitchedRay;
        public Pose AcceleratedRay => _acceleratedRay;

        public static Quaternion WristRayOffset
        {
            get => _wristRayOffset;
            set => _wristRayOffset = value;
        }

        private void Awake()
        {
            this.Ensure(ref _conecaster);
            this.Ensure(ref _cursorVisual);
            _conecaster.ConeRadiusDegrees = DefaultConeRadiusDegrees;
            _cursorVisual.enabled = false;
        }

        public override void Open(in Snapshot.Rig rigSnapshot, TargetContext targetContext)
        {
            _targetContext = targetContext;
        }

        public override void Close(in Snapshot.Rig rigSnapshot, TargetContext closingTargetContext)
        {
            _targetContext = null;
            _cursorVisual.enabled = false;
        }

        [SerializeField, ReadOnly] private Pose _pitchedRay = Pose.identity;
        [SerializeField, ReadOnly] private Pose _acceleratedRay = Pose.identity;

        private static HandsInputContext _instance;

        public static HandsInputContext Instance
        {
            get
            {
                if (!_instance) _instance = FindFirstObjectByType<HandsInputContext>(FindObjectsInactive.Include);
                return _instance;
            }
        }

        private Pose GetPitchedRayPose(Snapshot.Rig rigSnapshot, Side side)
        {
            if(_forceDirectRay) DisableRayPitch();

            var handSnapshot = rigSnapshot.GetHand(side);

            var wristForward = handSnapshot.Wrist.right * -1;
            wristForward = WristRayOffset * wristForward;
            var wristRotation = Quaternion.LookRotation(wristForward);
            var wristPose = new Pose(handSnapshot.Wrist.position, wristRotation);

            var rayPose = handSnapshot.Ray;

            if (!rigSnapshot.IsUserInHmd) return rayPose;

            var head = rigSnapshot.Gaze.Head;
            var headRightFlat = head.right.WithY(0).normalized;
            var headForwardFlat = head.forward.WithY(0).normalized;

            rayPose.position += Vector3.up * _rayOffset.y;
            rayPose.position += headRightFlat * _rayOffset.x;
            rayPose.position += headForwardFlat * _rayOffset.z;

            var forward = rayPose.forward;
            var rightAxis = Vector3.Cross(forward, Vector3.up);

            var signedRayPitch = Vector3.SignedAngle(forward.WithY(0), forward, rightAxis);
            var accelerationDegrees = Mathf.Max(0, signedRayPitch - RayPitchAccelerationBegin);

            _acceleratedRay = rayPose;
            var acceleratedPitch = RayPitch + Mathf.Pow(accelerationDegrees, RayPitchAccelerationPow);
            _acceleratedRay.rotation = Quaternion.LookRotation(Quaternion.AngleAxis(acceleratedPitch, rightAxis) * forward);
            return _acceleratedRay;
        }

        [SerializeField] private Side _activeSide = Side.Right;
        [SerializeField] private bool _rayActive;

        public override Snapshot.Focus GetFocus(in Snapshot.Rig rigSnapshot)
        {
            // var isPinchingSide = rigSnapshot.IsPinching(HandFinger.Index);
            var side = GetBestHandSide(rigSnapshot);
            var primary = IsPinching(rigSnapshot, side, HandFinger.Index);
            var secondary = IsPinching(rigSnapshot, side, HandFinger.Middle);

            if (primary == true)
            {
                Debug.Log("isPinching");
                gameManager.pinchBool = true;
            }
            else { gameManager.pinchBool = false; }

            _rayActive = rigSnapshot.GetHand(side).RayActive;
            _cursorVisual.enabled = _rayActive;

            var ray = GetPitchedRayPose(rigSnapshot, side).ToRay();
            var hit = _targetContext.RaycastWorldSpace(ray);

            // _cursorWorld = hit.PlaneHit;
            _cursorWorld = hit.ColliderSnapPoint;

            var isTargetContextHit = hit.IsPlaneHit && _targetContext.isActiveAndEnabled;
            if (!isTargetContextHit) return Snapshot.Focus.Empty(side, primary, secondary);

            if (!_conecaster.PrioritizeBestHit(ray, Target.All, out _conecastResult)) return Snapshot.Focus.Empty(side, primary, secondary);
            _lastFocus = new Snapshot.Focus(_conecastResult, side, primary, secondary);
            return _lastFocus;
        }

        public virtual Side GetBestHandSide(Snapshot.Rig rigSnapshot)
        {
            if (_activeSide != Side.Both) return _activeSide;

            var leftTracked = rigSnapshot.LeftHand.IsTracked;
            var rightTracked = rigSnapshot.RightHand.IsTracked;

            //are both hands tracked?
            if (leftTracked && rightTracked)
            {
                var leftPointerPoseValid = rigSnapshot.LeftHand.RayActive;
                var rightPointerPoseValid = rigSnapshot.RightHand.RayActive;
                if (leftPointerPoseValid && rightPointerPoseValid)
                {
                    //return whichever wrist is higher. assume other is by side
                    return (rigSnapshot.LeftHand.Wrist.position.y > rigSnapshot.RightHand.Wrist.position.y)
                        ? Side.Left
                        : Side.Right;
                }

                //only one side has valid pose
                return leftPointerPoseValid ? Side.Left : Side.Right;
            }

            //only one side is tracked
            return leftTracked ? Side.Left : Side.Right;

        }

        public override TargetContext BestSurface(in Snapshot.Rig rigSnapshot)
        {
            var ray = GetPitchedRayPose(rigSnapshot, _activeSide).ToRay();
            var candidateHit = ConecastTargetContext(ray);
            return candidateHit.Context;
        }

        public override Drag CreateDrag(InteractionState latest, InteractionState begin)
        {
            // always CV Drag, never IMU
            return new Drag(latest, begin);
        }
    }
}
