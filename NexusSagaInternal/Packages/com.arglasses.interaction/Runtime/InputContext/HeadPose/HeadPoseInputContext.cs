using UnityEngine;

namespace ARGlasses.Interaction
{
    public class HeadPoseInputContext : InputContext
    {
        public override InputCategory Category => InputCategory.Cursor;

        [SerializeField,] private float _downwardPitch = 4f;
        public float DownwardPitch => _downwardPitch;
        [SerializeField,] private float _downwardMultiplier = 1.8f;
        public float DownwardMultiplier => _downwardMultiplier;

        [SerializeField, ReadOnly] private Conecaster _conecaster;
        public Conecaster Conecaster => _conecaster;

        [SerializeField, ReadOnly] private ConecastResult _conecastResult;
        public ConecastResult ConecastResult => _conecastResult;

        [SerializeField, ReadOnly] private Snapshot.Focus _lastFocus;
        [SerializeField, ReadOnly] private Vector3 _cursorWorld;
        public override Vector3 CursorWorld => _cursorWorld;

        [SerializeField, ReadOnly] public TargetContext _targetContext;
        public override TargetContext TargetContext => _targetContext;
        private HeadPoseCursorVisual _cursor;

        protected void Awake()
        {
            this.Sibling(ref _conecaster);
            this.Ensure(ref _cursor);
        }

        public override Snapshot.Focus GetFocus(in Snapshot.Rig rigSnapshot)
        {
            // get modified head ray
            var rayOrigin = rigSnapshot.Gaze.Head.position;
            var rayDirection = rigSnapshot.Gaze.Head.forward;
            var rayRotation = Quaternion.Euler(_downwardPitch, 0, 0);
            rayDirection = rayDirection.RotateAround(rigSnapshot.Gaze.Head.right, rayRotation);

            // pitch further down as you move your head down
            if (rayDirection.y < 0) rayDirection.y *= _downwardMultiplier;
            rayDirection.Normalize();

            Ray headRay = new Ray(rayOrigin, rayDirection);

            // set cursor position
            Plane cursorSurfacePlane = new Plane(-_targetContext.Forward, _targetContext.transform.position);
            if (cursorSurfacePlane.Raycast(headRay, out var hitDistance)) _cursorWorld = headRay.origin + hitDistance * headRay.direction;

            // get target
            _conecaster.PrioritizeBestHit(headRay, _targetContext.ActiveTargets, out _conecastResult);

            var target = _conecastResult.target;
            var hitPose = _conecastResult.HitPose;
            var targetPose = target ? target.transform.ToPose() : default;

            var rightSideOnly = Side.Right;
            var primary = IsPinching(rigSnapshot, rightSideOnly, HandFinger.Index);
            var secondary = IsPinching(rigSnapshot, rightSideOnly, HandFinger.Middle);
            _lastFocus = new Snapshot.Focus(target, hitPose, targetPose, rightSideOnly, primary, secondary);
            return _lastFocus;
        }

        public override void Open(in Snapshot.Rig rigSnapshot, TargetContext targetContext)
        {
            _targetContext = targetContext;
        }

        public override void Close(in Snapshot.Rig rigSnapshot, TargetContext closingTargetContext)
        {
            _targetContext = null;
        }
    }
}
