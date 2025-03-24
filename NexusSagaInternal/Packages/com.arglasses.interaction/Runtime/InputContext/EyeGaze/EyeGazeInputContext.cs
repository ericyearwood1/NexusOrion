using UnityEngine;

namespace ARGlasses.Interaction
{
    public class EyeGazeInputContext : InputContext
    {
        [SerializeField, ReadOnly] private Conecaster _conecaster;
        public Conecaster Conecaster => _conecaster;

        [SerializeField, ReadOnly] private ConecastResult _conecastResult;
        public ConecastResult ConecastResult => _conecastResult;

        [SerializeField, ReadOnly] private GazeVelocityBuffer _eyeVelocityBuffer;

        [SerializeField, ReadOnly] private IFixationPrediction _fixationPrediction;

        [SerializeField, ReadOnly] private Snapshot.Focus _lastFocus;
        public override Vector3 CursorWorld => _conecastResult.HitPose.position;

        [SerializeField, ReadOnly] public TargetContext _targetContext;
        public override TargetContext TargetContext => _targetContext;
        public override InputCategory Category => InputCategory.Eyes;

        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;

        protected void Awake()
        {
            this.Sibling(ref _conecaster);
            this.Sibling(ref _eyeVelocityBuffer);
            this.Scene(ref _wristband);
            _fixationPrediction = GetComponent<IFixationPrediction>();
        }

        public override Snapshot.Focus GetFocus(in Snapshot.Rig rigSnapshot)
        {
            _conecaster.PrioritizeBestHit(rigSnapshot.Gaze.Eyes.ToRay(), Target.All, out _conecastResult);

            if (_fixationPrediction != null)
            {
                Debug.LogError("Need to plumb HitSurface");
                var gazeHit = new IFixationPrediction.GazeHitInfo()
                {
                    Time = Time.time,
                    HitPosition = _conecastResult.gazePoint,
                    EyeHitSurfaceDistance = _conecastResult.gazePointDistance,
                    HitSurface = null
                };

                _fixationPrediction.AddGazeHit(gazeHit);
                //Debug.Log("Eyes hit: " + gazeHit);
            }

            // var target = _eyeVelocityBuffer.HasFixation ? _conecastResult.target : _lastFocus.Target;

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
            _wristband.WhenPinch += HandleImuResetReference;
        }

        public override void Close(in Snapshot.Rig rigSnapshot, TargetContext closingTargetContext)
        {
            _targetContext = null;
            _wristband.WhenPinch -= HandleImuResetReference;
        }

        private void HandleImuResetReference(HandFinger finger, bool isPressed)
        {
            _wristband.Streams.ResetReference();
        }
    }
}
