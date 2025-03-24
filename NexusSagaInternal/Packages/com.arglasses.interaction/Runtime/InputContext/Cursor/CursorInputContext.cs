using System;
using System.Collections;
using System.Linq;
using CTRL.Math;
using UnityEngine;
using static ARGlasses.Interaction.MagneticGlowCursorVisual;

namespace ARGlasses.Interaction
{
    public class CursorInputContext : InputContext
    {
        [SerializeField] private TargetContext _leashedSurface;

        [SerializeField] private float _conecastSurfaceOffset = 0.5f;

        public override InputCategory Category => InputCategory.Cursor;

        [SerializeField] public TargetContextVisual.Style _style = TargetContextVisual.Style.Outline;

        [SerializeField] private float _handTrackingSensitivity = 2.5f;

        public event Action<float> WhenCursorSensitivityUpdated = delegate { };
        [SerializeField] private float _imuSensitivity = ARGlassesWristbandStreams.DefaultImuSensitivity;
        public float ImuSensitivity
        {
            get => _imuSensitivity;
            set
            {
                if (Mathf.Abs(_imuSensitivity - value) < float.Epsilon) return;
                _imuSensitivity = value;
                WhenCursorSensitivityUpdated(value);
            }
        }

        public event Action<float> WhenCursorSnapRadiusUpdated = delegate { };
        public float SnapRadius
        {
            get => _conecaster.ConeRadiusDegrees;
            set
            {
                if (Mathf.Abs(_conecaster.ConeRadiusDegrees - value) < float.Epsilon) return;
                _conecaster.ConeRadiusDegrees = value;
                WhenCursorSnapRadiusUpdated(value);
            }
        }
        public const float DefaultSnapRadius = 1.5f;

        public PanelTransition CycleCursor()
        {
            PanelTransitionMode = (PanelTransition)(((int)PanelTransitionMode + 1) % (int)PanelTransition.MAX);
            return PanelTransitionMode;
        }

        public void CycleCursorStyle()
        {
            _cursorVisual.CycleMode();
        }

        public event Action<PanelTransition> WhenCursorModeUpdated = delegate { };

        public const PanelTransition DefaultPanelTransition = PanelTransition.HeadNearestPoint;
        [SerializeField] private PanelTransition _panelTransitionMode = DefaultPanelTransition;
        public PanelTransition PanelTransitionMode
        {
            get => _panelTransitionMode;
            set
            {
                if (_panelTransitionMode == value) return;
                _panelTransitionMode = value;
                WhenCursorModeUpdated(value);
            }
        }

        public enum PanelTransition
        {
            HeadNearestPoint,
            HeadPreserveLocalUnscaled,
            HeadPreserveLocalScaled,
            GazePreserveLocal,
            GazeNearestPoint,
            Continuous,
            LeashedSurface,
            Center,
            Bottom,
            Top,
            MAX,
        }

        [SerializeField, ReadOnly] private TargetContext _previousTargetContext;
        [SerializeField, ReadOnly] public TargetContext _targetContext;
        public override TargetContext TargetContext => _targetContext;

        [SerializeField, ReadOnly] private ARGlassesWristbandStreams _wristbandStreams;

        [SerializeField, ReadOnly] private Conecaster _conecaster;
        public Conecaster Conecaster => _conecaster;

        [SerializeField, ReadOnly] private ConecastResult _conecastResult;
        public ConecastResult ConecastResult => _conecastResult;

        [SerializeField, ReadOnly] private Snapshot.Focus _lastFocus;
        [SerializeField, ReadOnly] private Vector3 _cursorLocal;

        public override Vector3 CursorWorld => _targetContext ? _targetContext.ToWorldSpace(_cursorLocal) : default;

        private CursorVisual _cursorVisual;
        public CursorVisual CursorVisual => _cursorVisual;

        [SerializeField, ReadOnly] private Vector2 _lastImuReferenceDelta;
        private void HandleReferenceReset() => _lastImuReferenceDelta = default;
        [SerializeField] private Pose _previousWrist = Pose.identity;

        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;
        private TargetContext _lastTargetContext;
        private bool _isPressingPrimary;
        private bool _isPressingSecondary;

        private void Awake()
        {
            this.Scene(ref _wristbandStreams);
            this.Scene(ref _wristband);
            this.Descendant(ref _leashedSurface, nameStrict: nameof(_leashedSurface), includeInactive: true);
            this.Ensure(ref _conecaster);
            this.Ensure(ref _cursorVisual);
            _conecaster.ConeRadiusDegrees = DefaultSnapRadius;
            _cursorVisual.enabled = false;
        }

        private IEnumerator Start()
        {
            // hack: trigger value updates
            yield return new WaitForEndOfFrame();
            WhenCursorSensitivityUpdated(ImuSensitivity);
            WhenCursorSnapRadiusUpdated(SnapRadius);
            PanelTransitionMode = _panelTransitionMode;
        }

        public override void Open(in Snapshot.Rig rigSnapshot, TargetContext targetContext)
        {
            _cursorVisual.enabled = true;
            if (_targetContext != null) _previousTargetContext = _targetContext;

            _targetContext = targetContext;
            _wristbandStreams.ResetReference();
            _wristbandStreams.PreResetReference += HandleReferenceReset;
            _lastImuReferenceDelta = _wristbandStreams.ProjectedRadians;

            TargetContextVisual.SetStyle(_style);
            _targetContext.SetFocused(true);

            CursorSurfaceChanged();
        }

        public override void Close(in Snapshot.Rig rigSnapshot, TargetContext closingTargetContext)
        {
            TargetContextVisual.SetStyle(TargetContextVisual.Style.None);

            if (_targetContext != closingTargetContext) Debug.LogError($"ClosingCursorSurface was different from cached CursorSurface :( \n{_targetContext}\n{closingTargetContext}");
            if (_targetContext != null) _targetContext.SetFocused(false);
            else Debug.LogError("_cursorSurface null");

            _cursorVisual.enabled = false;
            _wristbandStreams.PreResetReference -= HandleReferenceReset;
        }

        private void CursorSurfaceChanged()
        {
            RevealStyle revealStyle = RevealStyle.Hide;

            var previousLocal = _cursorLocal;
            if (PanelTransitionMode == PanelTransition.HeadPreserveLocalUnscaled) { }
            if (PanelTransitionMode == PanelTransition.HeadPreserveLocalScaled && _previousTargetContext) PreserveLocalScaled();
            if (PanelTransitionMode == PanelTransition.GazePreserveLocal) PreserveLocalScaled();
            if (PanelTransitionMode == PanelTransition.GazeNearestPoint) NearestPoint();
            if (PanelTransitionMode == PanelTransition.HeadNearestPoint) NearestPoint();
            if (PanelTransitionMode == PanelTransition.LeashedSurface) _cursorLocal = Vector3.zero;
            if (PanelTransitionMode == PanelTransition.Continuous)
            {
                revealStyle = RevealStyle.Show;
                var previousWorld = _previousTargetContext ? _previousTargetContext.ToWorldSpace(_cursorLocal) : _targetContext.transform.position;
                _cursorLocal = _targetContext.ConstrainWorldPointToSurfaceLocal(previousWorld);
            }

            if (PanelTransitionMode == PanelTransition.Center) _cursorLocal = Vector3.zero;
            if (PanelTransitionMode == PanelTransition.Bottom) _cursorLocal = Vector3.down * 5;
            if (PanelTransitionMode == PanelTransition.Top) _cursorLocal = Vector3.up * 5;

            _cursorVisual.CursorSurfaceChanged(revealStyle);

            void NearestPoint()
            {
                if (!_previousTargetContext) return;
                var previousWorld = _previousTargetContext.ToWorldSpace(previousLocal);
                _cursorLocal = _targetContext.ConstrainWorldPointToSurfaceLocal(previousWorld);
            }

            void PreserveLocalScaled()
            {
                var currentSize = _targetContext.Collider.size;
                var previousSize = _previousTargetContext.Collider.size;
                _cursorLocal.x = previousLocal.x * (currentSize.x / previousSize.x);
                _cursorLocal.y = previousLocal.y * (currentSize.y / previousSize.y);
            }
        }

        private Ray CursorRay => new(CursorWorld + _targetContext.Forward * _conecastSurfaceOffset * -1, _targetContext.Forward);

        public override TargetContext BestSurface(in Snapshot.Rig rigSnapshot)
        {
            if (PanelTransitionMode == PanelTransition.LeashedSurface)
            {
                _leashedSurface.gameObject.SetActive(true);
                _lastTargetContext = _leashedSurface;
                return _leashedSurface;
            }
            _leashedSurface.gameObject.SetActive(false);

            if (IsPinchingPrimary && _lastTargetContext) return _lastTargetContext;

            if (PanelTransitionMode == PanelTransition.Continuous && _lastTargetContext) return _lastTargetContext;

            if (PanelTransitionMode is PanelTransition.GazePreserveLocal or PanelTransition.GazeNearestPoint)
            {
                _lastTargetContext = ConecastTargetContext(rigSnapshot.Gaze.Eyes.ToRay()).Context;
                return _lastTargetContext;
            }

            _lastTargetContext = base.BestSurface(rigSnapshot);
            return _lastTargetContext;
        }

        private bool IsPinchingPrimary => _isPressingPrimary;

        public override Snapshot.Focus GetFocus(in Snapshot.Rig rigSnapshot)
        {
            var rightSideOnly = Side.Right;
            _isPressingPrimary = IsPinching(rigSnapshot, rightSideOnly, HandFinger.Index);
            _isPressingSecondary = IsPinching(rigSnapshot, rightSideOnly, HandFinger.Middle);

            Vector3 cursorLocalDelta = default;

            if (!rigSnapshot.Wristband.IsConnected && !Application.isEditor)
            {
                Pose wristPose = rigSnapshot.RightHand.Wrist;
                if (_previousWrist == Pose.identity) _previousWrist = wristPose;

                cursorLocalDelta = _targetContext.ToLocalSpace(wristPose.position) - _targetContext.ToLocalSpace(_previousWrist.position);
                cursorLocalDelta = cursorLocalDelta.WithZ(0);
                cursorLocalDelta *= _imuSensitivity * _handTrackingSensitivity;
                _previousWrist = wristPose;
            }
            else
            {
                var relativeImu = _wristbandStreams.ProjectedRadians;
                cursorLocalDelta.x = MathUtil.DeltaAngleRadian(_lastImuReferenceDelta.x, relativeImu.x);
                cursorLocalDelta.y = MathUtil.DeltaAngleRadian(_lastImuReferenceDelta.y, relativeImu.y);
                cursorLocalDelta *= _imuSensitivity;
                _lastImuReferenceDelta = relativeImu;
            }

#if false && UNITY_EDITOR
            cursorLocalDelta.x = Input.GetAxis("Mouse X") * 0.02f;
            cursorLocalDelta.y = Input.GetAxis("Mouse Y") * 0.02f;
#endif
            _cursorLocal = ApplyCursorDelta(cursorLocalDelta);

            if (_conecaster.PrioritizeBestHit(CursorRay, _targetContext.ActiveTargets, out _conecastResult))
            {
                return _lastFocus = new Snapshot.Focus(_conecastResult, rightSideOnly, _isPressingPrimary, _isPressingSecondary);
            }

            return _lastFocus = Snapshot.Focus.Empty(rightSideOnly, _isPressingPrimary, _isPressingSecondary);
        }

        [SerializeField] private float _continuousSurfaceHopDistance = 0.3f;

        // only if we are not pinching, constrain to the TargetContext boundaries and reset the IMU reference if hitting a boundary
        private Vector3 ApplyCursorDelta(Vector3 deltaLocal)
        {
            var newCursorLocal = _cursorLocal + deltaLocal;
            if (IsPinchingPrimary) return newCursorLocal;

            var constraintedCursorLocal = _targetContext.ConstrainLocalPointToSurfaceLocal(newCursorLocal);
            var constraint = newCursorLocal - constraintedCursorLocal;
            if (constraint.IsNearZero()) return newCursorLocal;

            if (PanelTransitionMode == PanelTransition.Continuous)
            {
                var surfaceHopPosition = _targetContext.ToWorldSpace(_cursorLocal + constraint.normalized * _continuousSurfaceHopDistance);
                _targetContext.FindNearestTargetContext(surfaceHopPosition, out var nearest, TargetContext.All.Where(c => !c.name.Contains("Debug")));
                if (nearest != _targetContext)
                {
                    _lastTargetContext = nearest;
                    _wristbandStreams.ResetReference();
                    return newCursorLocal;
                }
            }

            _wristbandStreams.ResetReference();
            return constraintedCursorLocal;
        }
    }
}
