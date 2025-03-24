using System;
using CTRL.Math;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.SocialPlatforms;

namespace ARGlasses.Interaction
{
    public static class ExtensionsDragSpace
    {
        public static bool IsWorld(this DragSpace space) => space == DragSpace.World;
        public static bool IsTarget(this DragSpace space) => space == DragSpace.Local;
        public static bool IsHead(this DragSpace space) => space == DragSpace.Head;
    }

    public enum DragSpace
    {
        World,
        Local,
        Head
    }

    public interface IDrag
    {
        float Magnitude => DeltaWorld.magnitude;
        float Duration { get; }
        Pose DeltaPose(DragSpace space);
        Vector3 GetDelta(DragSpace space = DragSpace.World);

        /// <summary>
        /// Delta vector of the Drag in Worldspace from beginning of pinch to current frame
        /// </summary>
        Vector3 DeltaWorld { get; }

        /// <summary>
        /// Delta vector of the Drag in Target's Localspace from beginning of pinch to current frame.
        /// Calculated using a cached Pose form Target's initial position, and does not account for subsequent Target motion
        /// </summary>
        Vector3 DeltaLocal { get; }

        /// <summary>
        /// Delta vector of the Drag in Headspace from beginning of pinch to current frame
        /// </summary>
        Vector3 DeltaHead { get; }

        Pose OriginWorld { get; }
        Pose DestinationWorld { get; }
    }

    [Serializable]
    public class ImuDrag : Drag
    {
        public static Vector3 LocalScale = new(0.5f, 0.5f, 0.5f);

        [SerializeField, ReadOnly] private Vector2 _beginProjectedDeltaRadians;
        [SerializeField, ReadOnly] private Vector2 _latestProjectedDeltaRadians;
        [SerializeField, ReadOnly] private Vector3 _deltaRadian;

        private Pose ReferencePose => Begin.TargetPose;
        protected override Pose WorldSpaceOrigin
        {
            get
            {
                _origin = ReferencePose;
                return _origin;
            }
        }

        protected override Pose WorldSpaceDestination
        {
            get
            {
                _beginProjectedDeltaRadians = Begin.ImuProjectedDeltaRadians; // radians
                _latestProjectedDeltaRadians = Latest.ImuProjectedDeltaRadians;

                _deltaRadian.x = MathUtil.DeltaAngleRadian(_beginProjectedDeltaRadians.x, _latestProjectedDeltaRadians.x);
                _deltaRadian.y = MathUtil.DeltaAngleRadian(_beginProjectedDeltaRadians.y, _latestProjectedDeltaRadians.y);

                var destinationPositionLocal = Vector3.Scale(_deltaRadian, LocalScale);
                var destinationRotation = ReferencePose.rotation;

                _destination = new Pose(ReferencePose.TransformPoint(destinationPositionLocal), destinationRotation);
                return _destination;
            }
        }

        public ImuDrag(InteractionState latest, InteractionState begin, float scale = 1) : base(latest, begin, scale)
        {
        }
    }

    [Serializable]
    public class Drag : IDrag
    {
        [SerializeField, ReadOnly] private InteractionState _latest;
        [SerializeField, ReadOnly] private InteractionState _begin;

        [SerializeField, ReadOnly] protected Pose _origin;
        [SerializeField, ReadOnly] protected Pose _destination;
        [SerializeField, ReadOnly] protected Vector3 _lastDeltaTarget;
        [SerializeField, ReadOnly] protected float _lastDeltaTargetMagnitude;
        protected float Scale;

        public InteractionState Latest => _latest;
        public InteractionState Begin => _begin;

        public Vector3 DeltaWorld => GetDelta(DragSpace.World);
        public Vector3 DeltaLocal => GetDelta(DragSpace.Local);
        public Vector3 DeltaHead => GetDelta(DragSpace.Head);
        public Pose OriginWorld => WorldSpaceOrigin;
        public Pose DestinationWorld => WorldSpaceDestination;

        public float Duration => _latest.TimeStamp - _begin.TimeStamp;

        protected virtual Pose WorldSpaceOrigin
        {
            get
            {
                _origin = Begin.Pinch;
                return _origin;
            }
        }

        protected virtual Pose WorldSpaceDestination
        {
            get
            {
                var delta = (Latest.Pinch.position - Begin.Pinch.position) * Scale;

                _destination = Latest.Pinch;
                _destination.position = delta + Begin.Pinch.position;
                return _destination;
            }
        }

        public Vector3 GetDelta(DragSpace space) => DeltaPose(space).position;
        public Quaternion GetDeltaRotation(DragSpace space) => DeltaPose(space).rotation;

        public Pose DeltaPose(DragSpace space)
        {
            var begin = WorldSpaceOrigin;
            var current = WorldSpaceDestination;

            if (space.IsTarget())
            {
                var targetPose = _begin.TargetPose;
                var localBegin = targetPose.InverseTransform(begin);
                var localCurrent = targetPose.InverseTransform(current);
                var deltaTarget = localCurrent.Subtract(localBegin);
                _lastDeltaTarget = deltaTarget.position;
                _lastDeltaTargetMagnitude = _lastDeltaTarget.magnitude;
                return deltaTarget;
            }

            if (space.IsHead())
            {
                var headPose = _begin.Head;
                var localBegin = headPose.InverseTransform(begin);
                var localCurrent = headPose.InverseTransform(current);
                return localCurrent.Subtract(localBegin);
            }

            var deltaWorld = current.Subtract(begin);
            return deltaWorld;
        }

        public Pose OrbitalPointerBegin() => _begin.TargetHit;

        public Pose OrbitalPointerLatest()
        {
            var beginPinch = WorldSpaceOrigin;
            var latestPinch = WorldSpaceDestination;

            Debug.DrawLine(beginPinch.position, latestPinch.position, Color.green);

            var beginHeadToPinch = beginPinch.position - _begin.Head.position;
            var latestHeadToPinch = latestPinch.position - _latest.Head.position;
            var rotation = Quaternion.FromToRotation(beginHeadToPinch.normalized, latestHeadToPinch.normalized);
            var rotatedHeadToHit = rotation * (_begin.TargetHit.position - _begin.Head.position);

            var beginHeadToPinchDist = Mathf.Max(beginHeadToPinch.magnitude, 0.001f);
            var latestHeadToPinchDist = Mathf.Max(latestHeadToPinch.magnitude, 0.001f);

            var distanceScaler = latestHeadToPinchDist / beginHeadToPinchDist;
            var position = _latest.Head.position + (rotatedHeadToHit * distanceScaler);

            var orbitalPointerLatest = new Pose(position, rotation * _begin.TargetHit.rotation);
            Debug.DrawLine(_begin.TargetHit.position, orbitalPointerLatest.position, Color.red);

            return orbitalPointerLatest;
        }

        public Pose OrbitalDelta() => OrbitalPointerLatest().Subtract(OrbitalPointerBegin());

        public Drag(InteractionState latest, InteractionState begin, float scale = 1f)
        {
            _latest = latest;
            _begin = begin;
            Scale = scale;
            Assert.IsTrue(Duration >= 0);
        }

        // private class Wrist
        // {
        //     private Vector3 _wrist
        //
        //     private float _wristRollDeltaAngle;
        //     private float _wristRollTotalAngle;
        //     public float WristRollDeltaAngle => _wristRollDeltaAngle;
        //     public float WristRollTotalAngle => _wristRollTotalAngle;
        //
        //     private float _wristYawDeltaAngle;
        //     private float _wristYawTotalAngle;
        //     public float WristYawDeltaAngle => _wristYawDeltaAngle;
        //     public float WristYawTotalAngle => _wristYawTotalAngle;
        //
        //     private float _wristPitchDeltaAngle;
        //     private float _wristPitchTotalAngle;
        //     public float WristPitchDeltaAngle => _wristPitchDeltaAngle;
        //     public float WristPitchTotalAngle => _wristPitchTotalAngle;
        //
        //     public void Update()
        //     {
        //         var begin = new Pose();
        //         var current = new Pose();
        //
        //         _wristRollDeltaAngle = begin.right.SignedAngleAlongAxis(current.right, begin.forward);
        //         _wristRollTotalAngle += _wristRollDeltaAngle;
        //
        //         _wristYawDeltaAngle = begin.forward.SignedAngleAlongAxis(current.forward, begin.up);
        //         _wristYawTotalAngle += _wristYawDeltaAngle;
        //
        //         _wristPitchDeltaAngle = begin.forward.SignedAngleAlongAxis(current.forward, begin.right);
        //         _wristPitchTotalAngle += _wristPitchDeltaAngle;
        //     }
        // }
    }
}
