using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class InteractionState
    {
        public InteractionState(InteractionState other, Snapshot.Focus focus) : this(other._rigSnapshot, focus, other._timeStamp)
        {
        }

        public InteractionState(Snapshot.Rig rigSnapshot) : this(rigSnapshot, Snapshot.Focus.Empty())
        {
        }

        public InteractionState(Snapshot.Rig rigSnapshot, Snapshot.Focus focus, float time = -1)
        {
            _timeStamp = time < 0 ? Time.time : time;
            _rigSnapshot = rigSnapshot;
            _focus = focus;
        }

        [SerializeField, ReadOnly] private float _timeStamp;
        public float TimeStamp => _timeStamp;

        [SerializeField, ReadOnly] private Snapshot.Rig _rigSnapshot;

        [SerializeField, ReadOnly] private Snapshot.Focus _focus;
        public Snapshot.Focus Focus => _focus;
        public bool HasTarget => _focus.HasTarget;

        public Side PrimarySide => _focus.PrimarySide;
        public Target Target => _focus.Target;
        public Pose TargetPose => _focus.TargetPose;
        public Pose TargetHit => _focus.TargetHit;
        public bool IsPressingPrimary => _focus.IsPressingPrimary;
        public bool IsPressingSecondary => _focus.IsPressingSecondary;

        public Pose Head => _rigSnapshot.Gaze.Head;
        public Pose Eyes => _rigSnapshot.Gaze.Eyes;

        private Snapshot.Hand PrimaryHand => _rigSnapshot.GetHand(PrimarySide);
        public Pose Pinch => PrimaryHand.Pinch;

        public Pose Wrist => PrimaryHand.Wrist;
        public Pose Ray => PrimaryHand.Ray;
        public bool RayActive => PrimaryHand.RayActive;
        public DPad DPad => _rigSnapshot.Wristband.DPad;
        public DiscreteHandMotion DiscreteHandMotion => _rigSnapshot.Wristband.DiscreteHandMotion;
        public Vector2 ImuProjectedDeltaRadians => _rigSnapshot.Wristband.ProjectedRadians;
        public float TwistDegrees => _rigSnapshot.Wristband.TwistDegrees;
        public bool IsWristbandConnected => _rigSnapshot.Wristband.IsConnected;

        public Pose GetHead(DragSpace space) => FromWorldSpace(space, Head);
        public Pose GetEyes(DragSpace space) => FromWorldSpace(space, Eyes);
        public Pose GetPinch(DragSpace space) => FromWorldSpace(space, Pinch);
        public Pose GetWrist(DragSpace space) => FromWorldSpace(space, Wrist);
        public Pose GetTargetHit(DragSpace space) => FromWorldSpace(space, TargetHit);
        public Pose GetTargetPose(DragSpace space) => FromWorldSpace(space, TargetPose);

        public Pose FromWorldSpace(DragSpace toSpace, Pose pose)
        {
            if (toSpace.IsTarget()) return ToTargetSpace(pose);
            if (toSpace.IsHead()) return ToHeadSpace(pose);
            return pose;
        }

        // todo need a way for InputContext to be in charge of using CV vs Wristband...
        public bool IsPalmUpCV(float dot = 0.7f) => Vector3.Dot(Wrist.up * (PrimarySide.IsLeft() ? -1 : 1), (Wrist.position - Head.position).normalized) > dot;

        public bool IsPalmUpImu(float dot = 0.5f)
        {
            var orientation = _rigSnapshot.Wristband.OrientationNotAlignedInYaw;
            var wristbandPalmDir = orientation * Vector3.down;

            // because we cannot trust the 'real world' offset in yaw between Wristband's IMU and user's head,
            // so we need to be a little generous with a world-space check instead.  we may need to dial this dot value down further.
            return Vector3.Dot(wristbandPalmDir, Vector3.up) > dot;
        }

        public bool IsAlignedToHeadPose(float dot = 0.5f) => Vector3.Dot(Pinch.forward, Head.forward) > dot;

        public Pose FromHeadSpace(Pose pose) => Head.Transform(pose);
        public Pose ToHeadSpace(Pose pose) => Head.InverseTransform(pose);
        public Vector3 FromHeadSpace(Vector3 position) => Head.TransformPoint(position);
        public Vector3 ToHeadSpace(Vector3 position) => Head.InverseTransformPoint(position);

        public Pose FromTargetSpace(Pose pose) => Target == null ? pose : TargetPose.Transform(pose);
        public Pose ToTargetSpace(Pose pose) => Target == null ? pose : TargetPose.InverseTransform(pose);
    }
}
