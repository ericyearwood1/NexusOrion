using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static partial class Snapshot
    {
        [Serializable]
        public struct Focus
        {
            public static Focus Empty() => Empty(Side.None, false, false);
            public static Focus Empty(Side side, bool isPressingPrimary, bool isPressingSecondary) => new(null, Pose.identity, Pose.identity, side, isPressingPrimary, isPressingSecondary);

            [SerializeField, ReadOnly] private Target _target;
            [SerializeField, ReadOnly] private Pose _targetHit;
            [SerializeField, ReadOnly] private Pose _targetPose;
            [SerializeField, ReadOnly] private Side _primarySide;
            [SerializeField, ReadOnly] private bool _isPressingPrimary;
            [SerializeField, ReadOnly] private bool _isPressingSecondary;

            public Focus(Target target, Side primarySide, bool isPressingPrimary, bool isPressingSecondary) : this(target, target.transform.ToPose(), target.transform.ToPose(), primarySide, isPressingPrimary, isPressingSecondary)
            {
            }

            public Focus(Target target, Pose targetHit, Pose targetPose, Side primarySide, bool isPressingPrimary, bool isPressingSecondary)
            {
                _target = target;
                _targetHit = targetHit;
                _targetPose = targetPose;
                _primarySide = primarySide;
                _isPressingPrimary = isPressingPrimary;
                _isPressingSecondary = isPressingSecondary;
            }

            public Focus(ConecastResult conecastResult, Side primarySide, bool isPressingPrimary, bool isPressingSecondary)
            {
                var targetPose = conecastResult.HasTarget ? conecastResult.target.transform.ToPose() : default;
                _target = conecastResult.target;
                _targetHit = conecastResult.HitPose;
                _targetPose = targetPose;
                _primarySide = primarySide;
                _isPressingPrimary = isPressingPrimary;
                _isPressingSecondary = isPressingSecondary;
            }

            public Target Target => _target;
            public Pose TargetHit => _targetHit;
            public Pose TargetPose => _targetPose;
            public bool HasTarget => _target != null;
            public Side PrimarySide => _primarySide;
            public bool IsPressingPrimary => _isPressingPrimary;
            public bool IsPressingSecondary => _isPressingSecondary;

        }

    }
}
