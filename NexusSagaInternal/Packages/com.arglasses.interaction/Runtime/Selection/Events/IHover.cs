using UnityEngine;

namespace ARGlasses.Interaction
{
    public interface IHover
    {
        Snapshot.Focus Focus { get; }
        Target Target => Focus.Target;
        Pose TargetPose => Focus.TargetPose;
        Pose TargetHit => Focus.TargetHit;
        HoverPhase Phase { get; }
        InteractionState State { get; }
        InputContext Context { get; }
        Pose Head => State.Head;
        Pose Eyes => State.Eyes;
        Pose Pinch => State.Pinch;
        Pose Wrist => State.Wrist;
        DPad DPad => State.DPad;
        DiscreteHandMotion DiscreteHandMotion => State.DiscreteHandMotion;
        Vector2 ImuReferenceDelta => State.ImuProjectedDeltaRadians;
    }
}
