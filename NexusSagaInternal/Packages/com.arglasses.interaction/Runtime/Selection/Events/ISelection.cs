using UnityEngine;

namespace ARGlasses.Interaction
{
    public interface ISelection
    {
        Snapshot.Focus Focus { get; }
        Target Target => Focus.Target;
        Pose TargetPose => Focus.TargetPose;
        Pose TargetHit => Focus.TargetHit;
        Side PrimarySide { get; }
        SelectionPhase Phase { get; }
        float Duration => Latest.TimeStamp - Begin.TimeStamp;
        InteractionState Latest { get; }
        InteractionState Begin { get; }
        InteractionState Previous { get; }
        IDrag Drag { get; }
        IDrag DragThisFrame { get; }
        InteractionStateSequence Sequence { get; }
        InputContext Context { get; }
    }
}
