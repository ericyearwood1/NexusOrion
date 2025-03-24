using UnityEngine;

namespace ARGlasses.Interaction
{
    public static class ExtensionsOVR
    {
        public static Pose ToWorldSpacePose(this OVRPlugin.EyeGazeInteractionState state)
        {
            // todo is this the best way to get WorldSpace from EyeGazeState?
            OVRPose ovrPose = state.Pose.ToOVRPose().ToWorldSpacePose();
            return new Pose(ovrPose.position, ovrPose.orientation);
        }

        public static Pose ToWorldSpacePose(this OVRPlugin.EyeGazeState state)
        {
            // todo is this the best way to get WorldSpace from EyeGazeState?
            OVRPose ovrPose = state.Pose.ToOVRPose().ToWorldSpacePose();
            return new Pose(ovrPose.position, ovrPose.orientation);
        }
    }
}
