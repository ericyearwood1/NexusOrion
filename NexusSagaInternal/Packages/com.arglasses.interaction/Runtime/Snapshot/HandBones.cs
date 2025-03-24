using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public static partial class Snapshot
    {
        [Serializable]
        public struct HandBones
        {
            public Pose Wrist;
            public Pose Forearm;

            public Pose IndexProximal;
            public Pose IndexDistal;
            public Pose IndexTip;

            public Pose ThumbProximal;
            public Pose ThumbDistal;
            public Pose ThumbTip;

            public float Scale;

            const float ThumbToIndexPointerPositionBias = 0.3f;
            public Pose CalculatePinchPose()
            {
                var distal = Vector3.Lerp(ThumbDistal.position, IndexDistal.position, ThumbToIndexPointerPositionBias);
                var proximal = Vector3.Lerp(ThumbProximal.position, IndexProximal.position, ThumbToIndexPointerPositionBias);
                var up = (IndexProximal.position - ThumbProximal.position).normalized;
                var toDistal = distal - proximal;
                if (toDistal == default) toDistal = Vector3.up;
                return new Pose(distal, Quaternion.LookRotation(toDistal, up));
            }

            public static HandBones Default(Side side, Pose wrist)
            {
                var thumbX = side.IsLeft() ? 0.01f : -0.01f;
                return new HandBones
                {
                    Wrist = wrist,
                    Forearm = wrist.AddLocalPosition(new Vector3(0, 0, -0.02f)),

                    IndexProximal = wrist.AddLocalPosition(new Vector3(0, 0.01f, 0.01f)),
                    IndexDistal = wrist.AddLocalPosition(new Vector3(0, 0.01f, 0.03f)),
                    IndexTip = wrist.AddLocalPosition(new Vector3(0, 0.01f, 0.04f)),

                    ThumbProximal = wrist.AddLocalPosition(new Vector3(thumbX, 0.0f, 0.01f)),
                    ThumbDistal = wrist.AddLocalPosition(new Vector3(thumbX, 0.0f, 0.02f)),
                    ThumbTip = wrist.AddLocalPosition(new Vector3(thumbX, 0.0f, 0.03f)),

                    Scale = 1,
                };
            }
        }

    }
}
