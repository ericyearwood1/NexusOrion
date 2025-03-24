using UnityEngine;

namespace ARGlasses.Interaction
{
    public interface IFixationPrediction
    {
        public class GazeHitInfo
        {
            public double Time;
            public Vector3 HitPosition;
            public float EyeHitSurfaceDistance;
            public FixationSurface HitSurface;
            public Vector3 HeadPosition;
        }

        public void AddGazeHit(GazeHitInfo gazeHitInfo);
    }
}
