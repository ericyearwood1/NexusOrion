using System;
using OSIG.Tools.Layout;
using OSIG.Tools.Units;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public interface ICursorEffect
    {
        public float NudgeClamp { get; }
        float GetRadiusPixels();
        void SetBackgroundOffset(Vector3 localOffset);
    }
}
