using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction.Motion
{
    public class Easing
    {

        public static AnimationCurve Linear => BezierToCurve(0f, 0f, 1f, 1f);
        
        public static AnimationCurve InQuad => BezierToCurve(0.11f, 0, 0.5f, 0);
        public static AnimationCurve OutQuad => BezierToCurve(0.5f, 1f, 0.89f, 1f);
        public static AnimationCurve InOutQuad => BezierToCurve(0.45f, 0f, 0.55f, 1f);
        
        public static AnimationCurve InCubic => BezierToCurve(0.32f, 0f, 0.67f, 0f);
        public static AnimationCurve OutCubic => BezierToCurve(0.33f, 1f, 0.68f, 1f);
        public static AnimationCurve InOutCubic => BezierToCurve(0.65f, 0f, 0.35f, 1f);
        
        public static AnimationCurve InQuart => BezierToCurve(0.5f, 0f, 0.75f, 0f);
        public static AnimationCurve OutQuart => BezierToCurve(0.25f, 1f, 0.5f, 1f);
        public static AnimationCurve InOutQuart => BezierToCurve(0.76f, 0f, 0.24f, 1f);
        
        public static AnimationCurve InQuint => BezierToCurve(0.64f, 0f, 0.78f, 0f);
        public static AnimationCurve OutQuint => BezierToCurve(0.22f, 1f, 0.36f, 1f);
        public static AnimationCurve InOutQuint => BezierToCurve(0.83f, 0f, 0.17f, 1f);
        
        public static AnimationCurve InExpo => BezierToCurve(0.7f, 0f, 0.84f, 0f);
        public static AnimationCurve OutExpo => BezierToCurve(0.16f, 1f, 0.3f, 1f);
        public static AnimationCurve InOutExpo => BezierToCurve(0.87f, 0f, 0.13f, 1f);
        
        public static AnimationCurve InCirc => BezierToCurve(0.55f, 0f, 1f, 0.45f);
        public static AnimationCurve OutCirc => BezierToCurve(0f, 0.55f, 0.45f, 1f);
        public static AnimationCurve InOutCirc => BezierToCurve(0.85f, 0f, 0.15f, 1f);
        
        public static AnimationCurve InBack => BezierToCurve(0.36f, 0f, 0.66f, -0.56f);
        public static AnimationCurve OutBack => BezierToCurve(0.34f, 1.56f, 0.64f, 1f);
        public static AnimationCurve InOutBack => BezierToCurve(0.68f, -0.6f, 0.32f, 1.6f);

        public static AnimationCurve ColorHoverOn => BezierToCurve(0.4f, 0.04f, 0.5f, 1f); // 300ms Duration
        public static AnimationCurve ColorHoverOff => BezierToCurve(0.2f, 0.24f, 0.9f, 1f); // 400ms Duration
        public static AnimationCurve MotionHoverOn => BezierToCurve(0.6f, 0.0f, 0.4f, 1f); // Delay 150ms, 475ms Duration
        public static AnimationCurve MotionHoverOff => BezierToCurve(0.24f, 0.24f, 0.6f, 1f); // 625ms Duration
        public static AnimationCurve InteractPress => BezierToCurve(0.23f, 0.23f, 0.24f, 1f); // 80ms Duration
        public static AnimationCurve InteractRelease => BezierToCurve(0.25f, 0.08f, 0.4f, 1f); // 100ms Duration
        public static AnimationCurve InteractDeselect => BezierToCurve(0.05f, 0.16f, 0.45f, 1.05f); // 300ms Duration
        
        public static AnimationCurve BezierToCurve(float a, float b, float c, float d)
        {
            Vector2 start = Vector2.zero;
            Vector2 finish = Vector2.one;

            CalcCurveSlope(start, new Vector2(a,b), out float outTan0, out float outWeight0);
            CalcCurveSlope(new Vector2(c,d), finish, out float inTan1, out float inWeight1);

            var kf0 = new Keyframe(start.x, start.y, 0f, outTan0, 0f, outWeight0);
            var kf1 = new Keyframe(finish.x, finish.y, inTan1, 0, inWeight1, 0f);

            return new AnimationCurve(kf0, kf1);
        }

        static void CalcCurveSlope(Vector2 p0, Vector2 p1, out float tan, out float weight)
        {
            Vector2 d = p1 - p0;
            float dxSign = Mathf.Sign(d.x);
            float dxAbs = dxSign * d.x;
            if (Mathf.Approximately(dxAbs, 0f))
            {
                dxAbs += 0.000001f;
                d.x = dxSign * dxAbs;
            }
            tan = d.y / d.x;
            weight = dxAbs;
        }
        
    }
}
