using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction.Motion
{

    [System.Serializable]
    public abstract class AMotionParams
    {
        public static MotionParams Default => new MotionParams(1f, 0.5f, 1f);

        public static MotionParamsEasing ColorHoverOn => new MotionParamsEasing(Easing.BezierToCurve(0.4f, 0.04f, 0.5f, 1.0f), 0.300f);
        public static MotionParamsEasing ColorHoverOff => new MotionParamsEasing(Easing.BezierToCurve(0.2f, 0.24f, 0.9f, 1.0f), 0.400f);
        public static MotionParamsEasing MotionHoverOn => new MotionParamsEasing(Easing.BezierToCurve(0.6f, 0f, 0.4f, 1.0f), 0.475f, 0.150f);
        public static MotionParamsEasing MotionHoverOff => new MotionParamsEasing(Easing.BezierToCurve(0.24f, 0.24f, 0.6f, 1.0f), 0.625f);
        public static MotionParamsEasing InteractPress => new MotionParamsEasing(Easing.BezierToCurve(0.23f, 0.23f, 0.24f, 1.0f), 0.080f);
        public static MotionParamsEasing InteractRelease => new MotionParamsEasing(Easing.BezierToCurve(0.25f, 0.08f, 0.4f, 1.0f), 0.100f);
        public static MotionParamsEasing InteractDeselect => new MotionParamsEasing(Easing.BezierToCurve(0.05f, 0.16f, 0.45f, 1.05f), 0.300f);
    }

    /// <summary>
    /// Stores parameters for frequency, damping, and response.
    /// For inspiration on how these can be used, recommended viewing:
    /// Giving Personality to Procedural Animations: https://www.youtube.com/watch?v=KPoeNZZ6H4s
    /// Designing Fluid Interfaces: https://developer.apple.com/videos/play/wwdc2018/803/
    /// </summary>
    [System.Serializable]
    public class MotionParams : AMotionParams
    {
        
        [Range(0.1f, 3f)]
        public float _frequency = 1f;

        [Range(0.001f, 3f)]
        public float _damping = 0.5f;

        [Range(-2f, 2f)]
        public float _response = 1f;

        [NonSerialized]
        public float _k1, _k2, _k3;

        public MotionParams() { }

        public MotionParams(float frequency, float damping, float response)
        {
            _frequency = frequency;
            _damping = damping;
            _response = response;
        }

        public MotionParams(MotionParams reference)
        {
            _frequency = reference._frequency;
            _damping = reference._damping;
            _response = reference._response;
        }
        
        public MotionParams Copy()
        {
            return (MotionParams) this.MemberwiseClone();
        }
        
        // Src: 
        public void UpdateConstants()
        {
            _k1 = _damping / (Mathf.PI * _frequency);
            _k2 = 1f / ((2 * Mathf.PI * _frequency) * (2 * Mathf.PI * _frequency));
            _k3 = _response * _damping / (2f * Mathf.PI * _damping);
        }
    }
    
    /// <summary>
    /// When passed to the motion system, allows for 'traditional'
    /// time-based motion using duration and an easing curve.
    /// </summary>
    [System.Serializable]
    public class MotionParamsEasing : AMotionParams
    {
        public AnimationCurve _easing;
        
        [Range(0.01f, 3f)]
        public float _duration;

        public float _delay;

        public MotionParamsEasing(AnimationCurve easing, float duration)
        {
            _easing = easing;
            _duration = duration;
            _delay = 0.0f;
        }

        public MotionParamsEasing(AnimationCurve easing, float duration, float delay)
        {
            _easing = easing;
            _duration = duration;
            _delay = delay;
        }

        public MotionParamsEasing(MotionParamsEasing reference)
        {
            _easing = reference._easing;
            _duration = reference._duration;
            _delay = reference._delay;
        }
        
        public MotionParamsEasing Copy()
        {
            return (MotionParamsEasing) this.MemberwiseClone();
        }
    }

}
