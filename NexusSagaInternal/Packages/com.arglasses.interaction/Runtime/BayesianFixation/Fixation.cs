using MathNet.Numerics.LinearAlgebra;
using System;

namespace ARGlasses.Interaction.BayesianFixation
{
    // originally @rahular Rahul Arora (IX)
    // Workplace Post: https://fb.workplace.com/notes/604111865167044
    // Google Doc: Bayesian Technique for Hover-Select using Gaze Fixations, https://docs.google.com/document/d/1reeICN07BNNxGrKmbST4T6gUCLeB2x2TUARuVONzE4c/edit?usp=sharing
    // Source Repo: https://ghe.oculus-rep.com/input-exploration/ImmersiveAR/tree/rahul/lodlab3-bayesian
    // adapted for ARGlasses.Interaction by @naschenbach Nathan Aschenbach

    public enum EyeMovementState
    {
        Fixation,
        Saccade,
        SmoothPursuit,
        VOR,
        Unknown
    }

    [Serializable]
    public class FixationClassifierParams
    {
        // Max. dispersion for a fixation (in °)
        public float MaxDispersion = 2.0f;

        // Min. duration for a fixation (in ms)
        public float MinDuration = 50.0f;
        // TODO: Implement median filtering for gaze and feed it here
        //public bool Filter = true;
        // Window size for median filter (in ms)
        //public float MedianFilterWindowDuration = 21.0f;
    }

    public struct Fixation
    {
        // time
        public double StartSampleTime;
        public double EndSampleTime;

        // position
        public FixationSurface Surface;
        public Matrix<double> Samples;

        // statistics
        public Vector<double> Mean;
        public Matrix<double> Covariance;

        public readonly override string ToString()
        {
            return $"Canvas: {Surface.gameObject.name}, μ: {Mean}, Time: [{StartSampleTime:F3}, {EndSampleTime:F3}] s";
        }
    }
}
