using System;
using System.Collections;
using System.Collections.Generic;
using ARGlasses.Interaction.Motion;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEngine;

namespace ARGlasses.Motion
{
    /// <summary>
    /// A filter for adding gain to user motion based on input velocity.
    /// This allows for more precise movement at lower velocities
    /// and faster, coarse movement at higher velocities.
    /// </summary>
    public class MotionFilterSigmoidGain<T>
    {
        [Serializable]
        public class FilterParams
        {
            public float NoGainVelPoint = 0.1f;
            public float HighGateVel = 1f;
            public float MaxGain = 2f;
            public float GainSlope = 1f;

            public static FilterParams Default = new FilterParams()
            {
                NoGainVelPoint = 0.1f,
                HighGateVel = 1f,
                MaxGain = 2f,
                GainSlope = 1f
            };
        }
        
        private readonly FilterParams _params;
        private readonly MotionHistory<T> _inputHistory;
        private readonly MotionHistory<T> _outputHistory;
        
        private readonly Func<T, Vector4> _typeToVector;
        private readonly Func<Vector4, T> _vectorToType;
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="inputGetter">Reference to the absolute input value</param>
        /// <param name="outputGetter">Reference to the output values, to maintain a continuous stream</param>
        /// <param name="filterParams">Tuning parameters for the filter</param>
        public MotionFilterSigmoidGain (Func<T> inputGetter, Func<T> outputGetter, FilterParams filterParams = null)
        {
            _params = filterParams ?? FilterParams.Default;
            _inputHistory = new MotionHistory<T>(inputGetter);
            _outputHistory = new MotionHistory<T>(outputGetter);
            
            _typeToVector = MotionUtils.ConvertTypeToVector<T>(typeof(T));
            _vectorToType = MotionUtils.ConvertVectorToType<T>(typeof(T));
        }

        public T Step()
        {
            Vector4 velocityAsVector = _typeToVector(_inputHistory.LatestVelocity);
            float gain = CalculateGain(velocityAsVector, _params.HighGateVel,
                _params.NoGainVelPoint, _params.MaxGain, _params.GainSlope);
            Vector4 displacementAsVector = _typeToVector(_inputHistory.LatestDisplacement);
            Vector4 adjustedDisplacement = gain * displacementAsVector;
            Vector4 currentPositionAsVector = _typeToVector(_outputHistory.LatestPosition);
            return _vectorToType(currentPositionAsVector + adjustedDisplacement);
        }
        
        // Adapted from: IX Gain Transfer Function Comparison
        // https://docs.google.com/document/d/1JClWObr1_vkVujesxvpMY9NtFvgQ-CSfm6t4lhEWoc8/edit#heading=h.z5js5vasdsm4
        private float CalculateGain(Vector4 velocity, float highGateSpeed, float noGainSpeedPoint, float maxGain, float speedupSlope)
        {
            float portion = velocity.magnitude / highGateSpeed;
            float exponent = -1 * speedupSlope * (portion - noGainSpeedPoint);
            return maxGain * 1 / (1 + Mathf.Exp(exponent));
        }
    }
}
