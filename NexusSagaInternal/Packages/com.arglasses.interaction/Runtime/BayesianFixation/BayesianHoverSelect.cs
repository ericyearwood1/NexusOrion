using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Distributions;

namespace ARGlasses.Interaction.BayesianFixation
{
    // originally @rahular Rahul Arora (IX)
    // Workplace Post: https://fb.workplace.com/notes/604111865167044
    // Google Doc: Bayesian Technique for Hover-Select using Gaze Fixations, https://docs.google.com/document/d/1reeICN07BNNxGrKmbST4T6gUCLeB2x2TUARuVONzE4c/edit?usp=sharing
    // Source Repo: https://ghe.oculus-rep.com/input-exploration/ImmersiveAR/tree/rahul/lodlab3-bayesian
    // adapted for ARGlasses.Interaction by @naschenbach Nathan Aschenbach

    public enum FixationWeightType
    {
        EndTime = 0,
        MidTime = 1,
        Integrated = 2
    }

    [Serializable]
    public class BayesianPredictorParams
    {
        // Gaze σ at the center of FOV (in °)
        public float StandardDeviationAtCenter = 1.24f;

        // Mistime between targeting and selection (in ms, +ve for leave-before-click behaviours)
        public float ExpectedGazeDownMistime = 0.0f;
        public float WeightFunctionConst = 7.0f;
        public FixationWeightType WeightType = FixationWeightType.MidTime;
        public float ProbabilityRandomMultiplierThreshold = 2.0f;
        public float ProbabilityRatioThreshold = 1.1f;
        public float DistanceLimit = 2.0f;
    }

    public class BayesianHoverSelect : MonoBehaviour
    {
        [SerializeField] private bool _verboseOutput = false;

        public bool VerboseOutput
        {
            get => _verboseOutput;
            set => _verboseOutput = value;
        }

        public const float TraceDurationMillis = 800.0f; // milliseconds
        public const double TraceDurationSeconds = TraceDurationMillis * 1e-3;

        public BayesianPredictorParams PredictorParams;

        public FixationRectangle Predict(
            IReadOnlyList<Fixation> fixations,
            List<FixationSurface> fixationSurfaces,
            double headDistance,
            double selectTime,
            Vector3 hmdPosition)
        {
            List<FixationRectangle> allTargets = fixationSurfaces.SelectMany(cv => cv.Rectangles()).ToList();

            if (fixations.Count == 0 || allTargets.Count == 0) return null;

            Vector<double> fixationWeights = GetFixationWeights(fixations, selectTime);

            if (CheckGazeTraceTooFarFromTargets(allTargets, fixations, hmdPosition, fixationWeights)) return null;

            // For each fixation, compute a probability distribution over all relevant targets.
            Matrix<double> perFixationDistribution = GetConditionalProbabilities(allTargets, fixationSurfaces, fixations, headDistance);

            // Step 5: Compute overall probability distribution over all relevant targets.
            Vector<double> distribution = perFixationDistribution * fixationWeights;

            // Step 6: Check thresholding criteria to find if a target is to be selected or if the selection event is to be treated as a false positive.
            // If it's the former, select the most likely target.
            var sortedDistribution = distribution.OrderByDescending(x => x).ToArray();
            double maxProbability = sortedDistribution[0];

            // probability should be at least some multiple k of the probability of random assignment.
            // Note that probability of random assignment is (1 / #Targets).
            double probabilityThreshold = PredictorParams.ProbabilityRandomMultiplierThreshold / allTargets.Count;
            if (maxProbability < probabilityThreshold) return null;

            if (allTargets.Count == 1) return allTargets[distribution.MaximumIndex()];

            double secondMaxProbability = sortedDistribution[1];
            var probabilityRatioTooLow = maxProbability / secondMaxProbability < PredictorParams.ProbabilityRatioThreshold;
            if (probabilityRatioTooLow) return null;

            return allTargets[distribution.MaximumIndex()];
        }

        private bool CheckGazeTraceTooFarFromTargets(List<FixationRectangle> allTargets, IReadOnlyList<Fixation> fixations, Vector3 hmdPosition, Vector<double> fixationWeights)
        {
            // We use the weighted mean of the fixation mean positions as the representative sample of the gaze trace.
            Matrix<double> means = Matrix<double>.Build.DenseOfColumns(fixations.Select(f => f.Mean));
            Vector<double> weightedMean = means * fixationWeights;
            Vector3 weightedMeanPosition = new Vector3(
                (float)weightedMean[0],
                (float)weightedMean[1],
                (float)weightedMean[2]);
            float minDist = allTargets.Select(tgt => tgt.Distance(weightedMeanPosition, false)).Min();
            float distanceLimit = Mathf.Tan(Mathf.Deg2Rad * PredictorParams.DistanceLimit) * (weightedMeanPosition - hmdPosition).magnitude;
            return minDist > distanceLimit;
        }

        private Matrix<double> GetConditionalProbabilities(
            List<FixationRectangle> allTargets,
            List<FixationSurface> fixationSurfaces,
            IReadOnlyList<Fixation> fixations,
            double headDistance)
        {
            int allTargetsCount = allTargets.Count;
            int fixationsCount = fixations.Count;
            var allFixationIndices = Enumerable.Range(0, fixationsCount).ToList();

            // conditional probabilities of target given a fixation: P(Tⱼ | Fᵢ)
            Matrix<double> probTargetGivenFixation = Matrix<double>.Build.Dense(allTargetsCount, fixationsCount);
            // conditional probabilities of fixation given a target: P(Fᵢ | Tⱼ)
            Matrix<double> probFixationGivenTarget = Matrix<double>.Build.Dense(allTargetsCount, fixationsCount);

            // A panel has targets `allTargets[nstart .. nend]`
            int targetIndexStart = 0;
            int targetIndexEnd;

            // Our assumption is that each fixation only impacts the targets on its own panel
            // That is, panel(Fᵢ) ≠ panel(Tⱼ) ⇒ P(Tⱼ | Fᵢ) = P(Fᵢ | Tⱼ) = 0
            foreach (var fixationSurface in fixationSurfaces)
            {
                var distanceQueryRectanglesCount = fixationSurface.Rectangles().Count;
                targetIndexEnd = targetIndexStart + distanceQueryRectanglesCount;
                var panelFixationIndices = allFixationIndices.Where(i => fixations[i].Surface == fixationSurface).ToList();
                if (!panelFixationIndices.Any()) continue;

                double gazeStdDev = headDistance * Math.Tan(Math.PI / 180.0 * PredictorParams.StandardDeviationAtCenter);
                Matrix<double> dummyColCov = Matrix<double>.Build.DenseIdentity(1);

                for (int targetIndex = targetIndexStart; targetIndex < targetIndexEnd; targetIndex++)
                {
                    var target = allTargets[targetIndex];
                    var targetPos = target.transform.position;
                    double[,] muArray =
                    {
                        {
                            targetPos.x,
                            targetPos.y,
                            targetPos.z,
                        }
                    };
                    // MathNET doesn't provide a Vector-valued multivariate normal distribution, so using a Matrix-valued distribution instead.
                    // μ is a column matrix (3 × 1)
                    Matrix<double> mu = Matrix<double>.Build.DenseOfArray(muArray).Transpose();
                    // Σ is a 3 × 3 matrix
                    Matrix<double> cov = Matrix<double>.Build.DenseIdentity(3, 3) * (gazeStdDev * gazeStdDev);
                    MatrixNormal targetNormal = new MatrixNormal(mu, cov, dummyColCov);

                    var importance = 1f;
                    foreach (var fixationIndex in panelFixationIndices)
                    {
                        probFixationGivenTarget[targetIndex, fixationIndex] = targetNormal.Density(fixations[fixationIndex].Mean.ToColumnMatrix());
                        probTargetGivenFixation[targetIndex, fixationIndex] = probFixationGivenTarget[targetIndex, fixationIndex] * importance;
                    }
                }
                targetIndexStart += distanceQueryRectanglesCount;
            }

            // Currently, `probTgtGivenFix[i, j]` contains I(Tᵢ) P(Fⱼ | Tᵢ), where I() is the importance/strength of a target, equivalent to an unnormalized notion of probability.
            // We have to normalize by ∑ᵢ I(Tᵢ) P(Fⱼ | Tᵢ) ≡ sum of ith column of `probTgtGivenFix` to get probability distribution P(Tᵢ | Fⱼ).
            var colSums = probTargetGivenFixation.ColumnSums();
            for (int fixationIndex = 0; fixationIndex < fixationsCount; ++fixationIndex)
            {
                for (int targetIndex = 0; targetIndex < allTargetsCount; ++targetIndex)
                {
                    probTargetGivenFixation[targetIndex, fixationIndex] /= colSums[fixationIndex];
                }

            }
            return probTargetGivenFixation;
        }

        private Vector<double> GetFixationWeights(IReadOnlyList<Fixation> fixations, double selectTime)
        {
            int fixationsCount = fixations.Count;
            Vector<double> weights = Vector<double>.Build.Dense(fixationsCount);

            double T0 = selectTime - 1e-3 * PredictorParams.ExpectedGazeDownMistime;
            double weightFunctionConst = PredictorParams.WeightFunctionConst;
            double totalTraceTime = 1e-3 * TraceDurationMillis;

            for (int i = 0; i < fixationsCount; ++i)
            {
                var tStart = fixations[i].StartSampleTime;
                var tEnd = fixations[i].EndSampleTime;
                var tMid = 0.5 * (tStart + tEnd);

                double weight = default;
                if (PredictorParams.WeightType == FixationWeightType.MidTime) weight = Math.Exp(-weightFunctionConst * Math.Abs(T0 - tMid) / totalTraceTime);
                else if (PredictorParams.WeightType == FixationWeightType.EndTime) weight = Math.Exp(-weightFunctionConst * Math.Abs(T0 - tEnd) / totalTraceTime);
                else if (PredictorParams.WeightType == FixationWeightType.Integrated)
                {
                    var start = Math.Exp(-weightFunctionConst * tStart / totalTraceTime);
                    var end = Math.Exp(-weightFunctionConst * tEnd / totalTraceTime);
                    if (tEnd <= T0) weight = (end - start) / weightFunctionConst;
                    else if (tStart >= T0) weight = (start - end) / weightFunctionConst;
                    else weight = (1.0 - start) / weightFunctionConst + (1.0 - end) / weightFunctionConst;
                }

                weights[i] = weight;
            }

            return weights / weights.Sum();
        }
    }
}
