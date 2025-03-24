using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Statistics;
using UnityEngine;

namespace ARGlasses.Interaction.BayesianFixation
{
    public class FixationCandidate
    {
        public double StartSampleTime => TimedSamples[0].Item1;
        public double EndSampleTime => TimedSamples[^1].Item1;
        public double Duration => EndSampleTime - StartSampleTime;

        public readonly FixationSurface Surface;
        public readonly List<Tuple<double, Vector3>> TimedSamples;
        public List<Vector3> Samples => TimedSamples.Select(x => x.Item2).ToList();

        public readonly double MaxAllowedDispersion;

        private float _xMin, _xMax, _yMin, _yMax, _zMin, _zMax;

        public const float DefaultFixationStdDevDeg = 1.24f;

        public FixationCandidate(double time, FixationSurface surface, Vector3 initSample, double maxDispersion)
        {
            Surface = surface;
            TimedSamples = new List<Tuple<double, Vector3>> { new(time, initSample) };

            MaxAllowedDispersion = maxDispersion;

            _xMin = _xMax = initSample.x;
            _yMin = _yMax = initSample.y;
            _zMin = _zMax = initSample.z;
        }

        public FixationCandidate(
            FixationCandidate old,
            double newSampleTime,
            Vector3 newSample)
        {
            MaxAllowedDispersion = old.MaxAllowedDispersion;
            Surface = old.Surface;

            int oldTimedSamplesCount = old.TimedSamples.Count;
            _xMin = _xMax = newSample.x;
            _yMin = _yMax = newSample.y;
            _zMin = _zMax = newSample.z;

            TimedSamples = new() { new Tuple<double, Vector3>(newSampleTime, newSample) };

            for (int i = oldTimedSamplesCount - 1; i >= 0; --i)
            {
                var sample = old.TimedSamples[i].Item2;
                var time = old.TimedSamples[i].Item1;

                float xMin = Mathf.Min(_xMin, sample.x);
                float xMax = Mathf.Max(_xMax, sample.x);
                float yMin = Mathf.Min(_yMin, sample.y);
                float yMax = Mathf.Max(_yMax, sample.y);
                float zMin = Mathf.Min(_zMin, sample.z);
                float zMax = Mathf.Max(_zMax, sample.z);

                var dispersion = (xMax - xMin) + (yMax - yMin) + (zMax - zMin);
                if (dispersion <= MaxAllowedDispersion)
                {
                    _xMin = xMin;
                    _xMax = xMax;
                    _yMin = yMin;
                    _yMax = yMax;
                    _zMin = zMin;
                    _zMax = zMax;

                    TimedSamples.Add(new Tuple<double, Vector3>(time, sample));
                }
                else
                {
                    break;
                }
            }

            TimedSamples.Reverse();
        }

        public bool TryAddSample(double time, Vector3 sample)
        {
            float xMin = Mathf.Min(_xMin, sample.x);
            float xMax = Mathf.Max(_xMax, sample.x);
            float yMin = Mathf.Min(_yMin, sample.y);
            float yMax = Mathf.Max(_yMax, sample.y);
            float zMin = Mathf.Min(_zMin, sample.z);
            float zMax = Mathf.Max(_zMax, sample.z);

            var dispersion = (xMax - xMin) + (yMax - yMin) + (zMax - zMin);
            if (dispersion <= MaxAllowedDispersion)
            {
                _xMin = xMin;
                _xMax = xMax;
                _yMin = yMin;
                _yMax = yMax;
                _zMin = zMin;
                _zMax = zMax;

                TimedSamples.Add(new Tuple<double, Vector3>(time, sample));
                return true;
            }

            return false;
        }

        public Fixation ToFixation(bool forceFullRankCovariance)
        {
            Fixation fixation = new()
            {
                StartSampleTime = StartSampleTime,
                EndSampleTime = EndSampleTime,
                Surface = Surface,
            };

            // TODO if needed
            // Currently, all samples are weighed equally.
            // It may be useful to extract the timestamps and weigh each sample with:
            // w[i] = ( (t[i+1] - t[i-1])/2 ) / ( t[^1] - t[0] )
            // to get a better integral.

            fixation.Samples = Matrix<double>.Build.Dense(3, Samples.Count, (r, c) => (double)Samples[c][r]);
            fixation.Mean = fixation.Samples.RowSums() / fixation.Samples.ColumnCount;

            var covxx = Statistics.Covariance(fixation.Samples.Row(0), fixation.Samples.Row(0));
            var covyy = Statistics.Covariance(fixation.Samples.Row(1), fixation.Samples.Row(1));
            var covzz = Statistics.Covariance(fixation.Samples.Row(2), fixation.Samples.Row(2));
            var covxy = Statistics.Covariance(fixation.Samples.Row(0), fixation.Samples.Row(1));
            var covyz = Statistics.Covariance(fixation.Samples.Row(1), fixation.Samples.Row(2));
            var covzx = Statistics.Covariance(fixation.Samples.Row(2), fixation.Samples.Row(0));

            fixation.Covariance = Matrix<double>.Build.Dense(3, 3);
            fixation.Covariance[0, 0] = covxx;
            fixation.Covariance[1, 1] = covyy;
            fixation.Covariance[2, 2] = covzz;
            fixation.Covariance[0, 1] = fixation.Covariance[1, 0] = covxy;
            fixation.Covariance[1, 2] = fixation.Covariance[2, 1] = covyz;
            fixation.Covariance[2, 0] = fixation.Covariance[0, 2] = covzx;

            if (forceFullRankCovariance)
            {
                //var evd = fixation.Covariance.Evd();
                //var D = evd.D;
                //int minIdx = D[0, 0] < D[1, 1] ?
                //    (D[0, 0] < D[2, 2] ? 0 : 2) :
                //    (D[1, 1] < D[2, 2] ? 1 : 2);
                //int maxIdx = D[0, 0] >= D[1, 1] ?
                //    (D[0, 0] >= D[2, 2] ? 0 : 2) :
                //    (D[1, 1] >= D[2, 2] ? 1 : 2);
                //int midIdx = 3 - minIdx - maxIdx;
                //if (evd.Rank < 3)
                //{
                //    if (evd.Rank == 2)
                //    {
                //        D[minIdx, minIdx] = D[midIdx, midIdx];
                //    }
                //    else if (evd.Rank == 1)
                //    {
                //Debug.LogWarning($"[Fixation] Covariance matrix is rank 1         {fixation.Covariance}");
                //        D[minIdx, minIdx] = D[midIdx, midIdx] = D[maxIdx, maxIdx];
                //    }
                //    else
                //    {
                //Debug.LogWarning($"[Fixation] Covariance matrix is zero {fixation.Covariance}");
                //        D = Matrix<double>.Build.DenseIdentity(3) * Mathf.Pow(2.0f * Mathf.Tan(0.5f * DEFAULT_FIXATION_STD_DEV_DEG * Mathf.Deg2Rad), 2.0f);
                //    }

                //    var v = evd.EigenVectors;
                //    fixation.Covariance = v * D * v;
                //}

                try
                {
                    var svd = fixation.Covariance.Svd();
                    var W = svd.W;
                    int minIdx = W[0, 0] < W[1, 1] ? (W[0, 0] < W[2, 2] ? 0 : 2) : (W[1, 1] < W[2, 2] ? 1 : 2);
                    int maxIdx = W[0, 0] >= W[1, 1] ? (W[0, 0] >= W[2, 2] ? 0 : 2) : (W[1, 1] >= W[2, 2] ? 1 : 2);
                    int midIdx = 3 - minIdx - maxIdx;
                    if (svd.Rank < 3)
                    {
                        if (svd.Rank == 2)
                        {
                            W[minIdx, minIdx] = W[midIdx, midIdx];
                        }
                        else if (svd.Rank == 1)
                        {
                            Debug.LogWarning($"[Fixation] Covariance matrix is rank 1 {fixation.Covariance}");
                            W[minIdx, minIdx] = W[midIdx, midIdx] = W[maxIdx, maxIdx];
                        }
                        else
                        {
                            Debug.LogWarning($"[Fixation] Covariance matrix is zero {fixation.Covariance}");
                            W = Matrix<double>.Build.DenseIdentity(3) * Mathf.Pow(2.0f * Mathf.Tan(0.5f * DefaultFixationStdDevDeg * Mathf.Deg2Rad), 2.0f);
                        }

                        fixation.Covariance = svd.U * W * svd.VT;
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[Fixation] SVD convergence failed for {fixation}, Σ: {fixation.Covariance}, {{g}} = {fixation.Samples}");
                    Debug.LogException(ex);
                }
            }

            return fixation;
        }
    }
}
