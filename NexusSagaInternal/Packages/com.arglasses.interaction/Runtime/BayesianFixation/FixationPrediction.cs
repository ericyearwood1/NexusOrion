using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction.BayesianFixation
{
    public class FixationPrediction : MonoBehaviour, IFixationPrediction
    {
        [SerializeField] private List<FixationSurface> _fixationSurfaces;
        [SerializeField, ReadOnly] private BayesianHoverSelect _bayesianHoverSelect;
        [SerializeField, ReadOnly] private FixationRectangle _currentPrediction;
        public FixationRectangle CurrentPrediction => _currentPrediction;

        private FixationCandidate _fixationCandidate = null;
        public FixationClassifierParams FixationParams;
        [SerializeField, ReadOnly] private List<Fixation> _recentFixationsCache = new();

        public const int MaxNumFixationStored = 50;
        [SerializeField, ReadOnly] private CircularBuffer<Fixation> _fixations;
        [SerializeField, ReadOnly] private CircularBuffer<IFixationPrediction.GazeHitInfo> _gazeTrace;

        public static int TraceSize => (int)Mathf.Ceil(1e-3f * BayesianHoverSelect.TraceDurationMillis * ExpectedFPS);
        private const float ExpectedFPS = 240.0f;

        private void Awake()
        {
            this.Ensure(ref _bayesianHoverSelect);
        }

        protected void OnEnable()
        {
            _gazeTrace = new CircularBuffer<IFixationPrediction.GazeHitInfo>(TraceSize);
            _fixations = new CircularBuffer<Fixation>(MaxNumFixationStored);
            _fixationCandidate = null;
        }

        protected void OnDisable()
        {
            _fixations.Clear();
            _fixations.Clear();
            _fixationCandidate = null;
        }

        public EyeMovementState CurrentEyeMovementState()
        {
            if (_fixationCandidate != null && _fixationCandidate.Duration >= FixationParams.MinDuration * 1e-3) return EyeMovementState.Fixation;
            return EyeMovementState.Unknown;
        }

        public Fixation? TryGetLastFixation()
        {
            if (_fixationCandidate == null && _fixations.Count == 0) return null;
            if (_fixationCandidate == null) return _fixations.Latest();
            if (_fixationCandidate.Duration >= FixationParams.MinDuration * 1e-3) return _fixationCandidate.ToFixation(true);
            return _fixations[^1];
        }

        public IReadOnlyList<Fixation> GetRecentFixations(double lastGazeHitTime, double duration)
        {
            double endTime = lastGazeHitTime;
            double startTime = endTime - duration;
            var fixationIndexStart = FixationIndexForStartTime(startTime);

            _recentFixationsCache.Clear();
            for (int i = fixationIndexStart; i < _fixations.Count; i++) _recentFixationsCache.Add(_fixations[i]);

            if (_fixationCandidate == null) return _recentFixationsCache;
            if (_fixationCandidate.Duration >= FixationParams.MinDuration * 1e-3) _recentFixationsCache.Add(_fixationCandidate.ToFixation(true));
            return _recentFixationsCache;
        }

        private int FixationIndexForStartTime(double startTime)
        {
            int startIdx = _fixations.Count - 1;
            while (startIdx >= 0 && _fixations[startIdx].EndSampleTime >= startTime) startIdx--;
            startIdx = startIdx < 0 ? 0 : startIdx;
            return startIdx;
        }

        public void AddGazeHit(IFixationPrediction.GazeHitInfo gazeHit)
        {
            _gazeTrace.Add(gazeHit);
            UpdateFixationCandidate(gazeHit);
            UpdateCurrentPrediction(gazeHit);
        }

        private void UpdateFixationCandidate(IFixationPrediction.GazeHitInfo gazeHit)
        {
            float maxDispersion = 2.0f * gazeHit.EyeHitSurfaceDistance * Mathf.Tan(0.5f * (Mathf.Deg2Rad * FixationParams.MaxDispersion));
            if (_fixationCandidate == null)
            {
                _fixationCandidate = new FixationCandidate(gazeHit.Time, gazeHit.HitSurface, gazeHit.HitPosition, maxDispersion);
                return;
            }

            if (_fixationCandidate.TryAddSample(gazeHit.Time, gazeHit.HitPosition)) return;
            if (_fixationCandidate.Duration >= FixationParams.MinDuration * 1e-3)
            {
                _fixations.Add(_fixationCandidate.ToFixation(true));
                _fixationCandidate = new FixationCandidate(gazeHit.Time, gazeHit.HitSurface, gazeHit.HitPosition, maxDispersion);
            }
            else
            {
                _fixationCandidate = new FixationCandidate(_fixationCandidate, gazeHit.Time, gazeHit.HitPosition);
            }
        }

        private void UpdateCurrentPrediction(IFixationPrediction.GazeHitInfo gazeHit)
        {
            var lastGazeHitTime = gazeHit.Time;
            IReadOnlyList<Fixation> fixations = GetRecentFixations(lastGazeHitTime, BayesianHoverSelect.TraceDurationSeconds);

            int k = _gazeTrace.Count - 1;
            while (k >= 0 && float.IsPositiveInfinity(_gazeTrace[k].EyeHitSurfaceDistance)) k--;

            double headDistance = k >= 0 ? _gazeTrace[k].EyeHitSurfaceDistance : 1.0;
            Vector3 headPosition = k >= 0 ? _gazeTrace[k].HeadPosition : Vector3.forward * 1;

            _currentPrediction = _bayesianHoverSelect.Predict(fixations, _fixationSurfaces, headDistance, lastGazeHitTime, headPosition);
        }
    }
}
