using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ARGlasses.Interaction
{
    /// <summary>
    /// Used by Conecaster to select the Best candidate from a list of ConecastResult
    /// </summary>
    public class ConecastPrioritizer : MonoBehaviour
    {
        public const float CompareByDistanceThreshold = 0.01f;

        [SerializeField] private bool _drawConecastDebug = true;
        [SerializeField] private float _hoverLeaveThresholdDegrees = 0.5f; // The maximum buffer in degrees

        [SerializeField, ReadOnly] private float _hoverDistance;
        [SerializeField, ReadOnly] private List<Target> _orderedTarget = new();
        [SerializeField, ReadOnly] private ConecastResult _lastResult;
        public ConecastResult LastResult => _lastResult;

        public float HoverDistance
        {
            get => _hoverDistance;
            set => _hoverDistance = value;
        }

        [SerializeField] private int _closestIndex;
        public int ClosestIndex => _closestIndex;

        [SerializeField, ReadOnly] private List<ConecastComparison> _comparisons = new();

        [Serializable]
        private struct ConecastComparison
        {
            [SerializeField, ReadOnly] private Target _prioritized;
            [SerializeField, ReadOnly] private Target _deprioritized;

            [Header("Comparison Sequence")]
            [SerializeField, ReadOnly] private int _sortOrderCompare;
            [SerializeField, ReadOnly] private int _hitPointDistanceCompare;
            [SerializeField, ReadOnly] private int _angleCompare;
            [SerializeField, ReadOnly] private int _hierarchyCompare;

            [Header("Comparison Details")]
            [SerializeField, ReadOnly, Tooltip("Neg: A is first, Pos: B is first")] private int _compare;
            [SerializeField, ReadOnly] private float _hitPointDistance;
            [SerializeField, ReadOnly] private bool _hitPointDistanceExceeded;
            [SerializeField, ReadOnly] private ConecastResult _a;
            [SerializeField, ReadOnly] private ConecastResult _b;

            // -1: first in sorted list, high priority
            public int Compare => _compare;

            public ConecastComparison(ConecastResult a, ConecastResult b)
            {
                _a = a;
                _b = b;

                _hitPointDistance = Mathf.Abs(_a.hitPointDistance - _b.hitPointDistance);
                _hitPointDistanceExceeded = _hitPointDistance > CompareByDistanceThreshold;
                _hitPointDistanceCompare = _hitPointDistanceExceeded ? _a.hitPointDistance.CompareTo(_b.hitPointDistance) : 0;

                var negativeSortOrderIsLastInConecastList = -1;
                _sortOrderCompare = _a.sortOrder.CompareTo(_b.sortOrder) * negativeSortOrderIsLastInConecastList;
                _angleCompare = _a.gazeToColliderAngleDegrees.CompareTo(_b.gazeToColliderAngleDegrees); // threshold?

                var lastInspectorOrderIsFirstInConecastList = -1;
                _hierarchyCompare = Hierarchy.CompareInspectorOrder(_a.target.transform, _b.target.transform) * lastInspectorOrderIsFirstInConecastList; // pick the last

                _compare = 0;
                if (_sortOrderCompare != 0) _compare = _sortOrderCompare;
                else if (_hitPointDistanceCompare != 0) _compare = _hitPointDistanceCompare;
                else if (_angleCompare != 0) _compare = _angleCompare;
                else if (_hierarchyCompare != 0) _compare = _hierarchyCompare;

                // -1 : a prioritized/first
                // 1 : b prioritized/first
                _prioritized = _compare > 0 ? _b.target : _compare < 0 ? _a.target : null;
                _deprioritized = _compare > 0 ? _a.target : _compare < 0 ? _b.target : null;
            }
        }

        int OrderConecastResult(ConecastResult a, ConecastResult b)
        {
            var comparison = new ConecastComparison(a,b);
            _comparisons.Add(comparison);
            return comparison.Compare;
        }

        /// <summary>
        ///  Once the list of ConecastResult is sorted, select the best one based upon HoverDistance
        /// </summary>
        private int FindBestIndexForDistance(List<ConecastResult> results)
        {
            var closestIndex = 0;
            var closestDistance = Mathf.Infinity;
            for (var i = 0; i < results.Count; i++)
            {
                var dist = Mathf.Abs(results[i].hitPointDistance - HoverDistance);
                if (dist > closestDistance) continue;
                closestIndex = i;
                closestDistance = dist;
            }

            return closestIndex;
        }

        public ConecastResult FindBestResultInfo(List<ConecastResult> results, Cone cone)
        {
            var bestHit = ConecastResult.Empty(cone);
            if (results.Count == 0)
            {
                _lastResult = bestHit;
                return bestHit;
            }

            _comparisons.Clear();
            results.Sort(OrderConecastResult);
            bestHit = results[0];

            // if (_lastResult.HasTarget)
            // {
            //     var stickyResult = results.FirstOrDefault(r => r.target == _lastResult.target);
            //     if (stickyResult.HasTarget && stickyResult.gazeToColliderAngleDegrees < _hoverLeaveThresholdDegrees) bestHit = stickyResult;
            // }

            _lastResult = bestHit;
            _orderedTarget.Clear();
            _orderedTarget.AddRange(results.Select(r => r.target));

            return bestHit;
        }

    }
}
