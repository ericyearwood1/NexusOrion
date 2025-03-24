// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

namespace ARGlasses.Interaction
{
    public class ScrollController : MonoBehaviour, IScrollEventHandler, Inertia.IBoundaries
    {
        public static Vector3 ScrollToInertialSpace(float v) => new Vector3(v, 0, 0);
        public static float InertialSpaceToScroll(Vector3 v) => v.x;

        private float _scrollSensitivityFactor = 1;
        public float ScrollSensitivityFactor
        {
            get => _scrollSensitivityFactor;
            set => _scrollSensitivityFactor = value;
        }
        
        [SerializeField] private ScrollModel _model;

        private void Awake()
        {
            this.Descendant(ref _model);
            Assert.IsNotNull(_model);
        }

        public float _paginationSpeed = 20f;
        public float _paginationSpeedMax = 1f;
        public bool _reverseRelativePagination;

        public ScrollModel Model => _model;
        public float Max => _model.Max;
        public bool IsHorizontal => _model.IsHorizontal;
        public event Action<bool> OnGrabChanged = delegate {  };

        public float Value
        {
            get => _model.Value;
            set => _model.Value = value;
        }

        public float ValueNormalized
        {
            get => _model.ValueNormalized;
            set => _model.ValueNormalized = value;
        }

        public bool HasOvershoot() => Overshoot.magnitude > Inertia.MinInteriaMagnitude; // we always expect to have x value
        public Vector3 Overshoot => ScrollToInertialSpace(Value) - OvershootOrigin;
        public Vector3 OvershootOrigin => ScrollToInertialSpace(Mathf.Clamp(Value, 0, Max));

        [ReadOnly] public float _pendingContinuousScroll;
        [ReadOnly] public float _pendingPaginationScroll;
        [ReadOnly] public float _lastSnapToPageScroll;
        [ReadOnly] public float _scrollSpeed;
        public float ScrollSpeed => _scrollSpeed;

        [SerializeField] private bool _disableScrolling;
        public bool DisableScrolling
        {
            get => _disableScrolling;
            set => _disableScrolling = value;
        }
        [SerializeField] private int _currentLandmarkIndex;
        public int CurrentLandmarkIndex => _currentLandmarkIndex;
        public event Action<int> WhenLandmarkIndexChanged;
        public event Action<ScrollEvent> PreScrollApplied;
        public event Action<ScrollEvent> WhenScrollApplied;

        [SerializeField] private ScrollEvent _lastHandledScroll;

        private List<float> PaginationLandmarks => _model.PaginationLandmarks;

        [SerializeField] private float _lastScrollDistanceMeters = 0f;

        [SerializeField] private bool _forcePaginationPositions;

        // todo pull this out
        public bool ForcePaginationPositions
        {
            get => _forcePaginationPositions;
            set => _forcePaginationPositions = value;
        }

        [SerializeField] private float _forcePaginationPositionLerp = 8f;
        [SerializeField] private float _forcePaginationPositionThreshold = 0.002f;

        private int _lastLandmarkIndex;
        private bool _isGrabbed;
        public bool IsGrabbed => _isGrabbed;

        [SerializeField] private float _paginationRatioWhenNoLandmarks = 4f;

        private int LandmarkFromValue
        {
            get
            {
                int currentLandmark = 0;
                for (var i = 0; i < PaginationLandmarks.Count; i++)
                {
                    if (Value > PaginationLandmarks[i] - (_model.ContainerMax * 0.5f))
                    {
                        currentLandmark = i;
                    }
                }

                return currentLandmark;
            }
        }

        public void PaginateRelative(int delta)
        {
            if (_reverseRelativePagination) delta = -delta;
            if (delta == 0) return;

            _model.SetLockedToEnd(false);
            if (PaginationLandmarks.Count < 2)
            {
                float pagination = (float)delta * _model.ContainerMax / _paginationRatioWhenNoLandmarks;
                _pendingPaginationScroll += pagination;
            }
            else
            {
                int newLandmarkIndex = Mathf.Clamp(_currentLandmarkIndex + delta, 0, PaginationLandmarks.Count - 1);
                PaginateAbsolute(newLandmarkIndex);
            }
        }

        public void PaginateAbsolute(int landmark, bool forceImmidiate = false)
        {
            if (PaginationLandmarks.Count == 0)
            {
                Debug.LogError($"PaginateAbsolute failed, no landmarks available for ScrollController {transform.GetPath()}");
                return;
            }

            _model.SetLockedToEnd(false);
            landmark = Mathf.Clamp(landmark, 0, PaginationLandmarks.Count - 1);
            _pendingPaginationScroll = PaginationLandmarks[landmark] - Value;
        }

        private float _lastValue;

        // TODO Most of this should not be resolved in Update, move to an event driven strategy
        private void Update()
        {
            if(_disableScrolling)
                return;
            UpdateCurrentPaginationLandmark();
            UpdateSnapToPage();
            ApplyPendingScroll();
            UpdateWasScrolling();
            UpdateScrollSpeed();
        }

        private void UpdateScrollSpeed()
        {
            var newValue = Value;
            _scrollSpeed = Mathf.Abs((newValue - _lastValue) / Time.deltaTime);
            _lastValue = newValue;
        }

        private void UpdateCurrentPaginationLandmark(bool suppress = false)
        {
            _currentLandmarkIndex = LandmarkFromValue;
            if (_currentLandmarkIndex != _lastLandmarkIndex)
            {
                _lastLandmarkIndex = _currentLandmarkIndex;
                if (!suppress) WhenLandmarkIndexChanged?.Invoke(_currentLandmarkIndex);
            }
        }

        private void UpdateSnapToPage()
        {
            _lastSnapToPageScroll = default;

            if (_isGrabbed || !_forcePaginationPositions) return;
            if (Mathf.Abs(_pendingContinuousScroll + _pendingPaginationScroll) > _forcePaginationPositionThreshold) return;
            if (PaginationLandmarks.Count == 0) return;

            _lastSnapToPageScroll = PaginationLandmarks[_currentLandmarkIndex] - Value;
        }

        private void ApplyPendingScroll(bool instant = false)
        {
            if (!HasAnyPendingScroll) return;

            if (_isGrabbed) _pendingPaginationScroll = default;

            var paginationDelta = Time.deltaTime * _paginationSpeed * Mathf.Clamp(_pendingPaginationScroll, -_paginationSpeedMax, _paginationSpeedMax);
            var snapToPageDelta = Mathf.Lerp(0, _lastSnapToPageScroll, Time.deltaTime * _forcePaginationPositionLerp);

            if (instant)
            {
                paginationDelta = _pendingPaginationScroll;
                snapToPageDelta = _lastSnapToPageScroll;
            }

            Value += _pendingContinuousScroll + paginationDelta + snapToPageDelta;

            if (Mathf.Abs(paginationDelta) > Mathf.Abs(_pendingPaginationScroll)) paginationDelta = 0;
            _pendingPaginationScroll -= paginationDelta;
            _pendingContinuousScroll = 0;
        }

        public void ApplyPendingInstantly()
        {
            ApplyPendingScroll(instant: true);
            UpdateCurrentPaginationLandmark(suppress: true);
        }

        private float _startTime = 0f;
        private bool _lastIsScrolling = false;
        private float _timeThreshold = 0.1f;

        private void UpdateWasScrolling()
        {
            if (Math.Abs(_lastScrollDistanceMeters - Value) > 0.00001f)
            {
                if (!_lastIsScrolling)
                {
                    _lastIsScrolling = true;
                    _startTime = Time.time;
                }
                else
                {
                    if (!_isScrolling && Time.time - _startTime > _timeThreshold)
                    {
                        _isScrolling = true;
                        OnScrollingBegin?.Invoke();
                    }
                }
            }
            else
            {
                if (_lastIsScrolling)
                {
                    _lastIsScrolling = false;
                    _startTime = Time.time;
                }
                else
                {
                    if (_isScrolling && Time.time - _startTime > _timeThreshold)
                    {
                        _isScrolling = false;
                        OnScrollingEnd?.Invoke();
                    }
                }
            }

            _lastScrollDistanceMeters = Value;
        }

        public bool HasPendingPaginationScroll => Mathf.Abs(_pendingPaginationScroll) > Inertia.MinInteriaMagnitude;
        public bool HasPendingContinuousScroll => Mathf.Abs(_pendingContinuousScroll) > Inertia.MinInteriaMagnitude;
        public bool HasSnapToPageScroll => Mathf.Abs(_lastSnapToPageScroll) > Inertia.MinInteriaMagnitude;
        public bool HasAnyPendingScroll => HasPendingPaginationScroll || HasPendingContinuousScroll || HasSnapToPageScroll;

        public event Action OnScrollingBegin;
        public event Action OnScrollingEnd;
        private bool _isScrolling;
        public bool IsScrolling => _isScrolling;
        public void ScrollToEnd() => Value = Max;

        public void ClearPendingScroll()
        {
            _pendingContinuousScroll = default;
            _pendingPaginationScroll = default;
        }

        public void ApplyScroll(ScrollEvent scrollEvent)
        {
            PreScrollApplied?.Invoke(scrollEvent);
            var isGrabMoveEvent = scrollEvent.GrabPhase.IsUpdate() || scrollEvent.GrabPhase.IsEnded();

            if (!_isGrabbed && isGrabMoveEvent) return;
            if (!_isGrabbed && scrollEvent.GrabPhase.IsBegin()) UpdateGrabbed(true);
            if (_isGrabbed && scrollEvent.GrabPhase.IsEnded()) UpdateGrabbed(false);

            if (scrollEvent.Axis == ScrollAxis.Vertical && IsHorizontal) return;
            if (scrollEvent.Axis == ScrollAxis.Horizontal && !IsHorizontal) return;

            PaginateRelative(scrollEvent.Pagination);

            _pendingContinuousScroll += (_scrollSensitivityFactor * scrollEvent.DeltaMeters);
            if (scrollEvent.AbsoluteMeters != 0) _pendingContinuousScroll = scrollEvent.AbsoluteMeters - Value;

            scrollEvent.handledBy = this;
            scrollEvent.Handle(BubbleEventHandlePolicy.StopBubbling);

            _lastHandledScroll = scrollEvent;
            WhenScrollApplied?.Invoke(scrollEvent);
        }

        private void UpdateGrabbed(bool isGrabbed)
        {
            _isGrabbed = isGrabbed;
            _model.GrabChanged(isGrabbed);
            OnGrabChanged(isGrabbed);
        }
    }
}
