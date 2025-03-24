// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public enum DragCancelState
    {
        In,
        Out
    }

    public static class ExtensionDragCancel
    {
        public static bool IsIn(this DragCancelState state) => state == DragCancelState.In;
        public static bool IsOut(this DragCancelState state) => state == DragCancelState.Out;
    }

    public class MechanicDragCancel : MonoBehaviour
    {
        public const float DefaultMaxDisplacement = 0.04f;
        public const float DefaultSnapBackSpeed = 20f;
        public const float DefaultBreakRatio = 4f;
        public const float DefaultNonRatioBreakThreshold = DefaultBreakRatio * DefaultMaxDisplacement;

        [SerializeField] private float _maxDisplacement = DefaultMaxDisplacement;
        [SerializeField] private float _snapBackSpeed = DefaultSnapBackSpeed;
        [SerializeField] private float _breakRatio = DefaultBreakRatio;
        [SerializeField] private bool _useRatioBasedBreakThreshold = true;
        [SerializeField] private float _nonRatioBreakThreshold = DefaultNonRatioBreakThreshold;
        public event Action<Vector3> WhenDisplacementLocalChanged = delegate { };
        public event Action<DragCancelState> WhenDragStateChanged = delegate { };

        [SerializeField, ReadOnly] private Vector2 _visualDimensions;
        [SerializeField, ReadOnly] private DragCancelState _state;
        public DragCancelState State => _state;

        [SerializeField, ReadOnly] private Vector3 _lastDisplacementLocal;
        public void SetMaxDisplacement(float v) => _maxDisplacement = v;
        public void SetSnapBackSpeed(float v) => _snapBackSpeed = v;
        public void SetUseRatioBasedBreakThreshold(bool v) => _useRatioBasedBreakThreshold = v;
        public void SetBreakRatio(float v) => _breakRatio = v;
        public void SetNonRatioBreakThreshold(float v) => _nonRatioBreakThreshold = v;

        [SerializeField] private MechanicDragMove _dragMoveMechanic;
        public MechanicDragMove DragMoveMechanic => _dragMoveMechanic;

        [SerializeField, ReadOnly] private SliderModel sliderModel;

        [SerializeField, ReadOnly] private Vector3 _lastDriftNormalized;

        private float? _minSliderDrift;
        private float? _maxSliderDrift;

        public bool IsGrabbing => _isGrabbing;
        [SerializeField, ReadOnly] private bool _isGrabbing;
        [SerializeField, ReadOnly] private bool _driftResolved;
        [SerializeField, ReadOnly] private float _lastToBreakpoint;
        [SerializeField, ReadOnly] private float _lastDisplacement;
        [SerializeField, ReadOnly] private Drag _lastDrag;

        private void Awake() => this.Ensure(ref _dragMoveMechanic);
        private void Start() => this.Sibling(ref sliderModel, optional: true);

        private void OnEnable() => _dragMoveMechanic.WhenDragMove += HandleDragMove;
        private void OnDisable() => _dragMoveMechanic.WhenDragMove -= HandleDragMove;

        private void HandleDragMove(Mechanic.DragMove.Event dragMove)
        {
            var drag = dragMove.Drag;
            _lastDrag = (Drag)drag;

            if (dragMove.Phase.IsDeadzone())
            {
                _isGrabbing = false;
                _minSliderDrift = null;
                _maxSliderDrift = null;
                _state = DragCancelState.In;
            }

            if (dragMove.Phase.IsBegin())
            {
                _isGrabbing = true;
                _minSliderDrift = null;
                _maxSliderDrift = null;
                _state = DragCancelState.In;
            }

            if (dragMove.Phase.IsEnded())
            {
                _isGrabbing = false;
                _minSliderDrift = null;
                _maxSliderDrift = null;
                // todo reset this AFTER ButtonModel resolves its EndPress
                // _state = DragCancelState.In;
                return; // Exit early after ending the drag
            }

            if (dragMove.Phase.IsUpdate())
            {
                if (sliderModel) SliderDrift(drag.DeltaLocal);
                else PressDrift(drag.DeltaLocal);
            }
        }

        private void SliderDrift(Vector3 dragTotalLocal)
        {
            var horizontal = sliderModel.IsHorizontal;
            var sliderDrift = horizontal ? dragTotalLocal.x : dragTotalLocal.y;

            if (sliderModel.ValueNormalized <= 0.01f && !_minSliderDrift.HasValue) _minSliderDrift = sliderDrift;
            if (sliderModel.ValueNormalized >= 0.99f && !_maxSliderDrift.HasValue) _maxSliderDrift = sliderDrift;

            var axis = horizontal ? Vector3.right : Vector3.up;
            if (sliderDrift <= _minSliderDrift) PressDrift(axis * (sliderDrift - _minSliderDrift.Value));
            else if (sliderDrift >= _maxSliderDrift) PressDrift(axis * (sliderDrift - _maxSliderDrift.Value));
            else
            {
                PressDrift(Vector3.zero);
                _minSliderDrift = null;
                _maxSliderDrift = null;
            }
        }

        private void PressDrift(Vector3 dragTotalLocal)
        {
            _driftResolved = false;
            var toBreakpoint = _useRatioBasedBreakThreshold
                    ? dragTotalLocal.magnitude / Mathf.Max(_maxDisplacement * _breakRatio, 0.001f)
                    : dragTotalLocal.magnitude / Mathf.Max(_nonRatioBreakThreshold, 0.001f);
            var driftThresholdExceeded = toBreakpoint > 0.5f;

            if (driftThresholdExceeded) toBreakpoint = 2.5f + toBreakpoint / (0.25f - toBreakpoint);
            else toBreakpoint = 8 * Mathf.Pow(toBreakpoint, 4f);

            var newState = driftThresholdExceeded ? DragCancelState.Out : DragCancelState.In;
            if (newState != _state)
            {
                _state = newState;
                if (enabled) WhenDragStateChanged(newState);
            }

            var radius = Mathf.Min(_visualDimensions.x, _visualDimensions.y) * 0.5f;
            var displacement = _maxDisplacement * radius;

            _lastToBreakpoint = toBreakpoint;
            _lastDisplacement = displacement;
            _lastDriftNormalized = dragTotalLocal.normalized;
            var displacementLocal = displacement * _lastDriftNormalized * toBreakpoint;
            UpdateDriftDisplacement(displacementLocal.WithZ(0));
        }

        private void Update()
        {
            // todo figure out where to do this properly..
            if (_dragMoveMechanic.Target.transform is RectTransform rectTransform) _visualDimensions = rectTransform.rect.size;
            else _visualDimensions = Vector2.one * 0.2f;

            if (!_isGrabbing && _lastDisplacementLocal.magnitude < 0.001f && !_driftResolved)
            {
                _driftResolved = true;
                UpdateDriftDisplacement(Vector3.zero);
            }

            if (_isGrabbing || _lastDisplacementLocal.magnitude < 0.001f) return;
            UpdateDriftDisplacement(Vector3.Lerp(_lastDisplacementLocal, Vector3.zero,
                _snapBackSpeed * Time.deltaTime));
        }

        private void UpdateDriftDisplacement(Vector3 displacementLocal)
        {
            if (enabled) WhenDisplacementLocalChanged(displacementLocal);
            _lastDisplacementLocal = displacementLocal;
        }
    }
}
