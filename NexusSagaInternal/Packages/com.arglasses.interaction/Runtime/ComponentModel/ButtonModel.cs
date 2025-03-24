// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public interface IButton
    {
        event Action WhenClicked;
        event Action WhenCancelled;
        event Action<bool> WhenValueChanged;
        event Action<TargetState> WhenStateChanged;
        event Action<Vector3> WhenDisplacementLocalChanged;
        bool Value { get; }
    }

    public class ButtonModel : MonoBehaviour, IButton
    {
        public event Action WhenClicked = delegate { };
        public event Action WhenCancelled = delegate { };
        public event Action WhenSuccess = delegate { };
        public event Action<bool> WhenValueChanged = delegate (bool b) { };
        public event Action<TargetState> WhenStateChanged = delegate { };
        public event Action<Vector3> WhenDisplacementLocalChanged = delegate { };
        public bool Value { get; private set; }

        public event Action WhenPressBegin = delegate { };
        public event Action WhenPressEnd = delegate { };

        [SerializeField] private Target _target;

        [SerializeField] private MechanicDragMove _dragMove;
        public MechanicDragMove MechanicDragMove => _dragMove;

        [SerializeField] private MechanicDragCancel _dragCancel;
        public MechanicDragCancel MechanicDragCancel => _dragCancel;

        [SerializeField] private MechanicDragScroll _dragScroll;
        public MechanicDragScroll MechanicDragScroll => _dragScroll;

        public Target Target => _target;
        public TargetState State => _target.State;

        [SerializeField, ReadOnly] private bool _isSelectionActive;
        [SerializeField, ReadOnly] private bool _hasDragged;

        protected void Awake()
        {
            this.Ensure(ref _target);
            this.Ensure(ref _dragMove);
            this.Ensure(ref _dragCancel);
            this.Ensure(ref _dragScroll);

            _target.WhenHovering += Hovering;
            _target.WhenSelecting += Selecting;
            _target.WhenStateChanged += TargetStateChanged;

            _dragCancel.WhenDragStateChanged += DragStateChanged;
            _dragCancel.WhenDisplacementLocalChanged += DragPositionChanged;
            _dragScroll.WhenScrollEvent += ScrollEvent;
        }

        public event Action<DPad> WhenDPad = delegate { };
        private void Hovering(IHover hover)
        {
            if (!hover.DPad.IsNone()) WhenDPad(hover.DPad);
        }

        private void DragPositionChanged(Vector3 displacementLocal)
        {
            WhenDisplacementLocalChanged(displacementLocal);
        }

        private void Selecting(ISelection selection)
        {
            if (selection.Phase.IsBegin())
            {
                _isSelectionActive = true;
                WhenPressBegin();
            }

            if (selection.Phase.IsEnded())
            {
                if (_isSelectionActive)
                {
                    if (_dragCancel.State.IsIn() && !_hasDragged) Click();
                    else WhenCancelled();
                }
                _isSelectionActive = false;
                _hasDragged = false;

                WhenPressEnd();
            }
        }

        private void ScrollEvent(ScrollEvent scrollEvent)
        {
            if (_isSelectionActive && scrollEvent.GrabPhase.IsBegin()) InterruptSelection();
        }

        public void InterruptSelection()
        {
            if (_isSelectionActive) WhenCancelled();
            _isSelectionActive = false;
            TargetStateChanged(_target.State.IsHover() ? TargetState.Hover : TargetState.Normal);
        }

        private void DragStateChanged(DragCancelState driftState)
        {
            if (driftState == DragCancelState.Out) _hasDragged = true;
            UpdateTargetState();
        }

        private void TargetStateChanged(TargetState state)
        {
            UpdateTargetState();
        }

        private void UpdateTargetState()
        {
            TargetState state = _target.State;
            if (_dragCancel.State.IsOut() && _dragCancel.IsGrabbing) state = TargetState.Drift;
            WhenStateChanged(state);
        }

        public void Click()
        {
            WhenClicked();
        }
    }
}
