// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class MechanicDragScroll : MonoBehaviour
    {
        public enum Style
        {
            Grab,
            Velocity
        }

        [SerializeField] private Style _style;

        [SerializeField] private MechanicDragMove _dragMoveMechanic;
        public MechanicDragMove DragMoveMechanic => this.Ensure(ref _dragMoveMechanic);
        public Selectable Target => _dragMoveMechanic.Target;

        [SerializeField, ReadOnly] private Drag _lastDrag;

        public event Action<ScrollEvent> WhenScrollEvent = delegate { };

        protected void Awake() => this.Ensure(ref _dragMoveMechanic);

        private void OnEnable()
        {
            _dragMoveMechanic.WhenDragMove += HandleDragMove;
            Target.WhenHovering += Hovering;
        }

        private void OnDisable()
        {
            _dragMoveMechanic.WhenDragMove -= HandleDragMove;
            Target.WhenHovering -= Hovering;
        }

        private void Hovering(IHover hover)
        {
            var dPad = hover.DPad;
            if (dPad.IsNone()) return;
            RouteScroll(ScrollEvent.CreatePagination(target: hover.Target, source: this, dPad));
        }

        [SerializeField] private bool _imuJoystickScroll;
        [SerializeField, ReadOnly] private ScrollEvent _lastRoutedEvent; // just for debugging

        private void RouteScroll(ScrollEvent scrollEventArgs)
        {
            scrollEventArgs.influenceInertia = false;
            EventBubbler.Bubble(scrollEventArgs, (Action<IScrollEventHandler, ScrollEvent>)((handler, e) => handler.ApplyScroll(e)));
            _lastRoutedEvent = scrollEventArgs;
        }

        private void HandleDragMove(Mechanic.DragMove.Event dragMove)
        {
            var drag = dragMove.Drag;
            var phase = dragMove.Phase;

            if (phase.IsBegin()) BubbleScrollGrabEvent(SelectionPhase.Begin);

            if (phase.IsUpdate())
            {
                if (_style == Style.Grab) GrabScrollUpdate(dragMove);
                if (_style == Style.Velocity) VelocityScrollUpdate(dragMove);
            }

            if (phase.IsEnded()) BubbleScrollGrabEvent(SelectionPhase.Success);

            _lastDrag = (Drag)drag;
        }

        private void VelocityScrollUpdate(Mechanic.DragMove.Event dragMove)
        {
            var drag = dragMove.Drag;
            var target = dragMove.Selection.Begin.Target;
            var dragLocal = drag.DeltaLocal;
            var phase = SelectionPhase.Update;
            var scrollEvent = ScrollEvent.CreateDeltaTimeVelocity(target, source: this, dragLocal, phase);
            RouteScroll(scrollEvent);
        }

        private void GrabScrollUpdate(Mechanic.DragMove.Event dragMove)
        {
            var phase = dragMove.Phase;

            if (phase.IsBegin()) BubbleScrollGrabEvent(SelectionPhase.Begin);

            if (phase.IsMoving())
            {
                var deltaLocalThisFrame = dragMove.Drag.DeltaLocal - _lastDrag.DeltaLocal;
                BubbleDeltaScrollEvent(deltaLocalThisFrame.x, ScrollAxis.Horizontal);
                BubbleDeltaScrollEvent(deltaLocalThisFrame.y, ScrollAxis.Vertical);
            }

            if (phase.IsEnded()) BubbleScrollGrabEvent(SelectionPhase.Success);
        }

        private void BubbleDeltaScrollEvent(float deltaMeters, ScrollAxis scrollAxis)
        {
            var e = new ScrollEvent(_dragMoveMechanic.Target, this)
            {
                DeltaMeters = deltaMeters,
                Axis = scrollAxis,
                GrabPhase = SelectionPhase.Update,
                influenceInertia = true
            };
            RouteEvent(e);
        }

        private void BubbleScrollGrabEvent(SelectionPhase grabPhase)
        {
            var eventArgs = new ScrollEvent(_dragMoveMechanic.Target, this) { GrabPhase = grabPhase, influenceInertia = true };
            RouteEvent(eventArgs);
        }

        private void RouteEvent(ScrollEvent scrollEvent)
        {
            void Callback(IScrollEventHandler handler, ScrollEvent e) => handler.ApplyScroll(e);
            EventBubbler.Bubble(scrollEvent, (Action<IScrollEventHandler, ScrollEvent>)Callback);
            if(scrollEvent.HandlePolicy != BubbleEventHandlePolicy.None) WhenScrollEvent(scrollEvent);
        }
    }
}
