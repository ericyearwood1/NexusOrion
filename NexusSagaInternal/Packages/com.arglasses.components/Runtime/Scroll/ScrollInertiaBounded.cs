// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public class ScrollInertiaBounded : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ScrollControllerBounded _controller;
        [SerializeField, ReadOnly] private Inertia _inertia;
        public Inertia Inertia => _inertia;

        protected void Awake()
        {
            this.Ensure(ref _inertia);
        }

        protected void Start()
        {
            this.Sibling(ref _controller);
            _controller.WhenScrollApplied += ScrollApplied;

            _inertia.Boundaries = _controller;
            _inertia.WhenDelta += inertia =>
                UpdateInertia(ScrollControllerBounded.InertialSpaceToScroll(inertia.Delta), ScrollControllerBounded.InertialSpaceToScroll(inertia.Absolute));
        }

        private void ScrollApplied(ScrollEvent @event)
        {
            if (!@event.influenceInertia || @event.source == this) return;
            var deltaMeters = ScrollControllerBounded.ScrollToInertialSpace(@event.DeltaMeters);
            _inertia.AddDelta(deltaMeters, @event.GrabPhase);
            _lastReceivedScroll = @event;
        }

        private void UpdateInertia(float delta, float absolute)
        {
            var targetElement = _lastReceivedScroll != null ? _lastReceivedScroll.Target : _controller;

            if (Mathf.Abs(delta) <= Inertia.MinInteriaMagnitude && Mathf.Abs(absolute) <= Inertia.MinInteriaMagnitude) return;

            lastSentScroll = new ScrollEvent(targetElement, this)
            {
                Axis = _controller.IsHorizontal ? ScrollAxis.Horizontal : ScrollAxis.Vertical, DeltaMeters = delta, AbsoluteMeters = absolute, influenceInertia = false
            };
            _controller.ApplyScroll(lastSentScroll);
        }

        [SerializeField, ReadOnly] private ScrollEvent _lastReceivedScroll;
        [SerializeField, ReadOnly] private ScrollEvent lastSentScroll;
    }
}
