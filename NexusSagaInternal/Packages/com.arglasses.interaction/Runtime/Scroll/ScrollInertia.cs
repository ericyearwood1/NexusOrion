// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public class ScrollInertia : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ScrollController _controller;
        [SerializeField, ReadOnly] private Inertia _inertia;
        public Inertia Inertia => _inertia;

        protected void Awake()
        {
            this.Ensure(ref _inertia);
        }

        protected void Start()
        {
            this.Sibling(ref _controller);
            _controller.WhenScrollApplied += ControllerScrollApplied;

            _inertia.Boundaries = _controller;
            _inertia.WhenDelta += InertiaDelta;
        }

        private void ControllerScrollApplied(ScrollEvent scrollEvent)
        {
            if (!scrollEvent.influenceInertia || scrollEvent.source == this) return;
            var deltaMeters = ScrollController.ScrollToInertialSpace(scrollEvent.DeltaMeters);
            _inertia.AddDelta(deltaMeters, scrollEvent.GrabPhase);
            _lastReceivedScroll = scrollEvent;
        }

        private void InertiaDelta(Inertia.Event inertia)
        {
            var delta = ScrollController.InertialSpaceToScroll(inertia.Delta);
            var absolute = ScrollController.InertialSpaceToScroll(inertia.Absolute);
            var smallDelta = Mathf.Abs(delta) <= Inertia.MinInteriaMagnitude;
            var smallAbsolute = Mathf.Abs(absolute) <= Inertia.MinInteriaMagnitude;
            if (smallDelta && smallAbsolute) return;

            var targetElement = _lastReceivedScroll != null ? _lastReceivedScroll.Target : _controller;
            var scrollAxis = _controller.IsHorizontal ? ScrollAxis.Horizontal : ScrollAxis.Vertical;
            lastSentScroll = new ScrollEvent(targetElement, this)
            {
                Axis = scrollAxis,
                DeltaMeters = delta,
                AbsoluteMeters = absolute,
                influenceInertia = false
            };
            _controller.ApplyScroll(lastSentScroll);
        }

        [SerializeField, ReadOnly] private ScrollEvent _lastReceivedScroll;
        [SerializeField, ReadOnly] private ScrollEvent lastSentScroll;
    }
}
