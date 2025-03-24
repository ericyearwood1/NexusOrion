using System.Collections.Generic;
using ARGlasses.Components.Scroll;
using ARGlasses.Interaction;
using UnityEngine;

namespace ARGlasses.Components
{
    public class ScrollModelInfiniteStack : ScrollModel
    {
        [SerializeField] private PoolingScroller _poolingScroller;

        public float PaginationDistanceMeters => _poolingScroller.PaginationDistance;

        public override bool IsHorizontal => false;//_scrollStack.Direction is LeftToRight or LeftToRight;

        protected override float GetMeters()
        {
            var value = _poolingScroller.ScrollDistance.GetMeters(_poolingScroller.UnitsContext);
            //if (FlipScrollStackProgress) return Max - value;
            return value;
        }

        protected override void SetMeters(float value)
        {
            //if (FlipScrollStackProgress) value = Max - value;
            _poolingScroller.ScrollDistance.SetFromMeters(_poolingScroller.UnitsContext, value);
            //isLockedToEnd = FlipScrollStackProgress ? _scrollStack.ScrollPercent < 0.005f : _scrollStack.ScrollPercent > 0.995f;
        }

        public override void GrabChanged(bool isGrabbed) => _isGrabbed = isGrabbed;

        public override float Max => _poolingScroller.Max;//_scrollStack.ScrollLength;

        public override float ContainerMax
        {
            get
            {
                var currentSize = _poolingScroller.GetComponent<RectTransform>().rect.size;
                _containerMeters = IsHorizontal ? currentSize.x : currentSize.y;
                return _containerMeters;
            }
        }

        public override List<float> PaginationLandmarks
        {
            get
            {
                _paginationLandmarks.Clear();

                float distance = PaginationDistanceMeters;
                int pages = Mathf.CeilToInt(Max / distance);
                for (int i = 0; i < pages+1; i++) _paginationLandmarks.Add(i * distance);

                return _paginationLandmarks;
            }
        }

        public float scrollToEndLerp = 4;
        public bool constrainOvershoot;
        public bool allowLockedToEnd;
        public bool isLockedToEnd;
        [SerializeField] private List<float> _paginationLandmarks = new List<float>();
        [SerializeField] private float _containerMeters;
        [SerializeField, ReadOnly] private bool _isGrabbed;

        private void Update()
        {
            if (!constrainOvershoot) return;

            if (Value < 0)
            {
                var meters = Mathf.Lerp(Value, 0, Time.deltaTime * scrollToEndLerp);
                SetMeters(meters);
            }

            if ((allowLockedToEnd && isLockedToEnd) || Value > Max)
            {
                var meters = Mathf.Lerp(Value, Max, Time.deltaTime * scrollToEndLerp);
                SetMeters(meters);
            }
        }
    }
}
