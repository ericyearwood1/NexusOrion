// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public enum ScrollAxis
    {
        Any,
        Vertical,
        Horizontal
    }

    public interface IScrollEventHandler
    {
        void ApplyScroll(ScrollEvent scrollEvent);
    }

    [Serializable]
    public class ScrollEvent : BubbleEvent
    {
        public static ScrollEvent Null { get; } = new(null, null, 0);

        public static ScrollEvent CreatePagination(Component target, MonoBehaviour source, DPad dPad)
        {
            if (dPad.IsNone()) return Null;
            var dir = dPad.ToVector2();
            var isHorizontal = Mathf.Abs(dir.x) > Mathf.Abs(dir.y);
            var pagination = (int)Mathf.Sign(isHorizontal ? dir.x : dir.y);
            var axis = isHorizontal ? ScrollAxis.Horizontal : ScrollAxis.Vertical;
            return new ScrollEvent(target, source) { Pagination = pagination, Axis = axis, influenceInertia = false };
        }

        public static ScrollEvent CreateDeltaTimeVelocity(Component target, MonoBehaviour source, Vector2 dir, SelectionPhase grabPhase = SelectionPhase.None)
        {
            if (dir == default) return Null;
            var isHorizontal = Mathf.Abs(dir.x) > Mathf.Abs(dir.y);
            var velocity = isHorizontal ? dir.x : dir.y;
            var axis = isHorizontal ? ScrollAxis.Horizontal : ScrollAxis.Vertical;
            return new ScrollEvent(target, source) { DeltaMeters = velocity * Time.deltaTime, Axis = axis, influenceInertia = true, GrabPhase = grabPhase};
        }

        public ScrollEvent(Component target, MonoBehaviour source, float? createdAt = null) : base(target)
        {
            this.createdAt = createdAt ?? Time.time;
            this.target = target;
            this.source = source;
        }

        public bool IsEmpty => target == null || (Pagination == 0 && AbsoluteMeters == 0 && DeltaMeters == 0);

        public SelectionPhase GrabPhase;
        public float createdAt;
        public int Pagination;
        public float AbsoluteMeters;
        public float DeltaMeters;
        public ScrollAxis Axis;
        public Component target;
        public Component handledBy;
        public MonoBehaviour source;
        public bool influenceInertia;
    }
}
