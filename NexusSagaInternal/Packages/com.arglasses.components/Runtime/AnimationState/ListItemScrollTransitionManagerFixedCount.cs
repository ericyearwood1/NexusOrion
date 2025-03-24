using System;
using System.Collections.Generic;
using System.Linq;
using ARGlasses.Components.Scroll;
using ARGlasses.Interaction;
using OSIG.Tools.Layout;
using UnityEngine;

namespace ARGlasses.Components
{
    public class ListItemScrollTransitionManagerFixedCount : MonoBehaviour
    {
        [SerializeField] private RectTransform _viewport;
        [SerializeField] private OCLayoutStack _stack;
        [SerializeField] private bool _useFixedVisibleCount;
        [SerializeField] private float _fixedVisibleCount = 3;
        [SerializeField] private float _scrollSpeedAnimationMultiplier = 4;
        [SerializeField] private ScrollModel _scrollModel;
        [SerializeField] private ListItemPool _pool;
        public Action<ArgListItem> OnChildBecameVisible;
        public Action<ArgListItem> OnChildBecameInvisible;

        private List<ArgListItem> _children = new List<ArgListItem>();

        private Dictionary<ArgListItem, VisibilityState> _visibilityStates =
            new Dictionary<ArgListItem, VisibilityState>();

        private float _scrollNormalizeCurrent;
        private float _scrollNormalizeDelta;
        private float _animationSpeed;

        void Start()
        {
            _scrollNormalizeCurrent = _scrollModel.ValueNormalized;
            _scrollModel.WhenValueNormalizedChanged += f =>
            {
                _scrollNormalizeDelta = Mathf.Abs(f - _scrollNormalizeCurrent);
                _animationSpeed = 1 + _scrollNormalizeDelta * _scrollSpeedAnimationMultiplier;
                _scrollNormalizeCurrent = f;
            };

            for (int i = 0; i < _stack.transform.childCount; i++)
            {
                var child = _stack.transform.GetChild(i).GetComponent<ArgListItem>();
                if (child != null)
                {
                    _children.Add(child);
                    _visibilityStates[child] = VisibilityState.Invisible; // Initialize all to invisible
                }
            }

            if (_useFixedVisibleCount)
            {
                // If using fixed count, initialize visibility based on distance to viewport center
                EnsureFixedVisibleItems(true);
            }
            else
            {
                EnsureViewportVisibleItems(true);
            }
        }

        void Update()
        {
            if (_useFixedVisibleCount)
            {
                EnsureFixedVisibleItems();
            }
            else
            {
                EnsureViewportVisibleItems();
            }
        }

        private void EnsureFixedVisibleItems(bool resolveImmediately = false)
        {
            // Find center of viewport
            float viewportCenter = _viewport.position.y;

            // Calculate distance from each child to the center of the viewport and sort by distance
            var sortedChildren = _children
                .Select(child => new
                    { Child = child, Distance = Mathf.Abs(viewportCenter - child.RectTransform.position.y) })
                .OrderBy(item => item.Distance)
                .ToList();

            // For each child, set visibility based on whether it's in the closest `fixedVisibleItems`
            for (int i = 0; i < sortedChildren.Count; i++)
            {
                var child = sortedChildren[i].Child;

                // Determine whether the item is closer to the top of the viewport
                bool closerToTopOfViewport = child.RectTransform.position.y > viewportCenter;

                bool isVisible = i < _fixedVisibleCount;
                VisibilityState currentState = isVisible ? VisibilityState.Visible : VisibilityState.Invisible;

                if (currentState != _visibilityStates[child] || resolveImmediately)
                {
                    if (!isVisible)
                    {
                        OnChildBecameInvisible?.Invoke(child);
                        child.View.OnScrollVisibilityChange(false, closerToTopOfViewport, _animationSpeed,
                            resolveImmediately, () =>
                            {
                                //Debug.Log("FINISHED FADING RECEIVED");
                                //_pool.Return(child);
                            });
                    }
                    else
                    {
                        OnChildBecameVisible?.Invoke(child);
                        child.View.OnScrollVisibilityChange(true, closerToTopOfViewport, _animationSpeed,
                            resolveImmediately);
                    }

                    _visibilityStates[child] = currentState;
                }
            }
        }

        private void EnsureViewportVisibleItems(bool resolveImmediately = false)
        {
            foreach (var child in _children)
            {
                VisibilityState currentState = GetViewportVisibilityState(_viewport, child);

                if (currentState != _visibilityStates[child] || resolveImmediately)
                {
                    if (_visibilityStates[child] == VisibilityState.Invisible &&
                        currentState != VisibilityState.Invisible)
                    {
                        // Changed from invisible to visible/partially visible
                        OnChildBecameVisible?.Invoke(child);
                        child.View.OnScrollVisibilityChange(true, currentState == VisibilityState.BottomEdge,
                            _animationSpeed,
                            resolveImmediately);
                    }
                    else if (_visibilityStates[child] != VisibilityState.Invisible &&
                             currentState == VisibilityState.Invisible)
                    {
                        // Changed from visible/partially visible to invisible
                        OnChildBecameInvisible?.Invoke(child);
                        child.View.OnScrollVisibilityChange(false,
                            _visibilityStates[child] == VisibilityState.BottomEdge, _animationSpeed,
                            resolveImmediately);
                    }

                    _visibilityStates[child] = currentState;
                }
            }
        }

        public VisibilityState GetViewportVisibilityState(RectTransform viewport, ArgListItem child)
        {
            Vector2 childBottomEdge = viewport.InverseTransformPoint(child.RectTransform.position -
                                                                     (Vector3.up * (child.RectTransform.rect.height /
                                                                         2)));
            Vector2 childTopEdge = viewport.InverseTransformPoint(child.RectTransform.position +
                                                                  (Vector3.up * (child.RectTransform.rect.height / 2)));

            float halfViewportHeight = viewport.rect.height / 2;

            bool isTopEdgeInViewport = childTopEdge.y <= halfViewportHeight && childTopEdge.y >= -halfViewportHeight;
            bool isBottomEdgeInViewport =
                childBottomEdge.y <= halfViewportHeight && childBottomEdge.y >= -halfViewportHeight;

            if (isTopEdgeInViewport && isBottomEdgeInViewport)
            {
                return VisibilityState.Visible;
            }
            else if (isTopEdgeInViewport)
            {
                return VisibilityState.TopEdge;
            }
            else if (isBottomEdgeInViewport)
            {
                return VisibilityState.BottomEdge;
            }
            else
            {
                return VisibilityState.Invisible;
            }
        }

        void OnDrawGizmosSelected()
        {
            foreach (var child in _children)
            {
                Gizmos.color =
                    _visibilityStates.ContainsKey(child) && _visibilityStates[child] != VisibilityState.Invisible
                        ? Color.green
                        : Color.red;
                Gizmos.DrawWireCube(child.RectTransform.position, child.RectTransform.rect.size);
            }
        }
    }
}
