using System;
using System.Collections.Generic;
using UnityEngine;
using ARGlasses.Components.Scroll;
using ARGlasses.Interaction;
using OSIG.Tools.Layout;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace ARGlasses.Components
{
    public class ListItemScrollTransitionManagerDistanceBased : MonoBehaviour
    {
        [SerializeField] private RectTransform _viewport;
        [SerializeField] private OCLayoutStack _stack;
        [SerializeField] private bool _useFixedVisibleCount;
        [SerializeField] private float _fixedVisibleCount = 3;
        [SerializeField] private float _scrollSpeedAnimationMultiplier = 4;
        [SerializeField] private ScrollModel _scrollModel;
        [SerializeField] private ListItemPool _pool;
        [SerializeField] private float _maxDistanceForFadeOut = .05f;
        public Action<ArgListItem> OnChildBecameVisible;
        public Action<ArgListItem> OnChildBecameInvisible;

        private List<ArgListItem> _children = new List<ArgListItem>();

        private Dictionary<ArgListItem, VisibilityState> _visibilityStates =
            new Dictionary<ArgListItem, VisibilityState>();

        private Dictionary<ArgListItem, float> _distanceFromViewport = new Dictionary<ArgListItem, float>();
        private float _scrollNormalizeCurrent;
        private float _scrollNormalizeDelta;
        private float _animationSpeed;
        private RectTransform _rectTransform;

        void Start()
        {
            _rectTransform = GetComponent<RectTransform>();
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
                    _visibilityStates[child] = VisibilityState.Invisible;
                }
            }

            EnsureViewportVisibleItems(true);
        }

        void Update()
        {
            EnsureViewportVisibleItems();
        }

        private void EnsureViewportVisibleItems(bool resolveImmediately = false)
        {
            float halfViewportHeight = _viewport.rect.height / 2;

            foreach (var child in _children)
            {
                Vector2 childBottomEdge = _viewport.InverseTransformPoint(child.RectTransform.position -
                                                                          (Vector3.up *
                                                                           (child.RectTransform.rect.height / 2)));
                Vector2 childTopEdge = _viewport.InverseTransformPoint(child.RectTransform.position +
                                                                       (Vector3.up * (child.RectTransform.rect.height /
                                                                           2)));

                VisibilityState currentState =
                    GetViewportVisibilityState(_viewport, child);

                _distanceFromViewport[child] = Mathf.Max(
                    Mathf.Abs(childTopEdge.y) - halfViewportHeight,
                    Mathf.Abs(childBottomEdge.y) - halfViewportHeight
                );

                child.View.DistanceVisibilityChange(_distanceFromViewport[child] / _maxDistanceForFadeOut, currentState);
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
                // Calculate the distance from the item to the top and bottom of the viewport
                float distanceToTop = Mathf.Abs(childTopEdge.y - halfViewportHeight);
                float distanceToBottom = Mathf.Abs(childBottomEdge.y - (-halfViewportHeight));

                // Compare the distances and return the appropriate state
                if (distanceToTop < distanceToBottom)
                {
                    return VisibilityState.TopEdge;
                }
                else
                {
                    return VisibilityState.BottomEdge;
                }
            }
        }

        void OnDrawGizmosSelected()
        {
            if (_rectTransform == null)
                _rectTransform = GetComponent<RectTransform>();
            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(transform.position, new Vector3(_rectTransform.rect.size.x, _rectTransform.rect.size.y + _maxDistanceForFadeOut*2, .001f));

            foreach (var child in _children)
            {
                Gizmos.color =
                    _visibilityStates.ContainsKey(child) && _visibilityStates[child] != VisibilityState.Invisible
                        ? Color.green
                        : Color.red;
                Gizmos.DrawWireCube(child.RectTransform.position, child.RectTransform.rect.size);

                if (_distanceFromViewport.ContainsKey(child))
                {
                    Gizmos.color = Color.yellow;
                    float distance = _distanceFromViewport[child];
                    Gizmos.DrawLine(child.RectTransform.position, child.RectTransform.position + Vector3.up * distance);
                    Gizmos.DrawLine(child.RectTransform.position, child.RectTransform.position - Vector3.up * distance);

#if UNITY_EDITOR
                    // Display the distance as a text label above the item
                    string label = string.Format("Distance: {0:0.00}", distance);
                    Handles.Label(child.RectTransform.position, label);
#endif
                }
            }
        }
    }
}
