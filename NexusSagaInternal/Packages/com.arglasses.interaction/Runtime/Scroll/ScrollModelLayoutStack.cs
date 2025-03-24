// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections;
using System.Collections.Generic;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;
using OSIG.Tools.Units;
using UnityEngine;
using static OSIG.Tools.Layout.OCLayoutStack.StackDirection;

namespace ARGlasses.Interaction
{
    public class ScrollModelLayoutStack : ScrollModel
    {
        [SerializeField, ReadOnly] private float _value;
        [SerializeField] private OCLayoutStack _scrollStack;
        [SerializeField] private float _elasticOvershootScale = 0.15f;
        [SerializeField] private float _scrollToEndLerp = 8;
        [SerializeField] private float _reboundLerp = 32;
        [SerializeField] private bool _isHorizontal;
        [SerializeField] private bool isLockedToEnd;
        public bool IsLockedToEnd => isLockedToEnd;
        [SerializeField] private List<float> _paginationLandmarks = new List<float>();

        [SerializeField, ReadOnly] private float _containerMeters;
        [SerializeField, ReadOnly] private float _lastValue;
        [SerializeField, ReadOnly] private bool _isGrabbed;
        [SerializeField, ReadOnly] private bool _allowLockedToEnd;

        public override float Value
        {
            get => GetMeters();
            set
            {
                SetMeters(value);
                OnValueChanged(Value);
                OnValueNormalizedChanged(ValueNormalized);
            }
        }

        private void Awake()
        {
            this.Descendant(ref _scrollStack, allowSibling: true);
            var scrollInertia = GetComponentInParent<ScrollInertia>();
            if (scrollInertia)
            {
                Debug.LogWarning("Remove ScrollInertia from parent please.");
                scrollInertia.enabled = false;
            }
            
            _scrollStack.OnContentLengthChanged += OnScrollStackContentLengthChanged;
        }

        private void OnScrollStackContentLengthChanged()
        {
            Update();
            OCLayoutSingletonDriver.ForceUpdateNowForAllLayoutInScene();
        }

        public override void GrabChanged(bool isGrabbed)
        {
            _isGrabbed = isGrabbed;
            if (isGrabbed) isLockedToEnd = false;
        }

        public override bool IsHorizontal => _isHorizontal;
        public float BoundedValue => Mathf.Clamp(Value, 0, Max);
        public float Overshoot => Value - BoundedValue;

        protected override float GetMeters() => _value;

        protected override void SetMeters(float value)
        {
            if (Mathf.Approximately(value, _value))
                return;

            _value = value;
            isLockedToEnd = _allowLockedToEnd && !_isGrabbed && (isLockedToEnd || ValueNormalized > 0.999995f);
            SetValueToScrollStack(value);
        }

        private void SetValueToScrollStack(float scrollStackValue)
        {
            float ElasticOvershootPosition(float overshoot)
            {
                var signedStrength = Mathf.Sign(overshoot) * 0.2f;
                return signedStrength * (overshoot / (overshoot + signedStrength));
            }

            scrollStackValue = BoundedValue + ElasticOvershootPosition(Overshoot) * ElasticOvershootScale;
            if (_scrollStack.Direction is LeftToRight or BottomToTop) scrollStackValue = Max - scrollStackValue;

            _scrollStack.SetScrollDistance(scrollStackValue.AsMeters());
        }

        public override float Max => _scrollStack.ScrollLength;

        public override float ContainerMax
        {
            get
            {
                var currentSize = _scrollStack.GetComponent<RectTransform>().rect.size;
                _containerMeters = IsHorizontal ? currentSize.x : currentSize.y;
                return _containerMeters;
            }
        }

        public override List<float> PaginationLandmarks
        {
            get
            {
                _paginationLandmarks.Clear();

                int pages = Mathf.CeilToInt(Max / ContainerMax);
                for (int i = 0; i < pages + 1; i++) _paginationLandmarks.Add(i * ContainerMax);

                return _paginationLandmarks;
            }
        }

        public float ElasticOvershootScale
        {
            get => _elasticOvershootScale;
            set => _elasticOvershootScale = value;
        }

        public void GoToEnd(float fromValue, float progress)
        {
            var meters = Mathf.Lerp(fromValue, Max, progress);
            SetMeters(meters);
        }

        public override void SetLockedToEnd(bool lockedToEnd)
        {
            _allowLockedToEnd = lockedToEnd;
            isLockedToEnd = lockedToEnd;
        }

        [SerializeField, ReadOnly] private float _inertia;
        private float _inertiaUpRate1 = 32;
        private float _inertiaDownRate1 = 2f;

        private void Update()
        {
            if (isLockedToEnd)
            {
                var meters = Mathf.Lerp(Value, Max, Time.deltaTime * _scrollToEndLerp);
                SetMeters(meters);
            }

            var delta = Value - _lastValue;
            if (_isGrabbed)
            {
                _inertia = Mathf.Lerp(_inertia, delta, Time.deltaTime * _inertiaUpRate1);
            }
            else
            {
                //todo why does overshoot lerps need to be slowed down? likely performance based
                if (_value > Max || _value < 0)
                {
                    _inertia = delta; //Mathf.Lerp(_inertia, delta, Time.deltaTime / Time.deltaTime * _inertiaDownRate1);
                }
                else
                {
                    _inertia = Mathf.Lerp(_inertia, delta, Time.deltaTime * _inertiaDownRate1);
                }

                var inertiaPosition = Mathf.Lerp(Value + _inertia, BoundedValue, Time.deltaTime * _reboundLerp);

                SetMeters(inertiaPosition);
            }

            _lastValue = Value;
        }
    }
}
