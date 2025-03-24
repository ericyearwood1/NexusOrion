// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections.Generic;
using System.Linq;
using OSIG.Tools.Layout;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ScrollModelRectTransform : ScrollModel, IOCLayoutListener
    {
        private RectTransform _rectTransform;
        private RectTransform RectTransform => this.Ensure(ref _rectTransform);

        [SerializeField] private int _steps = 0;
        public void SetSteps(int value) => _steps = value;

        public override bool IsHorizontal => RectTransform.rect.width > RectTransform.rect.height;
        public float ContainerSize => IsHorizontal ? RectTransform.rect.width : RectTransform.rect.height;

        [SerializeField] private float _scrollDistanceMeters;
        [SerializeField] private float _cachedNormalized;
        [SerializeField, ReadOnly] private bool _isGrabbed;

        private void Awake()
        {
            this.Ensure(ref _rectTransform);
        }

        protected override float GetMeters() => _scrollDistanceMeters;
        protected override void SetMeters(float value)
        {
            _scrollDistanceMeters = Mathf.Clamp(value, 0, Max);
            _cachedNormalized = Mathf.InverseLerp(Min, Max, _scrollDistanceMeters);
        }

        public override void GrabChanged(bool isGrabbed) => _isGrabbed = isGrabbed;

        public override float ContainerMax => ContainerSize / Mathf.Max(1, _steps);
        public override float Max => ContainerSize;

        private readonly List<float> _paginationLandmarks = new();
        public override List<float> PaginationLandmarks
        {
            get
            {
                _paginationLandmarks.Clear();
                for (int i = 0; i < _steps + 1; i++) _paginationLandmarks.Add(ContainerMax * i);
                return _paginationLandmarks;
            }
        }

        protected void OnRectTransformDimensionsChange()
        {
            UpdateDimensions();
        }

        public void OnApplyLayout(IOCLayoutComponent layoutComponent)
        {
            if (Application.isPlaying) UpdateDimensions();
        }

        private void UpdateDimensions()
        {
            ValueNormalized = _cachedNormalized;
        }
    }
}
