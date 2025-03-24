// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public interface ISlider
    {
        event Action<TargetState> WhenStateChanged;
        event Action<Vector3> WhenDisplacementLocalChanged;
        public event Action<float> WhenValueChanged;
        event Action<float> WhenValueNormalizedChanged;
        bool IsHorizontal { get; }
        float Value { get; set; }
        float ValueNormalized { get; set; }
        float MinValue { get; }
        float MaxValue { get; }
        int Steps { get; }
    }

    public class SliderModel : MonoBehaviour, ISlider
    {
        [SerializeField, ReadOnly] private TargetState _state;

        public event Action<TargetState> WhenStateChanged = delegate { };
        public event Action<Vector3> WhenDisplacementLocalChanged = delegate {  };
        public event Action<float> WhenValueChanged = delegate { };
        public event Action<float> WhenValueNormalizedChanged = delegate { };
        public bool IsHorizontal => ScrollController.IsHorizontal;

        // We use a ScrollModel as our Model
        [SerializeField] private ScrollModelRectTransform _scrollModel;

        public float MinValue
        {
            get
            {
                Init();
                return _scrollModel.Min;
            }
        }
        public float MaxValue
        {
            get
            {
                Init();
                return _scrollModel.Max;
            }
        }

        public float Value {
            get
            {
                Init();
                return _scrollModel.Value;
            }
            set
            {
                Init();
                _scrollModel.Value = value;
            }
        }

        public float ValueNormalized
        {
            get
            {
                Init();
                return _scrollModel.ValueNormalized;
            }
            set
            {
                Init();
                _scrollModel.ValueNormalized = value;
            }
        }

        [SerializeField] private int _steps = 0;
        public int Steps => _steps;

        [SerializeField] private Target _target;
        public Target Target
        {
            get
            {
                Init();
                return _target;
            }
        }

        [SerializeField] private MechanicDragCancel _drift;
        public MechanicDragCancel Drift => _drift;

        [SerializeField] private ScrollController _scrollController;
        public ScrollController ScrollController
        {
            get
            {
                Init();
                return _scrollController;
            }
        }

        [SerializeField] private ScrollInertia _scrollInertia;
        public ScrollInertia ScrollInertia => _scrollInertia;

        [SerializeField] private MechanicDragScroll _eventEmitter;

        protected void Awake()
        {
            Init();
            _target.WhenStateChanged += StateChanged;
            _drift.WhenDisplacementLocalChanged += v => WhenDisplacementLocalChanged(v);
            _scrollModel.WhenValueNormalizedChanged += (fill) => WhenValueChanged(_scrollModel.Value);
            _scrollModel.WhenValueNormalizedChanged += (fill) => WhenValueNormalizedChanged(fill);
            SetSteps(_steps);
        }

        private bool _wasInit;
        private void Init()
        {
            if (_wasInit) return;
            _wasInit = true;
            this.Ensure(ref _target);
            this.Ensure(ref _drift);
            if (!_scrollModel) _scrollModel = GetComponentInChildren<ScrollModelRectTransform>();
            this.Ensure(ref _scrollModel);
            this.Ensure(ref _scrollController);
            Assert.AreEqual(_scrollController.Model, _scrollModel); // todo let's clean this up
            this.Ensure(ref _scrollInertia);
            this.Ensure(ref _eventEmitter);
        }

        private void StateChanged(TargetState state)
        {
            _state = state;
            WhenStateChanged.Invoke(_state);
        }

        public void SetSteps(int steps)
        {
            _steps = steps;
            _scrollModel.SetSteps(_steps);
        }
    }
}
