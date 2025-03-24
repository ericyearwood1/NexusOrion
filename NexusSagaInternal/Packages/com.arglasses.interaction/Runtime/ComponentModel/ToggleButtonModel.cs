// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public interface IToggle
    {
        event Action<bool> WhenValueChanged;
        event Action<TargetState> WhenStateChanged;
        event Action<Vector3> WhenDisplacementLocalChanged;
        bool Value { get; }
    }

    public class ToggleButtonModel : MonoBehaviour, IToggle
    {
        [SerializeField] private ButtonModel buttonModel;
        public ButtonModel ButtonModel => buttonModel;

        [SerializeField] private bool _value;

        public event Action<TargetState> WhenStateChanged = delegate {  };
        public event Action<Vector3> WhenDisplacementLocalChanged = delegate {  };
        public event Action<bool> WhenValueChanged = delegate { };

        public bool Value
        {
            get => _value;
            set
            {
                if (value == _value) return;
                _value = value;
                WhenValueChanged.Invoke(_value);
            }
        }

        protected void Awake()
        {
            this.Ensure(ref buttonModel);
            buttonModel.WhenClicked += () => Value = !_value;
            buttonModel.WhenStateChanged += state => WhenStateChanged(state);
            buttonModel.WhenDisplacementLocalChanged += displacementLocal => WhenDisplacementLocalChanged(displacementLocal);
        }
    }
}
