// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public abstract class ScrollModel : MonoBehaviour
    {
        public virtual void SetLockedToEnd(bool lockedToEnd)
        {
        }

        public virtual float Value
        {
            get => GetMeters();
            set
            {
                SetMeters(value);
                WhenValueChanged(Value);
                WhenValueNormalizedChanged(ValueNormalized);
            }
        }

        public float ValueNormalized
        {
            get => Mathf.Clamp01(Value / (Max == 0 ? 1 : Max));
            set => Value = value * (Max == 0 ? 1 : Max);
        }

        public abstract bool IsHorizontal { get; }
        public virtual float Min => 0;
        public virtual float Max => 1;
        public abstract float ContainerMax { get; }
        public event Action<float> WhenValueChanged = delegate { };
        public event Action<float> WhenValueNormalizedChanged = delegate { };
        public abstract List<float> PaginationLandmarks { get; }

        protected abstract float GetMeters();
        protected abstract void SetMeters(float value);

        protected void OnValueChanged(float value)
        {
            WhenValueChanged(value);
        }

        protected void OnValueNormalizedChanged(float value)
        {
            WhenValueNormalizedChanged(value);
        }

        public abstract void GrabChanged(bool isGrabbed);
    }
}
