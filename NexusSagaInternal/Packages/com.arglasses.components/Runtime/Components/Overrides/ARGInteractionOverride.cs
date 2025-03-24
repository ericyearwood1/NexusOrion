using System;
using System.Collections;
using ARGlasses.Interaction;
using ITK;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Components
{
    public class ARGInteractionOverride : MonoBehaviour
    {
        [SerializeField, Tooltip("Force Drag-Cancelling enabled/disabled")] [Group("Drag Cancel")]
        private bool _dragCancelEnabled = true;

        [SerializeField, Tooltip("Override maximum distance when Drag-Cancelling (-1 to ignore override)")]
        [Group("Drag Cancel")]
        private OverridableValue<float> _dragCancelMaxDisplacement =
            new(false, MechanicDragCancel.DefaultMaxDisplacement);

        [SerializeField, Tooltip("Override maximum distance when Drag-Cancelling (-1 to ignore override)")]
        [Group("Drag Cancel")]
        private OverridableValue<float> _dragCancelSnapBackSpeed = new(false, MechanicDragCancel.DefaultSnapBackSpeed);

        [SerializeField, Tooltip("Override deadzone needed before triggering drag events")] [Group("Drag Move")]
        private OverridableValue<float> _dragMoveDeadzone = new(false, MechanicDragMove.DefaultDeadzone);

        [SerializeField, Tooltip("Override drag break ratio")] [Group("Drag Cancel")]
        private OverridableValue<float> _dragBreakRatio = new(false, MechanicDragCancel.DefaultBreakRatio);

        [SerializeField, Tooltip("Override drag break ratio")] [Group("Drag Cancel")]
        private OverridableValue<float> _dragBreakNonRatioThreshold =
            new(false, MechanicDragCancel.DefaultNonRatioBreakThreshold);

        [SerializeField, Tooltip("Override drag break ratio")] [Group("Drag Cancel")]
        private OverridableValue<bool> _useRatioBasedDragBreak = new(false, true);

        [FormerlySerializedAs("_colliderOverride")]
        [SerializeField, Tooltip("Override drag threshold needed before bubbling scroll events")]
        private OverridableValue<Collider> _targetCollider = new(false, null);

        [SerializeField, ReadOnly] private MechanicDragCancel _dragCancel;
        [SerializeField, ReadOnly] private MechanicDragMove _dragMove;
        [SerializeField, ReadOnly] private Target _target;

        private void Awake()
        {
            var viewController = GetComponent<ViewController>();

            if (viewController == null)
            {
                Debug.LogWarning("No view controller sibling to interaction override.");
                enabled = false;
                return;
            }

            viewController.SubscribeToInitialization((() =>
            {
                this.Descendant(ref _dragCancel, optional: true);
                if (_dragCancel)
                {
                    _dragCancel.enabled = _dragCancelEnabled;

                    if (_dragCancelMaxDisplacement.Override)
                    {
                        if (_dragCancelMaxDisplacement.Value > 0)
                            _dragCancel.SetMaxDisplacement(_dragCancelMaxDisplacement.Value);
                    }

                    if (_dragCancelSnapBackSpeed.Override)
                    {
                        if (_dragCancelSnapBackSpeed.Value > 0)
                            _dragCancel.SetSnapBackSpeed(_dragCancelSnapBackSpeed.Value);
                    }

                    if (_useRatioBasedDragBreak.Override)
                    {
                        _dragCancel.SetUseRatioBasedBreakThreshold(_useRatioBasedDragBreak.Value);
                    }

                    if (_dragBreakRatio.Override)
                    {
                        _dragCancel.SetBreakRatio(_dragBreakRatio.Value);
                    }

                    if (_dragBreakNonRatioThreshold.Override)
                    {
                        _dragCancel.SetNonRatioBreakThreshold(_dragBreakNonRatioThreshold.Value);
                    }
                }

                this.Descendant(ref _dragMove, optional: true);
                if (_dragMove)
                {
                    if (_dragMoveDeadzone.Override)
                    {
                        _dragMove.DeadzoneMeters = _dragMoveDeadzone.Value;
                    }
                }

                this.Descendant(ref _target, optional: true);
                if (_target)
                {
                    if (_targetCollider.Override)
                    {
                        _target.Colliders = new[] { _targetCollider.Value };
                    }
                }
            }));
        }
    }
}

[System.Serializable]
public class OverridableValue<T>
{
    public OverridableValue(bool shouldOverride, T value)
    {
        Override = shouldOverride;
        Value = value;
    }

    public bool Override;
    public T Value;
}
