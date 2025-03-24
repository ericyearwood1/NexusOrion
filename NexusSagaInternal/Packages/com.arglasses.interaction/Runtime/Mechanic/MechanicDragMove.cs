using System;
using UnityEngine;
using UnityEngine.Assertions;
using static ARGlasses.Interaction.Mechanic.DragMove;
using static ARGlasses.Interaction.Mechanic.DragMove.Phase;

namespace ARGlasses.Interaction
{
    public static partial class ExtensionsMechanic
    {
        public static bool IsNone(this Phase phase) => phase == None;
        public static bool IsDeadzone(this Phase phase) => phase == Deadzone;
        public static bool IsBegin(this Phase phase) => phase == Begin;
        public static bool IsUpdate(this Phase phase) => phase == Update;
        public static bool IsSuccess(this Phase phase) => phase == Success;
        public static bool IsCancel(this Phase phase) => phase == Cancel;

        public static bool IsMoving(this Phase phase) => phase.IsBegin() || phase.IsUpdate();
        public static bool IsEnded(this Phase phase) => phase.IsSuccess() || phase.IsCancel() || phase.IsNone();
    }

    public static partial class Mechanic
    {
        public static class DragMove
        {
            [Serializable]
            public struct Event
            {
                public Phase Phase;
                public IDrag Drag;
                public ISelection Selection;
            }

            public enum Phase
            {
                None,
                Deadzone,
                Begin,
                Update,
                Success,
                Cancel,
            }
        }
    }

    public class MechanicDragMove : MonoBehaviour
    {
        public const float DefaultDeadzone = 0.05f;
        [SerializeField] private float _deadzoneMeters = DefaultDeadzone;
        public float DeadzoneMeters
        {
            get => _deadzoneMeters;
            set => _deadzoneMeters = value;
        }

        public event Action<Mechanic.DragMove.Event> WhenDragMove = delegate { };
        public event Action<float> WhenTwistRadians = delegate { };

        [SerializeField] private Selectable _target;
        public Selectable Target => this.Ensure(ref _target, defaultType: typeof(Target));

        [SerializeField, ReadOnly] private Mechanic.DragMove.Event _event;
        private Mechanic.DragMove.Event Event => _event;

        [SerializeField, ReadOnly] private Drag _lastCvDrag;
        [SerializeField, ReadOnly] private ImuDrag _lastImuDrag;

        [SerializeField, ReadOnly] private float _latestTwistDegrees;
        [SerializeField, ReadOnly] private Vector2 _latestRelativeImu;
        [SerializeField, ReadOnly] private float _dragMagnitude;
        [SerializeField, ReadOnly] private Vector3 _dragDeltaLocal;

        protected void Awake() => this.Ensure(ref _target, defaultType: typeof(Target));

        private void OnEnable() => _target.WhenSelecting += HandleSelecting;
        private void OnDisable() => _target.WhenSelecting -= HandleSelecting;

        private void HandleSelecting(ISelection selection)
        {
            var drag = selection.Drag;
            _lastCvDrag = drag as Drag;
            _lastImuDrag = drag as ImuDrag;

            _event.Drag = drag;
            _latestRelativeImu = selection.Latest.ImuProjectedDeltaRadians;
            _latestTwistDegrees = selection.Latest.TwistDegrees;
            _event.Selection = selection;

            _dragDeltaLocal = _event.Drag.DeltaLocal;
            _dragMagnitude = _dragDeltaLocal.magnitude;

            if (selection.Phase.IsBegin())
            {
                _event.Phase = Deadzone;
                WhenDragMove(_event);
            }

            if (_event.Phase.IsEnded()) return;

            if (selection.Phase.IsUpdate())
            {
                if (_event.Phase.IsBegin()) _event.Phase = Update;
                if (_event.Phase.IsDeadzone() && _dragMagnitude > _deadzoneMeters) _event.Phase = Begin;
                WhenDragMove(_event);
                WhenTwistRadians(_latestTwistDegrees);
            }

            if (selection.Phase.IsSuccess() && _event.Phase.IsUpdate())
            {
                _event.Phase = Success;
                WhenDragMove(_event);
            }

            if (selection.Phase.IsCancel() && _event.Phase.IsUpdate())
            {
                _event.Phase = Cancel;
                WhenDragMove(_event);
            }
        }

        public void ForceReset()
        {
            _event.Phase = Deadzone;
            WhenDragMove(_event);
        }

        public void ForceSuccess()
        {
            _event.Phase = Success;
            WhenDragMove(_event);
        }

        public void ForceCancel()
        {
            if (_event.Phase.IsCancel()) return;
            _event.Phase = Cancel;
            WhenDragMove(_event);
        }
    }
}
