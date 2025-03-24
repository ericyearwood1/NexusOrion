using System;
using UnityEngine;
using static ARGlasses.Interaction.Mechanic.TwoHanded;
using static ARGlasses.Interaction.Mechanic.TwoHanded.Phase;

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
        public static class TwoHanded
        {
            [Serializable]
            public struct Event
            {
                public Phase Phase;
                public IDrag LeftDrag;
                public IDrag RightDrag;
                public Pose Center;
                public Pose CenterBegin;
                public Pose CenterDelta => Center.Subtract(CenterBegin);
                public float Distance;
                public float DistanceBegin;
                public float DistanceDelta => Distance - DistanceBegin;
                public float Scale => Mathf.Max(Distance / Mathf.Max(DistanceBegin, 0.002f), 0.002f);
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
    public class MechanicTwoHanded : MonoBehaviour
    {
        [SerializeField] private Selectable _target;
        public Selectable Target => _target;

        public event Action<Mechanic.TwoHanded.Event> WhenTwoHanded = delegate { };
        public event Action<Mechanic.TwoHanded.Event> WhenTwoHandedBegin = delegate { };
        public event Action<Mechanic.TwoHanded.Event> WhenTwoHandedEnd = delegate { };

        [SerializeField, ReadOnly] private Mechanic.TwoHanded.Event _event;
        private Mechanic.TwoHanded.Event Event => _event;

        protected void Awake()
        {
            this.Ensure(ref _target, defaultType: typeof(Target));
        }

        private void OnEnable()
        {
            _target.WhenSelecting += HandleSelecting;
        }

        private void OnDisable()
        {
            _target.WhenSelecting -= HandleSelecting;
        }

        private void HandleSelecting(ISelection selection)
        {
            _event.Selection = selection;

            var latest = selection.Latest;
            var begin = selection.Begin;

            var target = selection.Target;
            var leftFocus = new Snapshot.Focus(target, Side.Left, latest.IsPressingPrimary, latest.IsPressingSecondary);
            var leftLatest = new InteractionState(latest, leftFocus);
            var leftBegin = new InteractionState(begin, leftFocus);
            _event.LeftDrag = selection.Context.CreateDrag(leftLatest, leftBegin);

            var rightFocus = new Snapshot.Focus(target, Side.Right, latest.IsPressingPrimary, latest.IsPressingSecondary);
            var rightLatest = new InteractionState(latest, rightFocus);
            var rightBegin = new InteractionState(begin, rightFocus);
            _event.RightDrag = selection.Context.CreateDrag(rightLatest, rightBegin);

            var leftToRight =  rightLatest.Pinch.position - leftLatest.Pinch.position;
            _event.Distance = leftToRight.magnitude;
            _event.Center.position = (leftLatest.Pinch.position + rightLatest.Pinch.position) * 0.5f;
            _event.Center.rotation = Quaternion.LookRotation(Vector3.Cross(leftToRight.normalized, Vector3.up));

            Debug.DrawLine(_event.Center.position, _event.Center.position + _event.Center.forward, Color.blue);

            if (!selection.Latest.PrimarySide.IsBoth())
            {
                if (_event.Phase.IsEnded()) return;
                _event.Phase = Success;
                WhenTwoHandedEnd(_event);
                WhenTwoHanded(_event);
                return;
            }

            if (_event.Phase.IsEnded())
            {
                _event.Phase = Deadzone;
                _event.CenterBegin = _event.Center;
                _event.DistanceBegin = _event.Distance;
                WhenTwoHanded(_event);
                return;
            }

            if (_event.Phase.IsDeadzone())
            {
                _event.CenterBegin = _event.Center;
                _event.DistanceBegin = _event.Distance;

                const float deadzone = 0.04f;
                if (_event.LeftDrag.Magnitude < deadzone && _event.RightDrag.Magnitude < deadzone)
                {
                    WhenTwoHanded(_event);
                    return;
                }

                _event.Phase = Begin;
                WhenTwoHandedBegin(_event);
                WhenTwoHanded(_event);
                _event.Phase = Update;
                return;
            }

            if (_event.Phase.IsUpdate())
            {
                WhenTwoHanded(_event);
                return;
            }
        }
    }
}
