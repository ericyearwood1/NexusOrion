using System;
using UnityEngine;
using UnityEngine.Serialization;
using static ARGlasses.Interaction.Mechanic.LongPress;
using static ARGlasses.Interaction.Mechanic.LongPress.Phase;

namespace ARGlasses.Interaction
{
    public static partial class ExtensionsMechanic
    {
        public static bool IsNone(this Phase phase) => phase == None;
        public static bool IsPreExecute(this Phase phase) => phase == PreExecute;
        public static bool IsExecute(this Phase phase) => phase == Execute;
        public static bool IsPostExecute(this Phase phase) => phase == PostExecute;

        public static bool IsSuccess(this Phase phase) => phase == Success;
        public static bool IsCancel(this Phase phase) => phase == Cancel;

        public static bool IsPressing(this Phase phase) => phase.IsPreExecute() || phase.IsExecute() || phase.IsPostExecute();
        public static bool IsLongPressing(this Phase phase) => phase.IsExecute() || phase.IsPostExecute();
        public static bool IsEnded(this Phase phase) => phase.IsSuccess() || phase.IsCancel() || phase.IsNone();
    }

    public static partial class Mechanic
    {
        public static class LongPress
        {
            [Serializable]
            public struct Event
            {
                public Phase Phase;
                public float Progress;
                public ISelection Selection;
            }

            public enum Phase
            {
                None,
                PreExecute,
                Execute,
                PostExecute,
                Success,
                Cancel,
            }
        }
    }

    public class MechanicLongPress : MonoBehaviour
    {
        public event Action<Mechanic.LongPress.Event> WhenLongPress = delegate { };
        [SerializeField, ReadOnly] private Mechanic.LongPress.Event _event;
        private Mechanic.LongPress.Event Event => _event;

        public event Action<Mechanic.LongPress.Event> WhenPressBegin = delegate { };
        public event Action<Mechanic.LongPress.Event> WhenPressEnd = delegate { };
        public event Action<Mechanic.LongPress.Event> WhenPressExecute = delegate {  };
        public event Action<Mechanic.LongPress.Event> WhenPressCancel = delegate { };
        public bool IsLongPressing => _event.Phase.IsLongPressing();

        [SerializeField] private float _requiredDuration = 1.0f;
        public float RequiredDuration
        {
            get => _requiredDuration;
            set => _requiredDuration = value;
        }

        [SerializeField, ReadOnly] private Selectable _target;
        public Selectable Target => this.Ensure(ref _target, defaultType: typeof(Target));

        protected void Awake() => this.Ensure(ref _target, defaultType: typeof(Target));
        private void OnEnable() => _target.WhenSelecting += HandleSelecting;
        private void OnDisable() => _target.WhenSelecting -= HandleSelecting;

        private void HandleSelecting(ISelection selection)
        {
            _event.Selection = selection;
            if (selection.Phase.IsBegin()) SelectionBegin();
            if (selection.Phase.IsUpdate()) Selecting(selection);
            if (selection.Phase.IsEnded()) SelectionEnd();
        }

        private void SelectionBegin()
        {
            _event.Phase = PreExecute;
            _event.Progress = 0;
            WhenLongPress(_event);
            WhenPressBegin(_event);
        }

        private void Selecting(ISelection selection)
        {
            if (_event.Phase.IsPreExecute())
            {
                _event.Progress = selection.Duration / _requiredDuration;
                WhenLongPress(_event);

                if (_event.Progress >= 1)
                {
                    _event.Progress = 1;
                    _event.Phase = Execute;
                    WhenLongPress(_event);
                    WhenPressExecute(_event);
                    _event.Phase = PostExecute;
                }
                return;
            }

            if (_event.Phase.IsPostExecute()) WhenLongPress(_event);
        }

        private void SelectionEnd()
        {
            if (_event.Phase.IsPreExecute())
            {
                _event.Phase = Cancel;
                WhenLongPress(_event);
                WhenPressCancel(_event);
            }

            if (_event.Phase.IsPostExecute())
            {
                _event.Phase = Success;
                WhenLongPress(_event);
            }

            WhenPressEnd(_event);
        }

        public void ForceSuccess()
        {
            _event.Phase = Success;
            WhenLongPress(_event);
        }

        public void ForceCancel()
        {
            if (_event.Phase.IsCancel()) return;
            _event.Phase = Cancel;
            WhenLongPress(_event);
        }
    }
}
