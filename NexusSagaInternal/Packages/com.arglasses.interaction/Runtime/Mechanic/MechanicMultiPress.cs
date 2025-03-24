using System;
using UnityEngine;
using UnityEngine.Serialization;
using static ARGlasses.Interaction.Mechanic.MultiPress;
using static ARGlasses.Interaction.Mechanic.MultiPress.Phase;

namespace ARGlasses.Interaction
{
    public static partial class ExtensionsMechanic
    {
        public static bool IsNone(this Phase phase) => phase == None;
        public static bool IsFirstBegin(this Phase phase) => phase == FirstBegin;
        public static bool IsFirstHold(this Phase phase) => phase == FirstHold;
        public static bool IsFirstReleased(this Phase phase) => phase == FirstReleased;

        public static bool IsExecuteSingle(this Phase phase) => phase == ExecuteSingleTap;
        public static bool IsExecuteDouble(this Phase phase) => phase == ExecuteDoubleTap;

        public static bool IsSecondBegin(this Phase phase) => phase == SecondBegin;
        public static bool IsSecondHold(this Phase phase) => phase == SecondHold;
        public static bool IsSecondReleased(this Phase phase) => phase == SecondReleased;

        public static bool IsExecuteAny(this Phase phase) => phase.IsExecuteSingle() || phase.IsExecuteDouble();
        public static bool IsCancel(this Phase phase) => phase == Cancel;
        public static bool IsEnded(this Phase phase) => phase.IsCancel() || phase.IsSecondReleased() || phase.IsNone();
    }

    public static partial class Mechanic
    {
        public static class MultiPress
        {
            [Serializable]
            public struct Event
            {
                public Phase Phase;
                public ISelection FirstSelection;
                public ISelection SecondSelection;
            }

            public enum Phase
            {
                None,
                FirstBegin,
                FirstHold,
                FirstReleased,

                SecondBegin,
                SecondHold,
                SecondReleased,

                ExecuteSingleTap,
                ExecuteDoubleTap,
                Cancel,
            }
        }
    }

    public class MechanicMultiPress : MonoBehaviour
    {
        public event Action<Mechanic.MultiPress.Event> WhenSinglePressExecute = delegate {  };
        public event Action<Mechanic.MultiPress.Event> WhenDoublePressExecute = delegate {  };
        public event Action<Mechanic.MultiPress.Event> WhenCancel = delegate {  };

        public event Action<Mechanic.MultiPress.Event> WhenMultiPress = delegate { };
        [SerializeField, ReadOnly] private Mechanic.MultiPress.Event _event;
        private Mechanic.MultiPress.Event Event => _event;

        [SerializeField] private float _maxInterval = 0.4f;
        public float MaxInterval => _maxInterval;

        [SerializeField, ReadOnly] private Selectable _target;
        public Selectable Target => this.Ensure(ref _target, defaultType: typeof(Target));

        protected void Awake() => this.Ensure(ref _target, defaultType: typeof(Target));
        private void OnEnable() => _target.WhenSelecting += HandleSelecting;
        private void OnDisable() => _target.WhenSelecting -= HandleSelecting;

        private void HandleSelecting(ISelection selection)
        {
            if (selection.Phase.IsBegin())
            {
                if (_event.Phase.IsFirstReleased())
                {
                    _event.SecondSelection = selection;
                    _event.Phase = ExecuteDoubleTap;
                    WhenMultiPress(_event);
                    WhenDoublePressExecute(_event);

                    _event.Phase = SecondBegin;
                    WhenMultiPress(_event);
                }
                else
                {
                    _event.FirstSelection = selection;
                    _event.SecondSelection = null;
                    _event.Phase = FirstBegin;
                    WhenMultiPress(_event);
                }
            }

            if (selection.Phase.IsUpdate())
            {
                if (_event.Phase.IsFirstBegin()) _event.Phase = FirstHold;
                if (_event.Phase.IsSecondBegin()) _event.Phase = SecondHold;
                WhenMultiPress(_event);
            }

            if (selection.Phase.IsSuccess())
            {
                if (_event.Phase.IsFirstHold()) _event.Phase = FirstReleased;
                if (_event.Phase.IsSecondHold()) _event.Phase = SecondReleased;
                WhenMultiPress(_event);
            }

            if (selection.Phase.IsCancel())
            {
                _event.Phase = Cancel;
                WhenMultiPress(_event);
                WhenCancel(_event);
            }
        }

        public void ForceCancel()
        {
            if (_event.Phase.IsCancel()) return;
            _event.Phase = Cancel;
            WhenMultiPress(_event);
            WhenCancel(_event);
        }

        private void Update()
        {
            TryExecuteSingleTap();
        }

        private void TryExecuteSingleTap()
        {
            if (!_event.Phase.IsFirstReleased()) return;

            var interval = Time.time - _event.FirstSelection.Begin.TimeStamp;
            if (interval > _maxInterval)
            {
                // we exceeded the timeout for a double press
                // fire off the original single press and clear our state
                _event.Phase = ExecuteSingleTap;
                WhenMultiPress(_event);
                WhenSinglePressExecute(_event);
                _event.Phase = None;
            }
        }
    }
}
