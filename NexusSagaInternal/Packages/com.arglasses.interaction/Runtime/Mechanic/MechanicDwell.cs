using System;
using UnityEngine;
using UnityEngine.Serialization;
using static ARGlasses.Interaction.Mechanic.Dwell;
using static ARGlasses.Interaction.Mechanic.Dwell.Phase;

namespace ARGlasses.Interaction
{
    public static partial class ExtensionsMechanic
    {
        public static bool IsNone(this Phase phase) => phase == None;
        public static bool IsPreExecute(this Phase phase) => phase == PreExecute;
        public static bool IsExecute(this Phase phase) => phase == Execute;
        public static bool IsPostExecute(this Phase phase) => phase == PostExecute;
        public static bool IsEnded(this Phase phase) => phase.IsNone();
    }

    public static partial class Mechanic
    {
        public static class Dwell
        {
            [Serializable]
            public struct Event
            {
                public Phase Phase;
                public float Progress;
            }

            public enum Phase
            {
                None,
                PreExecute,
                Execute,
                PostExecute,
            }
        }
    }

    public class MechanicDwell : MonoBehaviour
    {
        public event Action WhenDwellBegin = delegate { };
        public event Action WhenDwellEnd = delegate { };
        public float Progress => _event.Progress;

        public event Action<Mechanic.Dwell.Event> WhenDwell = delegate { };
        [SerializeField, ReadOnly] private Mechanic.Dwell.Event _event;
        private Mechanic.Dwell.Event Event => _event;

        [SerializeField] private Selectable _target;
        public Selectable Target => _target;

        [SerializeField] private float _requiredDwellTime = 1.0f;

        private void Awake() => this.Ensure(ref _target, relation: Relation.Descendant, defaultType: typeof(Target));
        private void OnEnable() => _target.WhenStateChanged += HandleStateChanged;
        private void OnDisable() => _target.WhenStateChanged -= HandleStateChanged;

        private void HandleStateChanged(TargetState state)
        {
            Debug.Log(state);

            if (state.IsNormal())
            {
                if (_event.Phase.IsPostExecute()) WhenDwellEnd();

                _event.Phase = None;
                _event.Progress = 0;
                WhenDwell(_event);
            }

            if (state.IsHover())
            {
                _event.Phase = PreExecute;
                _event.Progress = 0;
                WhenDwell(_event);
            }
        }

        void Update()
        {
            if (_event.Phase.IsPreExecute())
            {
                _event.Progress += Time.deltaTime / _requiredDwellTime;
                WhenDwell(_event);

                if (_event.Progress > _requiredDwellTime)
                {
                    _event.Progress = 1;
                    _event.Phase = Execute;
                    WhenDwell(_event);
                    WhenDwellBegin();
                    _event.Phase = PostExecute;
                }
            }

            if (_event.Phase.IsPostExecute()) WhenDwell(_event);
        }
    }
}
