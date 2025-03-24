using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public struct InteractionStateSequence
    {
        [SerializeField, ReadOnly] private InteractionState _latest;
        public InteractionState Latest => _latest;

        [SerializeField, ReadOnly] private InteractionState _previous;
        public InteractionState Previous => _previous;

        [SerializeField, ReadOnly] private InteractionState _begin;
        public InteractionState Begin => _begin;

        public Side PrimarySide => Latest.PrimarySide;
        public float Duration => Latest.TimeStamp - Begin.TimeStamp;
        public Snapshot.Focus Focus => Latest.Focus;
        public bool IsPressing => Latest.IsPressingPrimary;

        public void Reset(Snapshot.Rig rig) => Reset(new InteractionState(rig));
        public void Reset(InteractionState state)  {
            // _begin = _previous = _latest = state;
        }

        public void SetLatest(InteractionState state)
        {
            _previous = _latest;
            _latest = state;
        }

        public void SetBegin()
        {
            _begin = _latest;
        }
    }
}
