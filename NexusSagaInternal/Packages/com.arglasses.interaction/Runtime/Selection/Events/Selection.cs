using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class Selection : ISelection
    {
        [SerializeField, ReadOnly] protected Snapshot.Focus _focus;
        [SerializeField, ReadOnly] protected SelectionPhase _phase;
        [SerializeField, ReadOnly] protected InteractionStateSequence _sequence;
        [SerializeField, ReadOnly] protected Drag _drag;
        [SerializeField, ReadOnly] protected Drag _dragThisFrame;
        [SerializeField, ReadOnly] protected InputContext _context;

        public Selection(ISelection source) : this(source.Focus, source.Phase, source.Sequence, source.Context)
        {
        }

        public Selection(Snapshot.Focus focus, SelectionPhase phase, InteractionStateSequence sequence, InputContext context)
        {
            _focus = focus;
            _context = context;
            _phase = phase;
            _sequence = sequence;
            _drag = context.CreateDrag(_sequence.Latest, _sequence.Begin);
            _dragThisFrame = context.CreateDrag(_sequence.Latest, _sequence.Previous);
        }

        public virtual Snapshot.Focus Focus => _focus;
        public virtual SelectionPhase Phase => _phase;
        public virtual Side PrimarySide => _sequence.Latest.PrimarySide;
        public virtual float Duration => _sequence.Duration;
        public virtual InteractionState Latest => _sequence.Latest;
        public virtual InteractionState Begin => _sequence.Begin;
        public virtual InteractionState Previous => _sequence.Previous;
        public virtual InteractionStateSequence Sequence => _sequence;
        public virtual IDrag Drag => _drag;
        public virtual IDrag DragThisFrame => _dragThisFrame;
        public virtual InputContext Context => _context;
    }
}
