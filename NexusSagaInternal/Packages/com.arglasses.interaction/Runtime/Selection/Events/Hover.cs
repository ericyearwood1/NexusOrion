using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    public class Hover : IHover
    {
        [SerializeField, ReadOnly] private Snapshot.Focus _focus;
        [SerializeField, ReadOnly] private HoverPhase _phase;
        [SerializeField, ReadOnly] private InteractionState _interactionState;
        [SerializeField, ReadOnly] private InputContext _context;

        public Hover(Snapshot.Focus focus, HoverPhase phase, InteractionState interactionState, InputContext context)
        {
            _focus = focus;
            _phase = phase;
            _interactionState = interactionState;
            _context = context;
        }

        public Snapshot.Focus Focus => _focus;
        public HoverPhase Phase => _phase;
        public InteractionState State => _interactionState;
        public InputContext Context => _context;
    }
}
