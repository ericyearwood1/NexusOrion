using System;
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Interaction
{
    public class InputContextListener : MonoBehaviour
    {
        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private InputContext _context;
        [SerializeField, ReadOnly] private TargetContext _surface;

        public InputContext InputContext => _context;
        public TargetContext TargetContext => _surface;
        public InputCategory InputContextCategory => !_context ? default : _context.Category;

        public UnityEvent<InputContext, TargetContext> WhenContextChanged = new();

        private void Start()
        {
            this.Scene(ref _selector);
            _selector.WhenContextChanged += HandleContextChanged;
        }

        private void HandleContextChanged(InputContext context, TargetContext surface)
        {
            _context = context;
            _surface = surface;
            WhenContextChanged.Invoke(context, surface);
        }
    }
}
