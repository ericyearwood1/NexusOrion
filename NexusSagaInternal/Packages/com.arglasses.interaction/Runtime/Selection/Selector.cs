using System;
using UnityEngine;
using static ARGlasses.Interaction.Snapshot;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.HoverSelectionEvents)]
    public class Selector : MonoBehaviour
    {
        [SerializeField, ReadOnly] private ARGlassesRig _rig;
        public ARGlassesRig Rig => _rig;

        private InputContext _pendingInputContext;
        [SerializeField, ReadOnly] private InputContext _input;

        public InputContext InputContext
        {
            get => _input;
            set => _pendingInputContext = value;
        }

        private TargetContext _pendingTargetContext;
        [SerializeField, ReadOnly] private TargetContext _target;

        public TargetContext TargetContext
        {
            get => _target;
            set => _pendingTargetContext = value;
        }

        public Vector3 CursorWorld => _input ? _input.CursorWorld : default;

        public event Action<InputContext, TargetContext> WhenContextChanged = delegate { };
        public event Action<InputCategory> WhenInputCategoryChanged = delegate { };

        public InputCategory Category => _input != null ? _input.Category : InputCategory.None;

        public bool HasHoverTarget => _hoverBegin.HasTarget;
        public Target Hovered => _hoverBegin.Target;
        public bool HasSelectionTarget => _selectedBegin.HasTarget;
        public Target Selected => _selectedBegin.Target;

        [SerializeField, ReadOnly] private Focus _hoverBegin;
        [SerializeField, ReadOnly] private Focus _selectedBegin;

        [SerializeField, ReadOnly] private InteractionStateSequence _sequence = new();
        public InteractionStateSequence Sequence => _sequence;

        [SerializeField, ReadOnly] private Rig _rigSnapshot;
        public Rig GetSnapshot() => _rigSnapshot;

        void Awake()
        {
            this.Ancestor(ref _rig);
        }

        [SerializeField, ReadOnly] private InputCategory _lastCategory;

        void Update()
        {
            _rigSnapshot = _rig.Snapshot();

            var pendingInput = _pendingInputContext;
            var pendingTargeting = _pendingTargetContext;

            if (_input != pendingInput || pendingTargeting != _target)
            {
                if (_input != null) _input.Close(_rigSnapshot, _target);

                _input = pendingInput;
                _target = pendingTargeting;
                _sequence.Reset(_rigSnapshot);

                if (_input != null && _target != null)
                {
                    _input.Open(_rigSnapshot, pendingTargeting);
                    if (_input.Category != _lastCategory)
                    {
                        WhenInputCategoryChanged(_input.Category);
                        _lastCategory = _input.Category;
                    }
                }

                WhenContextChanged(_input, _target);
            }

            if (_input == null || _target == null)
            {
                _sequence.Reset(_rigSnapshot);
                return;
            }

            var focus = _input.GetFocus(_rigSnapshot);
            var latest = new InteractionState(_rigSnapshot, focus);
            _sequence.SetLatest(latest);

            PreRoute();
            UpdateNoFocus();
            UpdateHoverFocus();
            UpdateSelectionFocus();
            PostRoute();
        }

        public bool IsPinching(Side side, HandFinger finger) => _input != null && _input.IsPinching(_rigSnapshot, side, finger);

        public event Action PreRoute = delegate { };
        public event Action PostRoute = delegate { };
        public event Action<ISelection> WhenSelecting = delegate { };
        public event Action<IHover> WhenHovering = delegate { };
        public event Action<InteractionStateSequence> WhenNoFocus = delegate { };

        [SerializeField, ReadOnly] private SelectorEvent _lastSelectorEvent;
        public SelectorEvent LastSelectorEvent => _lastSelectorEvent;

        private void EmitSelecting(SelectionPhase phase, InteractionStateSequence sequence)
        {
            var focus = sequence.Begin.Focus;
            var target = focus.Target;
            Selection selection = new Selection(focus, phase, sequence, _input);
            var route = new SelectorEvent(this, selection);

            try
            {
                target.HandleSelectorEvent(route);
                WhenSelecting(selection);
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }

            _lastSelectorEvent = route;
        }

        [SerializeField, ReadOnly] private Focus _lastHoverFocus;
        private void EmitHovering(HoverPhase phase, InteractionState interactionState)
        {
            // before gaze exits a target, need to end hovering with a Focus that includes the Target
            var focus = phase.IsEnd() ? _lastHoverFocus : interactionState.Focus;
            var target = focus.Target;
            Hover hover = new Hover(focus, phase, interactionState, _input);
            var route = new SelectorEvent(this, hover);

            try
            {
                target.HandleSelectorEvent(route);
                WhenHovering(hover);
            }
            catch (Exception e)
            {
                Debug.LogError(e);
            }

            _lastHoverFocus = focus;
            _lastSelectorEvent = route;
        }

        private void EmitNoFocus()
        {
            WhenNoFocus(_sequence);
        }

        private void UpdateNoFocus()
        {
            if (HasSelectionTarget) return;
            if (HasHoverTarget) return;
            if (_sequence.Focus.HasTarget) // hover begin
            {
                _hoverBegin = _sequence.Focus;
                EmitHovering(HoverPhase.Begin, _sequence.Latest);
                return;
            }

            EmitNoFocus();
        }

        private void UpdateHoverFocus()
        {
            if (HasSelectionTarget) return;
            if (!HasHoverTarget) return;

            if (_sequence.Focus.Target != _hoverBegin.Target) // hover end
            {
                EmitHovering(HoverPhase.End, _sequence.Latest);
                _hoverBegin = Focus.Empty();
                return;
            }

            if (_sequence.IsPressing) // select begin
            {
                _sequence.SetBegin();
                _selectedBegin = _sequence.Focus;
                EmitSelecting(SelectionPhase.Begin, _sequence);
                return;
            }

            EmitHovering(HoverPhase.Update, _sequence.Latest);
        }

        private void UpdateSelectionFocus()
        {
            if (!HasSelectionTarget) return;
            if (!_sequence.IsPressing) // select ended
            {
                EmitSelecting(SelectionPhase.Success, _sequence);
                _selectedBegin = Focus.Empty();
                return;
            }

            EmitSelecting(SelectionPhase.Update, _sequence);
        }
    }
}
