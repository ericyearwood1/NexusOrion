using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ShellInputEvents : MonoBehaviour
    {
        public KeyCode _editorToggleWakeKey = KeyCode.End;
        public KeyCode _editorToggleLaunchKey = KeyCode.Home;
        public bool _debugLogs = true;

        [SerializeField, ReadOnly] private WakeState _wakeState;
        public event Action<WakeState> WhenWakeStateChanged = delegate {  };

        [SerializeField, ReadOnly] private PersonalSpaceState _personalSpaceState;
        public event Action<PersonalSpaceState> WhenPersonalSpaceStateChanged = delegate {  };

        [SerializeField, ReadOnly] private SelectorGlobalEvents _globalEvents;

        private void Awake() => this.Scene(ref _globalEvents);

        private void OnEnable()
        {
            _globalEvents.Right.Middle.WhenShortPinch += TogglePersonalSpace;
            _globalEvents.Right.Middle.WhenDoublePinch += ToggleWake;
        }

        private void OnDisable()
        {
            _globalEvents.Right.Middle.WhenShortPinch -= TogglePersonalSpace;
            _globalEvents.Right.Middle.WhenDoublePinch -= ToggleWake;
        }

        public bool IsWakeActive
        {
            get => WakeState.IsAwake();
            set => WakeState = value ? WakeState.Awake : WakeState.Sleep;
        }

        public WakeState WakeState
        {
            get => _wakeState;
            set
            {
                if (_wakeState == value) return;
                _wakeState = value;
                if (PersonalSpaceState.IsActive()) PersonalSpaceState = PersonalSpaceState.Inactive;
                WhenWakeStateChanged(_wakeState);
            }
        }

        public PersonalSpaceState PersonalSpaceState
        {
            get => _personalSpaceState;
            set
            {
                if (!IsWakeActive || _personalSpaceState == value) return;
                _personalSpaceState = value;
                WhenPersonalSpaceStateChanged(_personalSpaceState);
            }
        }

        public void ToggleWake() => IsWakeActive = !IsWakeActive;
        public void TogglePersonalSpace() => PersonalSpaceState = PersonalSpaceState.Toggle();

        private void Update()
        {
            if (Input.GetKeyDown(_editorToggleWakeKey)) ToggleWake();
            if (Input.GetKeyDown(_editorToggleLaunchKey)) TogglePersonalSpace();
        }
    }
}
