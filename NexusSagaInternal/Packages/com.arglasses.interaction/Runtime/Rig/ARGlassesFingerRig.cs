using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    [Serializable]
    [DefaultExecutionOrder(ExecutionOrder.ARGlassesFingerRig)]
    public class ARGlassesFingerRig : MonoBehaviour
    {
        [SerializeField] private HandFinger _finger;
        public HandFinger Finger
        {
            get => _finger;
            set => _finger = value;
        }

        [SerializeField, ReadOnly] private Selector _selector;
        public Selector Selector => _selector;
        [SerializeField, ReadOnly] private ARGlassesHandRig _ovrOvrHandRig;

        [SerializeField, ReadOnly] private bool _isPinching;
        [SerializeField, ReadOnly] private bool _wasPinching;

        [SerializeField] private Mechanic.PressDetector.Double _doublePress = new();
        public Mechanic.PressDetector.Double DoublePress => _doublePress;

        [SerializeField] private Mechanic.PressDetector.Short _shortPress = new();
        public Mechanic.PressDetector.Short ShortPress => _shortPress;

        [SerializeField] private Mechanic.PressDetector.Long _longPress = new();
        public Mechanic.PressDetector.Long LongPress => _longPress;

        public event Action<bool> WhenPinchChanged = delegate { };
        public event Action WhenPinchPressed = delegate { };
        public event Action WhenPinchReleased = delegate { };

        public event Action WhenDoublePinch = delegate { };
        public event Action WhenShortPinch = delegate { };
        public event Action WhenLongPinch = delegate { };

        public bool IsDoublePinching => _doublePress.IsDoublePressing;
        public bool IsShortPinching => _shortPress.IsShortPressing;
        public bool IsLongPinching => _longPress.IsLongPressing;

        private void Awake()
        {
            this.Ancestor(ref _ovrOvrHandRig);
            this.Scene(ref _selector);

            _doublePress.WhenDoublePress += () => WhenDoublePinch();
            _shortPress.WhenShortPress += () => WhenShortPinch();
            _longPress.WhenLongPress += () => WhenLongPinch();
        }

        public bool Sync(bool isPinching)
        {
            _wasPinching = _isPinching;
            _isPinching = isPinching;

            // make sure we emit DoublePinch first incase anyone needs to suppress a simultaneous ShortPinch
            _doublePress.Sync(_isPinching, _wasPinching);
            _shortPress.Sync(_isPinching, _wasPinching);
            _longPress.Sync(_isPinching, _wasPinching);

            // once all the higher level detectors have fired, emit regular press/release

            if (_isPinching && !_wasPinching) WhenPinchPressed();
            if (!_isPinching && _wasPinching) WhenPinchReleased();
            if (_isPinching != _wasPinching) WhenPinchChanged(_isPinching);

            return _isPinching;
        }
    }
}
