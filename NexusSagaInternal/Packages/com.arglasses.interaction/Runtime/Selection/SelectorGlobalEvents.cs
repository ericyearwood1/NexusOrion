using System;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SelectorGlobalEvents : MonoBehaviour
    {
        [Serializable]
        public class HandEvents
        {
            [SerializeField, ReadOnly] private FingerEvents _index;
            [SerializeField, ReadOnly] private FingerEvents _middle;

            public FingerEvents Index => _index;
            public FingerEvents Middle => _middle;

            public HandEvents(Side side)
            {
                _index = new FingerEvents(side, HandFinger.Index);
                _middle = new FingerEvents(side, HandFinger.Middle);
            }
        }

        [Serializable]
        public class FingerEvents
        {
            [SerializeField, ReadOnly] private Side _side;
            public Side Side => _side;

            [SerializeField, ReadOnly] private HandFinger _finger;
            public HandFinger Finger => _finger;

            [SerializeField, ReadOnly] private bool _isPinching;
            [SerializeField, ReadOnly] private bool _wasPinching;

            [SerializeField] private Mechanic.PressDetector.Double _doublePress = new();
            public Mechanic.PressDetector.Double DoublePress => _doublePress;

            [SerializeField] private Mechanic.PressDetector.Short _shortPress = new();
            public Mechanic.PressDetector.Short ShortPress => _shortPress;

            [SerializeField] private Mechanic.PressDetector.Long _longPress = new();

            public FingerEvents(Side side, HandFinger finger)
            {
                _side = side;
                _finger = finger;
                _doublePress.WhenDoublePress += () => WhenDoublePinch?.Invoke();
                _shortPress.WhenShortPress += () => WhenShortPinch?.Invoke();
                _longPress.WhenLongPress += () => WhenLongPinch?.Invoke();
            }

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

            public bool Sync(bool isPinching)
            {
                _wasPinching = _isPinching;
                _isPinching = isPinching;

                // make sure we emit DoublePinch first incase anyone needs to suppress a simultaneous ShortPinch
                _doublePress.Sync(_isPinching, _wasPinching);
                _shortPress.Sync(_isPinching, _wasPinching);
                _longPress.Sync(_isPinching, _wasPinching);

                // once all the higher level detectors have fired, emit regular press/release
                if (_isPinching && !_wasPinching) WhenPinchPressed?.Invoke();
                if (!_isPinching && _wasPinching) WhenPinchReleased?.Invoke();
                if (_isPinching != _wasPinching) WhenPinchChanged?.Invoke(_isPinching);

                return _isPinching;
            }
        }

        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;

        [SerializeField, ReadOnly] private HandEvents _left;
        [SerializeField, ReadOnly] private HandEvents _right;
        public event Action<DPad> WhenDPad = delegate {  };
        public event Action<DiscreteHandMotion> WhenDiscreteHandMotion = delegate {  };

        public Selector Selector => _selector;

        private bool _initialized;
        private HandEvents EnsureInitialized(Side side)
        {
            if (!_initialized) Initialize();
            return side.IsLeft() ? _left : side.IsRight() ? _right : null;
        }

        private void Initialize()
        {
            _initialized = true;
            _left = new HandEvents(Side.Left);
            _right = new HandEvents(Side.Right);
        }

        public HandEvents Left => EnsureInitialized(Side.Left);
        public HandEvents Right => EnsureInitialized(Side.Right);

        private void Awake()
        {
            this.Sibling(ref _selector);
            this.Scene(ref _wristband);
            Initialize();
        }

        private void OnEnable()
        {
            _selector.PreRoute += HandlePreRoute;
            _wristband.WhenDPad += HandleDPad;
            _wristband.WhenDiscreteHandMotion += HandleDiscreteHandMotion;
        }


        private void OnDisable()
        {
            _selector.PreRoute -= HandlePreRoute;
            _wristband.WhenDPad -= HandleDPad;
            _wristband.WhenDiscreteHandMotion -= HandleDiscreteHandMotion;
        }

        private void HandleDPad(DPad dPad) => WhenDPad(dPad);
        private void HandleDiscreteHandMotion(DiscreteHandMotion motion) => WhenDiscreteHandMotion(motion);

        private void HandlePreRoute()
        {
            _left.Index.Sync(_selector.IsPinching(Side.Left, HandFinger.Index));
            _left.Middle.Sync(_selector.IsPinching(Side.Left, HandFinger.Middle));
            _right.Index.Sync(_selector.IsPinching(Side.Right, HandFinger.Index));
            _right.Middle.Sync(_selector.IsPinching(Side.Right, HandFinger.Middle));
        }
    }
}
