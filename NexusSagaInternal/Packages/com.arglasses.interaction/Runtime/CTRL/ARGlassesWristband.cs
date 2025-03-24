// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using CTRL;
using CTRL.Data;
using CTRL.Schemes;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;

namespace ARGlasses.Interaction
{
    [DefaultExecutionOrder(ExecutionOrder.ARGlassesWristband)]
    public class ARGlassesWristband : MonoBehaviour
    {
        [SerializeField, ReadOnly] private CTRLClient _ctrlr;
        [SerializeField, ReadOnly] private UIEvents _uiEvents;
        [SerializeField, ReadOnly] private ARGlassesWristbandStreams _streams;
        public ARGlassesWristbandStreams Streams => this.Descendant(ref _streams);

        [SerializeField, ReadOnly] private ARGlassesWristbandPushPull _wristbandPushPull;
        public ARGlassesWristbandPushPull WristbandPushPull => this.Descendant(ref _wristbandPushPull);

        [SerializeField, ReadOnly] private ARGlassesWristbandTwist _wristbandTwist;
        public ARGlassesWristbandTwist ARGlassesWristbandTwist => this.Descendant(ref _wristbandTwist);

        [SerializeField, ReadOnly] private bool _logEvents;
        [SerializeField, ReadOnly] private float _isDataStreamingThreshold = 0.3f;

        [SerializeField, ReadOnly] private Side _side = Side.Right;
        public Side Side => _side;

        public bool IsPinching(HandFinger digit)
        {
            if (digit.IsIndex()) return _isPinchingIndex;
            if (digit.IsMiddle()) return _isPinchingMiddle;
            return false;
        }

        public bool IsConnected => _isConnected;
        public event Action<bool> WhenConnectionChanged = delegate { };
        public bool IsDataStreaming => _isDataStreaming;
        public event Action<bool> WhenDataStreamingChanged;

        public event Action<HandFinger, bool> WhenPinch = delegate { };
        public event Action<DPad> WhenDPad = delegate { };
        public event Action<DiscreteHandMotion> WhenDiscreteHandMotion = delegate { };

        public Quaternion AbsoluteOrientation => Streams.AbsoluteOrientation;
        public Quaternion UserOrientation => Streams.UserOrientation;

        public Vector2 ProjectedRadians => Streams.ProjectedRadians;
        public Vector3 PitchedAcceleration => Streams.PitchedAcceleration;

        public float Battery => _battery;

        [SerializeField, ReadOnly] private bool _isPinchingIndex;
        [SerializeField, ReadOnly] private bool _isPinchingMiddle;
        private float _lastBroadcastPress = float.MinValue;
        private float _minPressThreshold = 0.1f;

        [SerializeField] private MouseKeyboardEmulator.EFakePinchMouseButton _fakeIndexMouse = MouseKeyboardEmulator.EFakePinchMouseButton.LeftClick;
        [SerializeField] private MouseKeyboardEmulator.EFakePinchMouseButton _fakeMiddleMouse = MouseKeyboardEmulator.EFakePinchMouseButton.RightClick;
        [SerializeField] private KeyCode fakeIndexKey = KeyCode.M;
        [SerializeField] private KeyCode fakeMiddleKey = KeyCode.Comma;
        [SerializeField] private KeyCode fakeShakeKey = KeyCode.Slash;
        [SerializeField] private KeyCode _left = KeyCode.LeftArrow;
        [SerializeField] private KeyCode _rightKey = KeyCode.RightArrow;
        [SerializeField] private KeyCode _upKey = KeyCode.UpArrow;
        [SerializeField] private KeyCode _downKey = KeyCode.DownArrow;

        [SerializeField, ReadOnly] private float _battery;
        [SerializeField, ReadOnly] private bool _isConnected;

        [SerializeField, ReadOnly] private bool _isDataStreaming;
        [SerializeField, ReadOnly] private bool _wasDataStreaming;
        [SerializeField, ReadOnly] private float _isStreamingLastDataRecievedTime;
        [SerializeField, ReadOnly] private DPad _lastDPad;
        [SerializeField, ReadOnly] private DPad _pendingDPad;
        [SerializeField, ReadOnly] private DiscreteHandMotion _lastDiscreteHandMotion;
        [SerializeField, ReadOnly] private DiscreteHandMotion _pendingDiscreteHandMotion;

        private double _lastImuTimestamp;
        [SerializeField, ReadOnly] private int _lastSnapshotFrame;
        [SerializeField, ReadOnly] private Snapshot.Wristband _lastSnapshot;

        public event Action<bool> WhenCtrlClientEnabledChanged = delegate { };
        public bool CtrlClientEnabled
        {
            get => this.Descendant(ref _ctrlr).enabled;
            set
            {
                this.Descendant(ref _ctrlr);
                if (_ctrlr.enabled == value) return;

                _ctrlr.enabled = value;
                WhenCtrlClientEnabledChanged(value);
            }
        }

        protected void Awake()
        {
            this.Descendant(ref _ctrlr);
            this.Descendant(ref _uiEvents);
            this.Descendant(ref _streams);
            this.Descendant(ref _wristbandPushPull);
            this.Descendant(ref _wristbandTwist);

            _uiEvents.OnButtonState.AddListener(HandleButtonState);
            _uiEvents.OnButtonAction.AddListener(HandleButtonAction);
            _uiEvents.OnDPadAction.AddListener(HandleDPadAction);
            _wristbandPushPull.WhenDiscreteHandMotion += BroadcastDiscreteHandMotion;
        }

        public Snapshot.Wristband GetSnapshot()
        {
            // todo this is stinky
            if (_lastSnapshotFrame == Time.frameCount)
            {
                Debug.LogError("ARGlassesWristband.GetSnapshot accessed more than once in a frame");
                return _lastSnapshot;
            }

            _lastSnapshotFrame = Time.frameCount;

            var dPad = _pendingDPad;
            _pendingDPad = DPad.None;

            var discreteHandMotion = _pendingDiscreteHandMotion;
            _pendingDiscreteHandMotion = DiscreteHandMotion.None;

            var twistDegrees = _wristbandTwist.TwistDegrees;
            return _lastSnapshot = new Snapshot.Wristband(_isPinchingIndex, _isPinchingMiddle, dPad, discreteHandMotion, Streams.UserOrientation, Streams.ProjectedRadians, twistDegrees, IsConnected, Side);
        }

        private void Log(string msg)
        {
            if (_logEvents) Debug.Log(msg);
        }

        private void BroadcastDiscreteHandMotion(DiscreteHandMotion motion)
        {
            Log($"BroadcastDiscreteHandMotion: {motion} {Time.time}");
            _lastDiscreteHandMotion = motion;
            _pendingDiscreteHandMotion = motion;
            WhenDiscreteHandMotion.Invoke(_pendingDiscreteHandMotion);
        }

        private void BroadcastDPadAction(DPad dPad)
        {
            Log($"BroadcastDPadAction: {dPad} {Time.time}");
            _lastDPad = dPad;
            _pendingDPad = dPad;
            WhenDPad.Invoke(dPad);
        }

        private void BroadcastPress(HandFinger digit)
        {
            if (Time.time - _lastBroadcastPress < _minPressThreshold) return;
            _lastBroadcastPress = Time.time;

            if (digit.IsIndex()) _isPinchingIndex = true;
            if (digit.IsMiddle()) _isPinchingMiddle = true;
            WhenPinch?.Invoke(digit, true);
        }

        private void BroadcastRelease(HandFinger digit)
        {
            if (digit.IsIndex()) _isPinchingIndex = false;
            if (digit.IsMiddle()) _isPinchingMiddle = false;
            WhenPinch?.Invoke(digit, false);
        }

        void HandleButtonState(ButtonState s)
        {
            Log($"HandleButtonState {s.buttonName} {s.button} {s.timing} {Time.time}");
            if (s.button != Button.Primary && s.button != Button.Secondary) return;
            if (s.timing == Timing.Start) BroadcastPress(s.button == Button.Secondary ? HandFinger.Middle : HandFinger.Index);
            if (s.timing == Timing.End) BroadcastRelease(s.button == Button.Secondary ? HandFinger.Middle : HandFinger.Index);
        }

        void HandleButtonAction(ButtonAction a)
        {
            Log($"HandleButtonAction {a.buttonName} {a.button} {a.timing} {a.actionName} {a.action} {Time.time}");
        }

        private void HandleDPadAction(DPadAction a)
        {
            // if (connectedHand) connectedHand.CancelPinch(HandFinger.Index); // Need to cancel any active thumbpress pinches when DPad emits
            Log($"HandleDPadAction {a.actionName} {a.action} {Time.time}");
            BroadcastDPadAction(a.ToDPad());
        }

        // Should update before WristbandPinchProvider
        void Update()
        {
            CheckFakeFingerInput();

            if (_ctrlr.State == ConnectionState.Disconnected && _isConnected)
            {
                _isConnected = false;
                Log($"{nameof(ARGlassesWristband)}.Disconnected()");
                WhenConnectionChanged(_isConnected);
            }

            if (_ctrlr.State == ConnectionState.Connected && !_isConnected)
            {
                _isConnected = true;
                Log($"{nameof(ARGlassesWristband)}.Connected()");
                WhenConnectionChanged(_isConnected);
            }

            if (Streams.Timestamp <= _lastImuTimestamp) return;
            _lastImuTimestamp = Streams.Timestamp;

            _isDataStreaming = _isConnected && Time.time - _isStreamingLastDataRecievedTime < _isDataStreamingThreshold;
            if (_isDataStreaming != _wasDataStreaming)
            {
                WhenDataStreamingChanged?.Invoke(_isDataStreaming);
                _wasDataStreaming = _isDataStreaming;
            }

            _isStreamingLastDataRecievedTime = Time.time;
        }


        private void CheckFakeFingerInput()
        {
            if (Input.GetKeyDown(fakeIndexKey) || Input.GetMouseButtonDown((int)_fakeIndexMouse)) BroadcastPress(HandFinger.Index);
            if (Input.GetKeyUp(fakeIndexKey) || Input.GetMouseButtonUp((int)_fakeIndexMouse)) BroadcastRelease(HandFinger.Index);

            if (Input.GetKeyDown(fakeMiddleKey) || Input.GetMouseButtonDown((int)_fakeMiddleMouse)) BroadcastPress(HandFinger.Middle);
            if (Input.GetKeyUp(fakeMiddleKey) || Input.GetMouseButtonUp((int)_fakeMiddleMouse)) BroadcastRelease(HandFinger.Middle);

            if (Input.GetKeyDown(_left)) BroadcastDPadAction(DPad.Left);
            if (Input.GetKeyDown(_rightKey)) BroadcastDPadAction(DPad.Right);
            if (Input.GetKeyDown(_upKey)) BroadcastDPadAction(DPad.Up);
            if (Input.GetKeyDown(_downKey)) BroadcastDPadAction(DPad.Down);
        }
    }
}
