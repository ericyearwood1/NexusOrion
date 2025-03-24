// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System.Text;
using CTRL;
using CTRL.ClientBehaviors;
using OSIG.Tools.Layout;
using ProtoKit.GraphicBase;
using TMPro;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class CTRLIndicatorVisuals : MonoBehaviour
    {
        [SerializeField] private CTRLClient _ctrlClient;
        [SerializeField] private ScanForHost _scanForHost;
        [SerializeField] private ARGlassesWristband _wristband;

        [SerializeField] private TextMeshProUGUI _emgDebugText;
        [SerializeField] private RoundedRect _ctrlrFillColor;
        [SerializeField] private RoundedRect _indexPressFillColor;
        [SerializeField] private RoundedRect _middlePressFillColor;
        [SerializeField] private OCLayoutProportional _vertical1DProportional;

        [SerializeField] private OCLayoutProportional _accelXProportional;
        [SerializeField] private OCLayoutProportional _accelYProportional;
        [SerializeField] private OCLayoutProportional _accelZProportional;

        [SerializeField] private RoundedRect _vertical1DHandle;
        [SerializeField] private RoundedRect _dpadHandle;

        private OCLayoutProportional _dpadProportional;
        private OCLayoutProportional _imuProportional;

        public bool IsScanning
        {
#if UNITY_WEBGL
            get { return false; }
#else
            get { return _scanForHost && _scanForHost.IsScanning; }
#endif
        }

        protected void Start()
        {
            this.Scene(ref _ctrlClient, optional: true);
            this.Scene(ref _scanForHost, optional: true);
            this.Scene(ref _wristband, optional: true);
            this.Descendant(ref _emgDebugText, nameStrict: nameof(_emgDebugText), includeInactive:true);
            this.Descendant(ref _ctrlrFillColor, nameStrict: nameof(_ctrlrFillColor), optional:true);
            this.Descendant(ref _indexPressFillColor, nameStrict: nameof(_indexPressFillColor), includeInactive:true);
            this.Descendant(ref _middlePressFillColor, nameStrict: nameof(_middlePressFillColor), includeInactive:true);
            this.Descendant(ref _vertical1DProportional, nameStrict: nameof(_vertical1DProportional), includeInactive:true);
            this.Descendant(ref _accelXProportional, nameStrict: nameof(_accelXProportional), includeInactive:true);
            this.Descendant(ref _accelYProportional, nameStrict: nameof(_accelYProportional), includeInactive:true);
            this.Descendant(ref _accelZProportional, nameStrict: nameof(_accelZProportional), includeInactive:true);
            this.Descendant(ref _dpadProportional, nameStrict: nameof(_dpadProportional), includeInactive:true);
            this.Descendant(ref _imuProportional, nameStrict: nameof(_imuProportional), includeInactive:true);

            if(!_ctrlClient || !_wristband) gameObject.SetActive(false);

            _indexPressFillColorA = _indexPressFillColor.ColorA;
            _middlePressFillColorA = _middlePressFillColor.ColorA;
            _vartical1DHandleColorA = _vertical1DHandle.ColorA;
            _dpadHandleColorA = _dpadHandle.ColorA;
            _emgDebugText.SetText(string.Empty);
            CtrlConnected(_ctrlClient.State == ConnectionState.Connected);

            Pinched(HandFinger.Index, false);
            Pinched(HandFinger.Middle, false);

            if (_displayLastLoggedMessage) Application.logMessageReceivedThreaded += LogMessageReceived;
            _ctrlClient.OnConnect.AddListener(CtrlConnectedTrue);
            _ctrlClient.OnDisconnect.AddListener(CtrlConnectedFalse);
            _ctrlClient.OnError.AddListener(CtrlClientError);
            _wristband.WhenPinch += Pinched;
            _wristband.WhenDPad += UpdateDPadActionIndicator;
#if !UNITY_WEBGL
            if (_scanForHost) _scanForHost.OnPotentialHostFound.AddListener(PotentialHostFound);
#endif
        }

        private void OnDestroy()
        {
            if (_displayLastLoggedMessage) Application.logMessageReceivedThreaded -= LogMessageReceived;
            _ctrlClient.OnConnect.RemoveListener(CtrlConnectedTrue );
            _ctrlClient.OnDisconnect.RemoveListener(CtrlConnectedFalse);
            _ctrlClient.OnError.RemoveListener(CtrlClientError);
            _wristband.WhenPinch -= Pinched;
            _wristband.WhenDPad -= UpdateDPadActionIndicator;
#if !UNITY_WEBGL
            if (_scanForHost) _scanForHost.OnPotentialHostFound.RemoveListener(PotentialHostFound);
#endif
        }

        private void CtrlConnectedTrue() => CtrlConnected(true);
        private void CtrlConnectedFalse() => CtrlConnected(false);

        [SerializeField] private float _updateTextRate = 0.4f;
        [SerializeField] private bool _displayLastLoggedMessage;

        [SerializeField] private string _lastErrorMessage;
        [SerializeField] private float _lastPotentialHostTime;
        [SerializeField] private float _lastScanTime;
        [SerializeField] private bool _isConnected;
        [SerializeField] private string _lastCtrlLog;

        [SerializeField] private bool _pendingHostFound;
        private readonly StringBuilder _sb = new StringBuilder(2048);
        private Color _indexPressFillColorA;
        private Color _middlePressFillColorA;
        private Color _vartical1DHandleColorA;
        private Color _dpadHandleColorA;
        private float _indicatorColorLerp = 2f;

        private float _prevImuProportionalWidth;
        private float _prevImuProportionalHeight;

        private Vector3 _prevAccel;

        private void UpdateDPadActionIndicator(DPad dPad)
        {
            var dir = dPad.ToVector2();
            _dpadProportional.SetWidthMax(Mathf.InverseLerp(-1, 1, dir.x));
            _dpadProportional.SetHeightMax(Mathf.InverseLerp(-1, 1, dir.y));
            _dpadProportional.SetDirty();
            _dpadHandle.ColorA = Color.white;
        }

        private void Pinched(HandFinger digit, bool isPress)
        {
            var visual = digit == HandFinger.Middle ? _middlePressFillColor : _indexPressFillColor;

            if (isPress) visual.ColorA = Color.white;
            visual.enabled = isPress;
        }

        private void CtrlClientError(string msg)
        {
            _lastErrorMessage = msg.Limit(3);
            UpdateEmgText();
        }

        private void CtrlConnected(bool isConnected)
        {
            _isConnected = isConnected;
            if(_ctrlrFillColor) _ctrlrFillColor.enabled = isConnected;
            UpdateEmgText();
        }

        private void PotentialHostFound(string host)
        {
            _pendingHostFound = true;
        }

        private void Update()
        {
            _indexPressFillColor.ColorA = Color.Lerp(_indexPressFillColor.ColorA, _indexPressFillColorA, Time.deltaTime * _indicatorColorLerp);
            _middlePressFillColor.ColorA = Color.Lerp(_middlePressFillColor.ColorA, _middlePressFillColorA, Time.deltaTime * _indicatorColorLerp);

            _vertical1DHandle.ColorA = Color.Lerp(_vertical1DHandle.ColorA, _vartical1DHandleColorA, Time.deltaTime * _indicatorColorLerp);
            _dpadHandle.ColorA = Color.Lerp(_dpadHandle.ColorA, _dpadHandleColorA, Time.deltaTime * _indicatorColorLerp);

            if (_pendingHostFound)
            {
                _pendingHostFound = false;
                if (Time.time - _lastPotentialHostTime < _updateTextRate) return;
                _lastPotentialHostTime = Time.time;
                UpdateEmgText();
            }

            var projectedRadians = _wristband.ProjectedRadians;
            var imuPropWidth = Mathf.InverseLerp(-1.5f, 1.5f, projectedRadians.x);
            var imuPropHeight = Mathf.InverseLerp(-1.5f, 1.5f, projectedRadians.y);
            if (!Mathf.Approximately(_prevImuProportionalWidth, imuPropWidth))
            {
                _imuProportional.SetWidthMax(imuPropWidth);
                _prevImuProportionalWidth = imuPropWidth;
            }

            if (!Mathf.Approximately(_prevImuProportionalHeight, imuPropHeight))
            {
                _imuProportional.SetHeightMax(imuPropHeight);
                _prevImuProportionalHeight = imuPropHeight;
            }

            void PositionAccelHandle(float val, float previous, OCLayoutProportional proportional)
            {
                if (Mathf.Approximately(val, previous)) return;
                var accelRange = 9.8f;
                proportional.SetHeightMax(Mathf.InverseLerp(-accelRange, accelRange, val));
            }

            var pitchedAcceleration = _wristband.PitchedAcceleration;
            PositionAccelHandle(pitchedAcceleration.x, _prevAccel.x, _accelXProportional);
            PositionAccelHandle(pitchedAcceleration.y, _prevAccel.y, _accelYProportional);
            PositionAccelHandle(pitchedAcceleration.z, _prevAccel.z, _accelZProportional);
            _prevAccel = pitchedAcceleration;
        }

        private void LogMessageReceived(string condition, string stacktrace, LogType type)
        {
            if (!condition.Contains("[CTRL]")) return;
            _lastCtrlLog = condition;
            UpdateEmgText();
        }

        private void UpdateEmgText()
        {
            _emgDebugText.color = _isConnected ? Color.white : Color.red;
            _sb.Clear();
            _sb.AppendLine($"EMG Status: {(_isConnected ? "Connected" : "Fail")} - {_ctrlClient.Host}:{_ctrlClient.Port}");

            if (!_isConnected)
            {
                if (!string.IsNullOrEmpty(_lastErrorMessage)) _sb.AppendLine(_lastErrorMessage.Trim());
#if !UNITY_WEBGL
                if (_scanForHost && _scanForHost.enabled)
                {
                    if (_scanForHost.IsScanning)
                    {
                        _lastScanTime = Time.time;
                        _sb.AppendLine($"Scanning for EMG connection...");
                    }
                    else
                    {
                        _sb.AppendLine($"Last scan time {Time.time - _lastScanTime:0} s ago"); //<size=18>
                    }
                }
#endif
                if (_displayLastLoggedMessage) _sb.AppendLine(_lastCtrlLog.Substring(0, Mathf.Min(_lastCtrlLog.Length, 150)));
            }

            _emgDebugText.SetText(_sb.ToString());
        }
    }
}
