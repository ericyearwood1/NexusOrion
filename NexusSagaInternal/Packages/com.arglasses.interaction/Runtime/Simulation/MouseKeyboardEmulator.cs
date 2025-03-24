using System;
using System.Linq;
using UnityEngine;
using UnityEditor;

namespace ARGlasses.Interaction
{
#if UNITY_EDITOR
    [CustomEditor(typeof(MouseKeyboardEmulator))]
    public class MouseKeyboardEmulatorEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            var instance = (MouseKeyboardEmulator) target;
            var sensitivity = instance.LocalMouseSensitivityPref;
            var newSensitivity = EditorGUILayout.FloatField("Local Mouse Sensitivity", sensitivity);
            if (sensitivity != newSensitivity) instance.LocalMouseSensitivityPref = newSensitivity;

            base.OnInspectorGUI();
        }
    }
#endif

    [DefaultExecutionOrder(ExecutionOrder.MouseKeyboardEmulator)]
    public class MouseKeyboardEmulator : MonoBehaviour
    {
        private const string LocalMouseSensitivityPrefKey = "MouseSensitivity";
        public float LocalMouseSensitivityPref
        {
            get => PlayerPrefs.GetFloat(LocalMouseSensitivityPrefKey, 1);
            set => PlayerPrefs.SetFloat(LocalMouseSensitivityPrefKey, value);
        }

        [SerializeField] private float _pushPullSpeed = 1f;
        [SerializeField] private float _playerMoveSpeed = 1f;
        [SerializeField] private float _forwardMoveSpeedScale = 0.5f;
        [SerializeField] private float _horizontalMoveSpeedScale = 0.75f;
        [SerializeField] private float _verticalMoveSpeedScale = 0.65f;

        [SerializeField] private Vector3 _handsOffsetFromEyes = new Vector3(0.2f, -0.15f, 0.25f);
        [SerializeField] private float _mouseToHandManipulationScale = 0.0025f;

        [SerializeField] private float _mouseControlWarmUpTime = 0.2f;

        [SerializeField] private bool _simulateEyeGazeJitter = false;
        [SerializeField] private Vector2 _jitterSimRange = new Vector2(0.01f, 0.01f);
        [SerializeField] private float _jitterSimFrequency = 0.1f;

        [SerializeField] private EFakePinchMouseButton _leftIndexMouseButton = EFakePinchMouseButton.LeftClick;
        [SerializeField] private EFakePinchMouseButton _leftMiddleMouseButton = EFakePinchMouseButton.MouseButton4;
        [SerializeField] private EFakePinchMouseButton _rightIndexMouseButton = EFakePinchMouseButton.RightClick;
        [SerializeField] private EFakePinchMouseButton _rightMiddleMouseButton = EFakePinchMouseButton.MouseButton5;

        [SerializeField] private KeyCode LeftIndexKey = KeyCode.X;
        [SerializeField] private KeyCode LeftMiddleKey = KeyCode.Z;

        [SerializeField] private KeyCode StopPlaybackKey = KeyCode.Escape;
        [SerializeField] private KeyCode PauseUnpauseKey = KeyCode.P;
        [SerializeField] private KeyCode LockScreenRotation = KeyCode.LeftShift;
        [SerializeField] private KeyCode _moveForwardKey = KeyCode.W;
        [SerializeField] private KeyCode _moveBackwardKey = KeyCode.S;
        [SerializeField] private KeyCode _moveLeftKey = KeyCode.A;
        [SerializeField] private KeyCode _moveRightKey = KeyCode.D;
        [SerializeField] private KeyCode _moveUpKey = KeyCode.Space;
        [SerializeField] private KeyCode _moveDownKey = KeyCode.LeftControl;

        [SerializeField] private bool _arrowKeysRotateHead = true;

        [SerializeField] private KeyCode _handDragKey = KeyCode.None;
        [SerializeField] private KeyCode _pullKey = KeyCode.None;
        [SerializeField] private KeyCode _pushKey = KeyCode.None;
        [SerializeField] private KeyCode _toggleDebugVisualization = KeyCode.F12;

        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private OVRCameraRig _ovrCameraRig;
        [SerializeField, ReadOnly] private ARGlassesRig _rig;
        [SerializeField, ReadOnly] private ARGlassesRigDebug _debug;

        [SerializeField, ReadOnly] private MeshRenderer _cursorPlane;
        [SerializeField, ReadOnly] private Vector3 _mouseOffsetFromCenter;
        [SerializeField] private float _mouseCursorOffsetZ = 0.002f;
        [SerializeField] private float _mouseCursorSize = 0.0005f;

        [SerializeField, ReadOnly] private bool _suppressPinches = false;
        [SerializeField, ReadOnly] private float _mouseSensitivity = 3f;

        private float _jitterTimer = 0f;

        private RuntimePlatform[] AllowedPlatforms { get; } =
            { RuntimePlatform.OSXEditor, RuntimePlatform.WindowsEditor, RuntimePlatform.WindowsPlayer, RuntimePlatform.OSXPlayer };

        public enum EFakePinchMouseButton
        {
            NONE = -1, // 0xFFFFFFFF
            LeftClick = 0,
            RightClick = 1,
            MiddleClick = 2,
            MouseButton4 = 3,
            MouseButton5 = 4,
        }

        protected void Awake()
        {
            if (!AllowedPlatforms.Contains(Application.platform))
            {
                enabled = false;
                return;
            }

            this.Scene(ref _selector);
            this.Scene(ref _ovrCameraRig);
            this.Scene(ref _rig);
            this.Scene(ref _debug);
            _camera = _ovrCameraRig.centerEyeAnchor.GetComponent<Camera>();

            this.Descendant(ref _cursorPlane, nameStrict: nameof(_cursorPlane));
        }

        private void Update()
        {
            if (Application.isEditor)
            {
#if UNITY_EDITOR
                if (Input.GetKeyDown(PauseUnpauseKey)) EditorApplication.isPaused = !EditorApplication.isPaused;
#endif
                if (Input.GetKeyDown(_toggleDebugVisualization)) _debug.Active = !_debug.Active;
            }

            if (!Application.isFocused || Time.time < _mouseControlWarmUpTime)
            {
                _suppressPinches = true;
                return;
            }

            var isLocked = Cursor.lockState == CursorLockMode.Locked;
            _cursorPlane.gameObject.SetActive(isLocked && !_rig.IsUserInHmd);
            if (isLocked)
            {
                _suppressPinches = _suppressPinches && Input.GetMouseButton(0);
                if (Input.GetKeyDown(StopPlaybackKey))
                {
                    Cursor.lockState = CursorLockMode.None;
                    return;
                }

                ApplyMouseKeyboard();
            }
            else
            {
                if (Input.GetKeyDown(StopPlaybackKey)) Application.Quit();
                if (Input.GetMouseButtonDown(0))
                {
                    _suppressPinches = true;
                    Cursor.lockState = CursorLockMode.Locked;
                }
            }
        }

        private float GetInput(KeyCode keyCode, float scale = 1) => keyCode != KeyCode.None && Input.GetKey(keyCode) ? scale : 0;

        [SerializeField, ReadOnly] private Vector3 _hmdEuler;
        [SerializeField] private Pose _hmdLocalPose;

        [SerializeField, ReadOnly] private Pose _playerMovementPose;
        [SerializeField, ReadOnly] private Pose _eyeGazePose;
        public Pose EmulatedEyeTracking => _eyeGazePose;

        [SerializeField, ReadOnly] private Camera _camera;

        [Serializable]
        public struct EmulatedHand
        {
            public Side _side;
            public bool _index;
            public bool _middle;
            public Pose _root;
            public Pose _pointer;
            public Vector3 _drag;

            public Snapshot.Hand Snapshot
            {
                get
                {
                    var handBones = Interaction.Snapshot.HandBones.Default(_side, _root);
                    return new(
                        side: _side,
                        pinch: _root,
                        wrist: _root,
                        ray: _pointer,
                        rayActive: true,
                        indexStrength: _index ? 1 : 0,
                        middleStrength: _middle ? 1 : 0,
                        indexPinching: _index,
                        middlePinching: _middle,
                        bones: handBones,
                        isTracked: true
                    );
                }
            }
        }

        [SerializeField] private EmulatedHand _left = new() { _side = Side.Left };
        public EmulatedHand EmulatedLeft => _left;
        [SerializeField] private EmulatedHand _right = new() { _side = Side.Right };
        public EmulatedHand EmulatedRight => _right;

        public EmulatedHand GetEmulation(Side side) => side.IsLeft() ? EmulatedLeft : side.IsRight() ? EmulatedRight : default;

        [SerializeField] private float _edgeRotationScale = 75;
        [SerializeField] private AnimationCurve _edgeRotationCurve = AnimationCurve.Linear(0, 0, 1, 1);

        private void ApplyMouseKeyboard()
        {
            var forwardMove = (Input.GetKey(_moveForwardKey) ? 1 : Input.GetKey(_moveBackwardKey) ? -1 : 0) * _playerMovementPose.forward;
            var horizontalMove = (Input.GetKey(_moveRightKey) ? 1 : Input.GetKey(_moveLeftKey) ? -1 : 0) * _playerMovementPose.right;
            var verticalMove = Vector3.up * (Input.GetKey(_moveUpKey) ? 1 : Input.GetKey(_moveDownKey) ? -1 : 0);

            var hmdPositionDelta = forwardMove * _forwardMoveSpeedScale + horizontalMove * _horizontalMoveSpeedScale + verticalMove * _verticalMoveSpeedScale;
            _playerMovementPose.position += hmdPositionDelta.normalized * Time.deltaTime * _playerMoveSpeed;

            Vector3 halfScreen = new Vector3(Screen.width, Screen.height) * 0.5f;
            var mouseDelta = new Vector3(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y")) * (_mouseSensitivity * LocalMouseSensitivityPref);
            if (_suppressPinches) mouseDelta = Vector3.zero;
            else
            {
                var clampedX = Mathf.Clamp(_mouseOffsetFromCenter.x + mouseDelta.x, -halfScreen.x, halfScreen.x);
                var clampedY = Mathf.Clamp(_mouseOffsetFromCenter.y + mouseDelta.y, -halfScreen.y, halfScreen.y);
                _mouseOffsetFromCenter = new Vector3(clampedX, clampedY);
            }

            if (!Input.GetKey(LockScreenRotation))
            {
                var mouseOffset = new Vector3(_mouseOffsetFromCenter.x / halfScreen.x, _mouseOffsetFromCenter.y / halfScreen.y);
                mouseOffset.x = _edgeRotationCurve.Evaluate(Mathf.Abs(mouseOffset.x)) * Mathf.Sign(mouseOffset.x);
                mouseOffset.y = _edgeRotationCurve.Evaluate(Mathf.Abs(mouseOffset.y)) * Mathf.Sign(mouseOffset.y);
                var angularDelta = new Vector3(-mouseOffset.y, mouseOffset.x, 0) * _edgeRotationScale * Time.deltaTime;
                _playerMovementPose.rotation = Quaternion.LookRotation(Quaternion.Euler(_playerMovementPose.rotation.eulerAngles + angularDelta) * Vector3.forward);
            }

            var trackingSpace = _ovrCameraRig.trackingSpace;
            if (_rig.IsUserInHmd)
            {
                trackingSpace.position = _playerMovementPose.position;
                return;
            }

            trackingSpace.Set(_playerMovementPose);

            if (_arrowKeysRotateHead) _hmdEuler += new Vector3(GetInput(KeyCode.UpArrow) - GetInput(KeyCode.DownArrow), GetInput(KeyCode.RightArrow) - GetInput(KeyCode.LeftArrow), 0);

            var hmdLocalPose = new Pose(Vector3.zero, Quaternion.Euler(_hmdEuler));
            var centerEyeAnchor = _ovrCameraRig.centerEyeAnchor;
            centerEyeAnchor.Set(hmdLocalPose, Space.Self);
            _hmdLocalPose = hmdLocalPose;

            var mousePosition = _mouseOffsetFromCenter + halfScreen;
            var mouseScreenPose = _camera.ScreenPointToRay(mousePosition).ToPose();
            var cursorRotation = Quaternion.LookRotation(mouseScreenPose.forward * -1) * Quaternion.Euler(-90, 0, 0);
            _cursorPlane.transform.SetPositionAndRotation(mouseScreenPose.position + mouseScreenPose.forward * _mouseCursorOffsetZ, cursorRotation);
            _cursorPlane.transform.localScale = Vector3.one * _mouseCursorSize;

            if (_simulateEyeGazeJitter)
            {
                _jitterTimer += Time.deltaTime;
                if (_jitterTimer >= _jitterSimFrequency)
                {
                    float jitterX = UnityEngine.Random.Range(-_jitterSimRange.x, _jitterSimRange.x);
                    float jitterY = UnityEngine.Random.Range(-_jitterSimRange.y, _jitterSimRange.y);
                    mouseScreenPose.position += new Vector3(jitterX, jitterY, 0);
                    _jitterTimer = 0f;
                }
            }

            var eyePosition = centerEyeAnchor.position;
            var eyeRotation = Quaternion.LookRotation(mouseScreenPose.position - eyePosition);
            _eyeGazePose = new Pose(eyePosition, eyeRotation);

            // var usingWristband = inputContextController.Prioritizer.UsingWristband;
            var handDragKeyActive = _handDragKey == KeyCode.None || Input.GetKey(_handDragKey);
            if (handDragKeyActive) // && !usingWristband
            {
                _left._drag += mouseDelta * _mouseToHandManipulationScale;
                _right._drag += mouseDelta * _mouseToHandManipulationScale;
            }

            var isLeftIndex = Input.GetMouseButton((int)_leftIndexMouseButton) || Input.GetKey(LeftIndexKey);
            var isLeftMiddle = Input.GetMouseButton((int)_leftMiddleMouseButton) || Input.GetKey(LeftMiddleKey);

            var isRightIndex = Input.GetMouseButton((int)_rightIndexMouseButton);
            var isRightMiddle = Input.GetMouseButton((int)_rightMiddleMouseButton);

            if (!isLeftIndex && !isLeftMiddle) _left._drag = default;
            if (!isRightIndex && !isRightMiddle) _right._drag = default;

            _left._index = isLeftIndex && !_suppressPinches;
            _left._middle = isLeftMiddle && !_suppressPinches;
            _right._index = isRightIndex && !_suppressPinches;
            _right._middle = isRightMiddle && !_suppressPinches;

            var scrollDelta = 0.2f * Input.GetAxis("Mouse ScrollWheel") * _pushPullSpeed;
            scrollDelta += 0.01f * (Input.GetKey(_pullKey) ? -_pushPullSpeed : Input.GetKey(_pushKey) ? _pushPullSpeed : 0);
            if (isLeftIndex || isLeftMiddle) _left._drag.z += scrollDelta;
            if (isRightIndex || isRightMiddle) _right._drag.z += scrollDelta;

            _left._root = _camera.transform.Transform(new Pose(_handsOffsetFromEyes.FlipX() + _left._drag, _camera.transform.rotation));
            _left._pointer = mouseScreenPose.WithPosition(_left._root.position);

            _right._root = _camera.transform.Transform(new Pose(_handsOffsetFromEyes + _right._drag, _camera.transform.rotation));
            _right._pointer = mouseScreenPose.WithPosition(_right._root.position);
        }
    }
}
