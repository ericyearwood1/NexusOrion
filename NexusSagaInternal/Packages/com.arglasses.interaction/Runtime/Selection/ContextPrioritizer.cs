using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static UnityEngine.KeyCode;

namespace ARGlasses.Interaction
{
    public class ContextPrioritizer : MonoBehaviour
    {
        [SerializeField, ReadOnly] private Selector _selector;
        [SerializeField, ReadOnly] private ARGlassesWristband _wristband;

        private const string InputContextKey = nameof(ContextPrioritizer) + "Index";
        [SerializeField] private bool _forceLeashedSurface;
        [SerializeField] private InputCategory _defaultInputCategory;

        public GameManager gameManager;

        public bool ForceLeashedSurface
        {
            get => _forceLeashedSurface;
            set => _forceLeashedSurface = value;
        }

        [SerializeField] private List<InputContext> _inputContexts = new();
        public KeyCode[] _keyCodes = { F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12 };

        [SerializeField, ReadOnly] private EyeGazeInputContext _eyeGazeContext;
        [SerializeField, ReadOnly] private HandsInputContext _handsContext;
        [SerializeField, ReadOnly] private CursorInputContext _cursorContext;
        [SerializeField, ReadOnly] private CardinalInputContext _cardinalContext;
        public CursorInputContext CursorInputContext => _cursorContext;

        [SerializeField, ReadOnly] private InputContext _inputContext;
        [SerializeField, ReadOnly] private TargetContext _targetContext;
        [SerializeField, ReadOnly] private ARGlassesRig _rig;

        [SerializeField] private bool _loadSavedContextCategory;

        

        private void Awake()
        {
            this.Ancestor(ref _rig);
            this.Scene(ref _selector);
            this.Scene(ref _wristband);

            this.Descendant(ref _eyeGazeContext);
            this.Descendant(ref _handsContext);
            this.Descendant(ref _cursorContext);
            this.Descendant(ref _cardinalContext);

            if (_inputContexts.Count == 0) _inputContexts = GetComponentsInChildren<InputContext>(includeInactive: false).ToList();
            _inputContext = _inputContexts[0];

            var inputCategory = _defaultInputCategory;
            if (_loadSavedContextCategory) inputCategory = (InputCategory)PlayerPrefs.GetInt(InputContextKey, 0);
            Category = inputCategory;

            // _wristband.WhenConnectionChanged += isConnected => Category = isConnected ? InputCategory.Cursor : Category;
        }

        public InputCategory Category
        {
            get => _inputContext != null ? _inputContext.Category : InputCategory.None;
            set
            {
                // var disallowCardinal = !Application.isEditor && (!_rig.Wristband || !_rig.Wristband.IsConnected);
                if (value.IsCardinal()) value = value.Next();

                if(value.IsCursor()) _wristband.CtrlClientEnabled = true;
                if(value.IsHands()) _wristband.CtrlClientEnabled = false;

                _inputContext = GetInputContext(value);
                Debug.Log($"Setting InputContext {_inputContext.Category}");

                PlayerPrefs.SetInt(InputContextKey, (int)_inputContext.Category);
            }
        }

        public bool HandRays
        {
            get => Category.IsHands();
            set
            {
                if(value && !Category.IsHands()) Category = InputCategory.Hands;
                if(!value && Category.IsHands()) Category = InputCategory.Eyes;
            }
        }

        public InputContext GetInputContext(InputCategory category)
        {
            InputContext newContext = null;
            if (category.IsNone()) category = ExtensionsInputCategory.Default;
            if (category.IsEyes()) newContext = _eyeGazeContext;
            if (category.IsHands()) newContext = _handsContext;
            if (category.IsCursor()) newContext = _cursorContext;
            if (category.IsCardinal()) newContext = _cardinalContext;
            return newContext;
        }

        public void CycleContext()
        {
            if (_inputContext)
            {
                Category = _inputContext.Category.Next();
                return;
            }

            Category = ExtensionsInputCategory.Default;
        }

        private void Update()
        {
            for (int i = 0; i < _inputContexts.Count; i++)
            {
                if (Input.GetKeyDown(_keyCodes[i]))
                {
                    Category = (InputCategory)(i + 1);
                    break;
                }
            }

            var snapshot = _rig.Snapshot();
            _selector.InputContext = _inputContext;
            _selector.TargetContext = _inputContext.BestSurface(snapshot);
            //Debug.Log("Gaze snapshot eyes is: " + snapshot.Gaze.Eyes.position);
            RaycastPointer(snapshot.Gaze.Eyes.position, snapshot.Gaze.Eyes.rotation);
            if (snapshot.RightHand.IsPinching(HandFinger.Index))
            {
                gameManager.pinchBool = true;
            }
            else
            {
                gameManager.pinchBool = false;
            }
        }

        public void RaycastPointer(Vector3 posePos, Quaternion poseRot)
        {
            Vector3 direction = poseRot * Vector3.forward;
            Ray ray = new Ray(posePos, direction);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                //Debug.Log("Hit: " + hit.collider.gameObject.name);
                if (hit.collider.gameObject.name == "Sphere")
                {
                    gameManager.sphereBool = true;
                }
                else { gameManager.sphereBool = false; }

                if (hit.collider.gameObject.name == "Cube")
                {
                    gameManager.cubeBool = true;
                }
                else { gameManager.cubeBool = false; }
            }
            else
            {
                gameManager.sphereBool = false;
                gameManager.cubeBool = false;
            }
        }

        [SerializeField] private bool _renderDebug = true;

        private void LateUpdate()
        {
            if (!_renderDebug || !_targetContext) return;
            DebugGizmos.Color = Color.white;
            DebugGizmos.LineWidth = 0.0015f;
            // Line.Draw(_cursorSurface.Collider);
            // if ((_nearestColliderPoint - _planeHit).magnitude > 0.001f) Line.Draw(_planeHit, _nearestColliderPoint);
        }
    }
}
