using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComplexDragMoveExample : MonoBehaviour
    {
        [SerializeField] private MechanicDragMove _mechanic;

        private void Awake()
        {
            this.Ensure(ref _mechanic);
            if (!_transform) _transform = transform;
        }

        private void OnEnable() => _mechanic.WhenDragMove += Handle;
        private void OnDisable() => _mechanic.WhenDragMove -= Handle;

        [SerializeField] private float _resetTime = 4;
        [SerializeField] private Transform _transform;
        [SerializeField] private Vector3 _deltaLocalScale = Vector3.one;

        [SerializeField, ReadOnly] private Pose _pointerOffset;
        [SerializeField, ReadOnly] private Pose _defaultLocalPosition;
        [SerializeField, ReadOnly] private Pose _grabPoseLocal;
        [SerializeField, ReadOnly] private double _lastMove;

        private void Handle(Mechanic.DragMove.Event dragMoveEvent)
        {
            var phase = dragMoveEvent.Phase;

            var dragDeltaLocal = dragMoveEvent.Drag.DeltaLocal; // target's local space
            var scaledDeltaLocal = Vector3.Scale(dragDeltaLocal, _deltaLocalScale);

            if (phase.IsBegin()) _grabPoseLocal = _transform.ToPose(Space.Self);

            if (phase.IsUpdate())
            {
                _transform.localPosition = _grabPoseLocal.position + scaledDeltaLocal;
                _lastMove = Time.time;
            }

            if (phase.IsSuccess())
            {
            }

            // Peel was cancelled for some reason
            if (phase.IsCancel())
            {
                _transform.Set(_defaultLocalPosition, Space.Self);
            }

            // You can dig into the Selection.Current object for 'source' data from the rig and manipulation
            var current = dragMoveEvent.Selection.Latest;
            var gazePose = current.Eyes;
            var pinchPose = current.Pinch;
            var gazeHitPose = current.TargetHit;

            // Dig into the Selection.Begin object stores values from the moment of "pinch begin"
            var begin = dragMoveEvent.Selection.Begin;

            // compare data from current with begin
            var wristMotionSinceBegin = current.Wrist.position - begin.Wrist.position;

            // ... but prefer using the provided helper methods when possible.
            // this gives the Mechanic a chance to override, tune, or scale concepts such as drag distance and duration
            var drag = dragMoveEvent.Selection.Drag;
            var totalDragInTargetSpace = drag.DeltaLocal;
            var timeSinceDragBegan = drag.Duration;

            var dragThisFrame = dragMoveEvent.Selection.DragThisFrame;
            var dragInTargetSpaceThisFrame = dragThisFrame.DeltaLocal;
            var timeSinceLastDragUpdate = dragThisFrame.Duration;
        }

        private void Start()
        {
            _defaultLocalPosition = transform.ToPose(Space.Self);
        }

        void Update()
        {
            if(Time.time - _lastMove > _resetTime) transform.Set(_defaultLocalPosition, Space.Self);
        }
    }
}
