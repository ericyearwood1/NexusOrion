using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComplexPeelExample : MonoBehaviour
    {
        [SerializeField] private MechanicPeel _mechanic;
        [SerializeField] private Transform _peeledContentRoot;
        [SerializeField] private float _spawnedScale = 0.1f;
        [SerializeField, ReadOnly] private GameObject _spawned;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenPeel += HandlePeel;
        private void OnDisable() => _mechanic.WhenPeel -= HandlePeel;

        private Pose _pointerOffset;
        private void HandlePeel(Mechanic.Peel.Event peelEvent)
        {
            var phase = peelEvent.Phase;
            var dragDeltaWorld = peelEvent.Drag.DeltaWorld; // worldspace drag
            var dragDeltaLocal = peelEvent.Drag.DeltaLocal; // target's local space

            Debug.Log($"{phase}: {dragDeltaLocal}");

            // worldspace position which our grab should be anchored to
            var destinationPosition = peelEvent.Drag.DestinationWorld;

            // We have started dragging, but haven't moved far enough in Z to actually Peel yet
            if (phase.IsPre())
            {
                if (!_spawned)
                {
                    _spawned = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    if (_peeledContentRoot) _spawned.transform.parent = _peeledContentRoot;
                }

                var normalizedCancel = dragDeltaLocal.WithZ(0).magnitude / _mechanic.CancelDistanceXY;
                var normalizedPeel = dragDeltaLocal.z / _mechanic.PeelDistanceZ;

                _spawned.transform.localScale = (Vector3.one * _spawnedScale * (1-normalizedCancel) * normalizedPeel).ForceNonZero();
                _spawned.transform.position = transform.position + peelEvent.Drag.DeltaWorld;

                var spawnedPose = _spawned.transform.ToPose();
                _pointerOffset = destinationPosition.InverseTransform(spawnedPose);
            }

            // We Peeled far enough to execute the Peel!
            if (phase.IsExecute())
            {
                Debug.Log("Execute!");
                _spawned.transform.localScale = Vector3.one * _spawnedScale * 1.5f;
            }

            // We are still hanging onto the Peeled item...
            if (phase.IsPost())
            {
                var pose = destinationPosition.Transform(_pointerOffset);
                _spawned.transform.Set(pose);
            }

            // User released voluntarily
            if (phase.IsSuccess())
            {
                _spawned.GetComponent<Renderer>().material.color = Color.green;
                _spawned = null;
            }

            // Peel was cancelled for some reason
            if (phase.IsCancel())
            {
                Destroy(_spawned);
                _spawned = null;
            }

            // You can dig into the Selection.Current object for 'source' data from the rig and manipulation
            var current = peelEvent.Selection.Latest;
            var gazePose = current.Eyes;
            var pinchPose = current.Pinch;
            var gazeHitPose = current.TargetHit;

            // Dig into the Selection.Begin object stores values from the moment of "pinch begin"
            var begin = peelEvent.Selection.Begin;

            // compare data from current with begin
            var wristMotionSinceBegin = current.Wrist.position - begin.Wrist.position;

            // ... but prefer using the provided helper methods when possible.
            // this gives the Mechanic a chance to override, tune, or scale concepts such as drag distance and duration
            var drag = peelEvent.Selection.Drag;
            var totalDragInTargetSpace = drag.DeltaLocal;
            var timeSinceDragBegan = drag.Duration;

            var dragThisFrame = peelEvent.Selection.DragThisFrame;
            var dragInTargetSpaceThisFrame = dragThisFrame.DeltaLocal;
            var timeSinceLastDragUpdate = dragThisFrame.Duration;
        }
    }
}
