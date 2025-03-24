using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public class ComplexLongPressExample : MonoBehaviour
    {
        [SerializeField] private MechanicLongPress _mechanic;
        [SerializeField] private float _spawnedScale = 0.1f;
        [SerializeField, ReadOnly] private GameObject _spawned;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenLongPress += Handle;
        private void OnDisable() => _mechanic.WhenLongPress -= Handle;

        private void Handle(Mechanic.LongPress.Event longPressEvent)
        {
            var phase = longPressEvent.Phase;
            var progress = longPressEvent.Progress;

            Debug.Log($"Phase: {phase}, Progress: {progress}");

            if (phase.IsPreExecute())
            {
                if (!_spawned)
                {
                    _spawned = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    _spawned.transform.position = transform.position;
                }

                _spawned.transform.localScale = Vector3.one * _spawnedScale * progress;
            }

            if (phase.IsExecute())
            {
                Debug.Log("Execute!");
                _spawned.transform.localScale = Vector3.one * _spawnedScale * 1.5f;
                _spawned.GetComponent<Renderer>().material.color = Color.green;
            }

            if (phase.IsEnded())
            {
                Destroy(_spawned);
                _spawned = null;
            }

            // You can dig into the Selection.Current object for 'source' data from the rig and manipulation
            var selection = longPressEvent.Selection;
            var current = selection.Latest;
            var gazePose = current.Eyes;
            var pinchPose = current.Pinch;
            var gazeHitPose = current.TargetHit;

            // Dig into the Selection.Begin object stores values from the moment of "pinch begin"
            var begin = selection.Begin;

            // compare data from current with begin
            var wristMotionSinceBegin = current.Wrist.position - begin.Wrist.position;

            // ... but prefer using the provided helper methods when possible.
            // this gives the Mechanic a chance to override, tune, or scale concepts such as drag distance and duration
            var drag = longPressEvent.Selection.Drag;
            var totalDragInTargetSpace = drag.DeltaLocal;
            var timeSinceDragBegan = drag.Duration;

            var dragThisFrame = longPressEvent.Selection.DragThisFrame;
            var dragInTargetSpaceThisFrame = dragThisFrame.DeltaLocal;
            var timeSinceLastDragUpdate = dragThisFrame.Duration;
        }
    }
}
