using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComplexDoublePressExample : MonoBehaviour
    {
        [SerializeField] private MechanicMultiPress _mechanic;
        [SerializeField] private float _spawnedScale = 0.1f;
        [SerializeField, ReadOnly] private GameObject _spawned;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenMultiPress += Handle;
        private void OnDisable() => _mechanic.WhenMultiPress -= Handle;

        private void Handle(Mechanic.MultiPress.Event multiPressEvent)
        {
            var phase = multiPressEvent.Phase;

            Debug.Log($"Phase: {phase}");

            if (phase.IsExecuteAny()) // Successfully Double Clicked
            {
                if (!_spawned)
                {
                    _spawned = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    _spawned.transform.position = transform.position;
                }

                _spawned.transform.localScale = Vector3.one * _spawnedScale;
            }

            if (phase.IsExecuteSingle())
            {
                _spawned.GetComponent<Renderer>().material.color = Color.red;
                Destroy(_spawned, 0.5f);
            }

            if (phase.IsExecuteDouble())
            {
                _spawned.GetComponent<Renderer>().material.color = Color.green;
            }

            if (phase.IsEnded())
            {
                if(_spawned) Destroy(_spawned);
                _spawned = null;
            }

            // You can dig into the Selection.Current object for 'source' data from the rig and manipulation

            if (multiPressEvent.FirstSelection == null || multiPressEvent.SecondSelection == null) return;
            var selection = multiPressEvent.FirstSelection;
            var secondSelection = multiPressEvent.SecondSelection;
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
            var drag = selection.Drag;
            var totalDragInTargetSpace = drag.DeltaLocal;
            var timeSinceDragBegan = drag.Duration;

            var dragThisFrame = secondSelection.DragThisFrame;
            var dragInTargetSpaceThisFrame = dragThisFrame.DeltaLocal;
            var timeSinceLastDragUpdate = dragThisFrame.Duration;
        }
    }
}
