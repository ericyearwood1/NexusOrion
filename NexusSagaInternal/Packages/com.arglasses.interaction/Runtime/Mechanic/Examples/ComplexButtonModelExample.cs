using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComplexButtonModelExample : MonoBehaviour
    {
        [SerializeField] private ButtonModel _buttonModel;
        [SerializeField] private GameObject _spawned;
        [SerializeField] private float _destroyDelay = 1f;

        protected void Awake()
        {
            this.Ensure(ref _buttonModel);
            _buttonModel.WhenClicked += HandleClicked;
            _buttonModel.WhenPressBegin += HandlePressBegin;
            _buttonModel.WhenPressEnd += HandlePressEnd;
            _buttonModel.WhenStateChanged += HandleStateChanged;
            _buttonModel.WhenDisplacementLocalChanged += HandleDisplacement;

            // you can access the source Selection object via the Target
            // but be careful since the order of calling the Button events above relative to WhenSelecting is not guaranteed within a frame
            var target = _buttonModel.Target;
            target.WhenSelecting += HandleSelecting;
        }

        private void HandleClicked()
        {
            Log("Clicked");
            var scale = Vector3.one * 0.1f;
            _spawned = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _spawned.transform.position = transform.position;
            _spawned.transform.localScale = scale;
            Destroy(_spawned, _destroyDelay);
        }

        private void HandlePressBegin()
        {
            Log("PressBegin");
        }

        private void HandlePressEnd()
        {
            Log($"HandlePressEnd");
        }

        private void HandleStateChanged(TargetState newState)
        {
            Log($"HandleStateChanged: {newState}");
        }

        private void HandleDisplacement(Vector3 localNudge)
        {
            Log($"HandleDisplacement: {localNudge}");
        }

        private void Log(string msg)
        {
            Debug.Log(msg);
        }

        private void HandleSelecting(ISelection selection)
        {
            // You can dig into the Selection.Current object for 'source' data from the rig and manipulation
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

            var dragThisFrame = selection.DragThisFrame;
            var dragInTargetSpaceThisFrame = dragThisFrame.DeltaLocal;
            var timeSinceLastDragUpdate = dragThisFrame.Duration;
        }

    }
}
