using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SimpleDragMoveLogger : MonoBehaviour
    {
        [SerializeField] private MechanicDragMove _mechanic;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenDragMove += Handle;
        private void OnDisable() => _mechanic.WhenDragMove -= Handle;

        private void Handle(Mechanic.DragMove.Event e)
        {
            var drag = e.Drag;
            var dragDeltaLocal = drag.DeltaLocal;
            var dragDeltaWorld = drag.DeltaWorld;
            var dragDeltaHead = drag.DeltaHead;

            Debug.Log($"{e.Phase}, World: {dragDeltaWorld}, Local: {dragDeltaLocal}, Head: {dragDeltaHead}");

            if (e.Phase.IsBegin())
            {
            }

            if (e.Phase.IsUpdate())
            {
            }

            // User released voluntarily
            if (e.Phase.IsSuccess())
            {
                Debug.Log($"Success! {dragDeltaWorld}");
            }

            // Peel was cancelled for some reason
            if (e.Phase.IsCancel())
            {
                Debug.Log($"Cancel! {dragDeltaWorld}");
            }
        }
    }
}
