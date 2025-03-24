using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SimplePeelLogger : MonoBehaviour
    {
        [SerializeField] private MechanicPeel _mechanic;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenPeel += HandlePeel;
        private void OnDisable() => _mechanic.WhenPeel -= HandlePeel;

        private void HandlePeel(Mechanic.Peel.Event e)
        {
            var drag = e.Drag;
            var dragDeltaLocal = drag.DeltaLocal;

            if (e.Phase.IsPre())
            {
                Debug.Log($"Not Peeled Yet {dragDeltaLocal}");
            }

            // We Peeled far enough to execute the Peel!
            if (e.Phase.IsExecute())
            {
                Debug.Log($"Execute! {dragDeltaLocal}");
            }

            // We are still hanging onto the Peeled item...
            if (e.Phase.IsPost())
            {
                Debug.Log($"Already Peeled, now manipulating... {dragDeltaLocal}");
            }

            // User released voluntarily
            if (e.Phase.IsSuccess())
            {
                Debug.Log($"Peel Success {dragDeltaLocal}");
            }

            // Peel was cancelled for some reason
            if (e.Phase.IsCancel())
            {
                Debug.Log($"Peel Cancel {dragDeltaLocal}");
            }
        }
    }
}
