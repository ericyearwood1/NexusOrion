using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SimpleDoublePressLogger : MonoBehaviour
    {
        [SerializeField] private MechanicMultiPress _mechanic;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenMultiPress += HandleDoublePress;
        private void OnDisable() => _mechanic.WhenMultiPress -= HandleDoublePress;

        private void HandleDoublePress(Mechanic.MultiPress.Event e)
        {
            Debug.Log($"Phase: {e.Phase}");

            if (e.Phase.IsExecuteDouble()) Debug.Log($"Execute!");
        }
    }
}
