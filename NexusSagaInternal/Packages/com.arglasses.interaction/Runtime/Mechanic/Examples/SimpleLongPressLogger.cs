using UnityEngine;

namespace ARGlasses.Interaction
{
    public class SimpleLongPressLogger : MonoBehaviour
    {
        [SerializeField] private MechanicLongPress _mechanic;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenLongPress += HandleLongPress;
        private void OnDisable() => _mechanic.WhenLongPress -= HandleLongPress;

        private void HandleLongPress(Mechanic.LongPress.Event e)
        {
            var progress = e.Progress;

            if (e.Phase.IsPreExecute())
            {
                Debug.Log($"Not Longpressing yet {progress}");
            }

            // We held the press for long enough to execute
            if (e.Phase.IsExecute())
            {
                Debug.Log($"Execute! {progress}");
            }

            // We are still hanging onto the longpress...
            if (e.Phase.IsPostExecute())
            {
                Debug.Log($"Already Longpressed {progress}");
            }

            // released voluntarily
            if (e.Phase.IsSuccess())
            {
                Debug.Log($"Longpress Success {progress}");
            }

            // cancelled for some reason
            if (e.Phase.IsCancel())
            {
                Debug.Log($"Longpress Cancel {progress}");
            }
        }
    }
}
