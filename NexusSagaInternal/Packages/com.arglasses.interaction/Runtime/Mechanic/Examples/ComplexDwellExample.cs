using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComplexDwellExample : MonoBehaviour
    {
        [SerializeField] private MechanicDwell _mechanic;
        [SerializeField] private float _spawnedScale = 0.05f;
        [SerializeField, ReadOnly] private GameObject _spawned;

        // notice that unlike the other examples, we don't need a Mechanic here since DPad events are discrete
        // we can subscribe directly to a Target component
        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenDwell += HandleDwell;
        private void OnDisable() => _mechanic.WhenDwell -= HandleDwell;

        private void HandleDwell(Mechanic.Dwell.Event dwellEvent)
        {
            var phase = dwellEvent.Phase;
            var progress = dwellEvent.Progress;

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

            if (phase.IsPostExecute())
            {

            }

            if (phase.IsEnded())
            {
                Destroy(_spawned);
                _spawned = null;
            }
        }
    }
}
