using System.Collections.Generic;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class DeltaTimeBufferMultiTest : MonoBehaviour
    {
        private const int Capacity = 3;

        [SerializeField] private List<Vector3> _events = new List<Vector3>()
        {
            new(1, 1, 1),
            new(1, 2, 1),
            new(1, 1, 1),
            new(3, 0, 1),
            new(3, -2, 1),
            new(3, 0, 1),
        };

        [SerializeField] private float _deadzone = 1f;

        [SerializeField] private DeltaTimeBufferMulti _buffer = new(Capacity);
        [SerializeField, ReadOnly] private List<DeltaTimeBuffer.Event> _peaksX = new();
        [SerializeField, ReadOnly] private List<DeltaTimeBuffer.Event> _peaksY = new();
        [SerializeField, ReadOnly] private List<DeltaTimeBuffer.Event> _peaksZ = new();
        private List<List<DeltaTimeBuffer.Event>> _peaks = new();

        private void Update()
        {
            for (int i = 0; i < _buffer.Count; i++) _buffer.Reset();
            for (int i = 0; i < _events.Count; i++) _buffer.AddEvent(_events[i], 0.001f);

            _peaks.Clear();
            _peaks.Add(_peaksX);
            _peaks.Add(_peaksY);
            _peaks.Add(_peaksZ);
            _buffer.FindPeaks(_peaks, _deadzone);
        }
    }
}
