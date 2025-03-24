using UnityEngine;

namespace ARGlasses.Interaction
{
    public class GazeVelocityBuffer : MonoBehaviour
    {
        [Tooltip("When eye is moving fast, hover states will be suppressed.  Higher values are more tolerant of eye motion.")] [SerializeField]
        private float _fakeHasFixationThreshold = 50f;

        [SerializeField, ReadOnly] private bool _hasFixation;
        public bool HasFixation => _hasFixation;
        public bool IsSaccade => !_hasFixation;

        [SerializeField, ReadOnly] private float _angularVelocity;
        private Vector3 _lastGazeDirection;

        private int _bufferIndex;
        private float[] _angularVelocityBuffer = new float[5];

        [SerializeField, ReadOnly] private ARGlassesRig _rig;

        private void Awake()
        {
            this.Ancestor(ref _rig);
        }

        void Update()
        {
            var gazeDirection = _rig.EyePose.forward;
            var angle = Vector3.Angle(gazeDirection, _lastGazeDirection);
            _angularVelocityBuffer[_bufferIndex] = angle / Time.deltaTime;

            _bufferIndex = (_bufferIndex + 1) % _angularVelocityBuffer.Length;
            _angularVelocity = ExtensionsUnity.Average(_angularVelocityBuffer);
            _hasFixation = _angularVelocity < _fakeHasFixationThreshold;
            _lastGazeDirection = gazeDirection;
        }
    }
}
