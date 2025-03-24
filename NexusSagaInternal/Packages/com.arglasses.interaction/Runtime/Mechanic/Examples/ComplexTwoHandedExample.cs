using UnityEngine;

namespace ARGlasses.Interaction
{
    public class ComplexTwoHandedExample : MonoBehaviour
    {
        [SerializeField] private MechanicTwoHanded _mechanic;

        private void Awake() => this.Ensure(ref _mechanic);
        private void OnEnable() => _mechanic.WhenTwoHanded += Handle;
        private void OnDisable() => _mechanic.WhenTwoHanded -= Handle;

        [SerializeField] private float _resetTime = 4;
        [SerializeField, ReadOnly] private Pose _pointerOffset;
        [SerializeField, ReadOnly] private Pose _defaultLocalPosition;
        [SerializeField, ReadOnly] private float _lastMove;
        [SerializeField, ReadOnly] private Pose _latestPointer;
        [SerializeField, ReadOnly] private Vector3 _localScaleBegin;
        [SerializeField] private Transform _transform;

        private void Handle(Mechanic.TwoHanded.Event twoHandedEvent)
        {
            var phase = twoHandedEvent.Phase;

            var center = twoHandedEvent.Center; // current centerpoint between the hands
            var scale = twoHandedEvent.Scale; // normalized change in distance between hands

            if (!_transform) _transform = transform;

            _latestPointer = center;

            if (phase.IsBegin())
            {
                _pointerOffset = _latestPointer.InverseTransform(_transform.ToPose());
                _localScaleBegin = _transform.localScale;
            }

            if (phase.IsUpdate())
            {
                var pose = _latestPointer.Transform(_pointerOffset);
                _transform.Set(pose);
                _transform.transform.localScale = _localScaleBegin * scale;
                _lastMove = Time.time;
            }

            if (phase.IsSuccess())
            {
            }

            if (phase.IsCancel())
            {
                _transform.Set(_defaultLocalPosition, Space.Self);
            }
        }

        private void Start()
        {
            _defaultLocalPosition = transform.ToPose(Space.Self);
        }

        void Update()
        {
            if(Time.time - _lastMove > _resetTime) transform.Set(_defaultLocalPosition, Space.Self);
        }
    }
}
