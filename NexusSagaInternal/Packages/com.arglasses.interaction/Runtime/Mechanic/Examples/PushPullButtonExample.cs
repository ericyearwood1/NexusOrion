using System.Collections;
using UnityEngine;

namespace ARGlasses.Interaction
{
    public class PushPullButtonExample : MonoBehaviour
    {
        [SerializeField] private ButtonModel _buttonModel;
        [SerializeField] private MechanicDragMove _mechanic;
        [SerializeField] private float _resetTime = 4;
        [SerializeField] private Vector3 _deltaLocalScale = Vector3.one;

        [SerializeField] private Transform _transformToMove;

        [SerializeField, ReadOnly] private Pose _defaultLocalPosition;
        [SerializeField, ReadOnly] private Pose _grabPoseLocal;
        [SerializeField, ReadOnly] private double _lastMove;

        [SerializeField] private float _imuUpDot = 0.5f;
        [SerializeField, ReadOnly] private bool _isPalmUpImu;
        [SerializeField, ReadOnly] private bool _isPalmUpCv;

        [SerializeField, ReadOnly] private PushPullButtonLabel _label;
        [SerializeField, ReadOnly] private float _pinchZ;
        [SerializeField, ReadOnly] private bool _gestureRecognized;

        private IEnumerator Start()
        {
            this.Scene(ref _label, optional: true);
            while (!this.Descendant(ref _buttonModel, allowSibling:true, optional: true)) yield return null;
            _buttonModel.Target.WhenHovering += Handle;
            _mechanic = _buttonModel.MechanicDragMove;
            _mechanic.WhenDragMove += Handle;

            if(_transformToMove) _defaultLocalPosition = _transformToMove.ToPose(Space.Self);
        }

        void Update()
        {
            if (_transformToMove && Time.time - _lastMove > _resetTime) _transformToMove.Set(_defaultLocalPosition, Space.Self);
        }

        private void Handle(IHover hover)
        {
            _isPalmUpImu = hover.State.IsPalmUpImu(_imuUpDot);
            _isPalmUpCv = hover.State.IsPalmUpCV(_imuUpDot);
        }

        private void Handle(Mechanic.DragMove.Event dragMoveEvent)
        {
            var phase = dragMoveEvent.Phase;
            UpdateTransform(phase, dragMoveEvent.Drag.DeltaLocal);

            var currentZ = dragMoveEvent.Drag.DeltaLocal.z;
            if (phase.IsDeadzone())
            {
                _gestureRecognized = false;
                _pinchZ = currentZ;
                Debug.Log($"IconPushPullButtonExample - Begin {_pinchZ}");
            }
            if (_gestureRecognized || !phase.IsMoving()) return;

            var deltaZ = currentZ - _pinchZ;
            var thresholdExceeded = Mathf.Abs(currentZ) > 0.05f || Mathf.Abs(deltaZ) > 0.01;
            if (!thresholdExceeded) return;

            var text = deltaZ > 0 ? "Push" : "Peel";
            Debug.Log($"IconPushPullButtonExample - {text}, cur: {currentZ}, delta: {deltaZ}" );

            if(_label) _label.SetText(text);
            //_buttonModel.CompoundMechanic.Reset();
            _gestureRecognized = true;
            _pinchZ = currentZ;

        }

        private void UpdateTransform(Mechanic.DragMove.Phase phase, Vector3 dragDeltaLocal)
        {
            if (!_transformToMove) return;

            if (phase.IsBegin()) _grabPoseLocal = _transformToMove.ToPose(Space.Self);
            if (phase.IsUpdate())
            {
                var scaledLocalMotion = Vector3.Scale(dragDeltaLocal, _deltaLocalScale);
                _transformToMove.localPosition = _grabPoseLocal.position + scaledLocalMotion;
                _lastMove = Time.time;
            }
        }
    }
}
