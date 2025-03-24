using ARGlasses.Interaction;
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ButtonModelEvents : MonoBehaviour
    {
        [SerializeField] private ButtonModel _buttonModel;

        public UnityEvent OnClick;
        public UnityEvent<Vector3> OnDisplacement;
        public UnityEvent OnPressBegin;
        public UnityEvent OnPressEnd;
        public UnityEvent<TargetState> OnStateChange;
        public UnityEvent<DragCancelState> OnDragCancelStateChange;
        public UnityEvent<Mechanic.DragMove.Event> OnDragMove;
        public UnityEvent<float> OnDragTwistAngle;
        public UnityEvent<ScrollEvent> OnScroll;

        protected void Awake()
        {
            this.Ensure(ref _buttonModel);

            _buttonModel.WhenClicked += HandleClick;
            _buttonModel.WhenPressBegin += HandlePressBegin;
            _buttonModel.WhenPressEnd += HandlePressEnd;
            _buttonModel.WhenStateChanged += HandleStateChange;
            _buttonModel.WhenDisplacementLocalChanged += HandleDisplacement;

            _buttonModel.MechanicDragCancel.WhenDragStateChanged += HandleDragCancelStateChange;
            _buttonModel.MechanicDragMove.WhenDragMove += HandleDragMove;
            _buttonModel.MechanicDragMove.WhenTwistRadians += HandleTwistRadians;
            _buttonModel.MechanicDragScroll.WhenScrollEvent += HandleScrollEvent;
        }

        protected void OnDestroy()
        {
            if (_buttonModel == null) return;
            _buttonModel.WhenClicked -= HandleClick;
            _buttonModel.WhenPressBegin -= HandlePressBegin;
            _buttonModel.WhenPressEnd -= HandlePressEnd;
            _buttonModel.WhenStateChanged -= HandleStateChange;
            _buttonModel.WhenDisplacementLocalChanged -= HandleDisplacement;

            _buttonModel.MechanicDragCancel.WhenDragStateChanged -= HandleDragCancelStateChange;
            _buttonModel.MechanicDragMove.WhenDragMove -= HandleDragMove;
            _buttonModel.MechanicDragMove.WhenTwistRadians -= HandleTwistRadians;
            _buttonModel.MechanicDragScroll.WhenScrollEvent -= HandleScrollEvent;
        }

        private void HandleClick()
        {
            OnClick?.Invoke();
        }

        private void HandlePressBegin()
        {
            OnPressBegin?.Invoke();
        }

        private void HandlePressEnd()
        {
            OnPressEnd?.Invoke();
        }

        private void HandleStateChange(TargetState newState)
        {
            OnStateChange?.Invoke(newState);
        }

        private void HandleDisplacement(Vector3 displacement)
        {
            OnDisplacement?.Invoke(displacement);
        }

        private void HandleDragCancelStateChange(DragCancelState newState)
        {
            OnDragCancelStateChange?.Invoke(newState);
        }

        private void HandleDragMove(Mechanic.DragMove.Event dragEvent)
        {
            OnDragMove?.Invoke(dragEvent);
        }

        private void HandleTwistRadians(float angle)
        {
            OnDragTwistAngle?.Invoke(angle);
        }

        private void HandleScrollEvent(ScrollEvent scrollEvent)
        {
            OnScroll?.Invoke(scrollEvent);
        }
    }
}
