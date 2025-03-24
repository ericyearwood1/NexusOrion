using System;
using UnityEngine;
using UnityEngine.Assertions;

namespace ARGlasses.Interaction
{
    public class CompoundMechanic : MonoBehaviour
    {
        [SerializeField, ReadOnly] private Selectable _selectable;
        public Selectable Selectable => this.Ensure(ref _selectable, defaultType: typeof(Target));
        public TargetState State => _selectable.State;

        [SerializeField, ReadOnly] private MechanicLongPress _longPress;
        public MechanicLongPress LongPress => this.Ensure(ref _longPress);

        [SerializeField, ReadOnly] private MechanicMultiPress _multiPress;
        public MechanicMultiPress MultiPress => this.Ensure(ref _multiPress);

        [SerializeField, ReadOnly] private MechanicDragMove _dragMove;
        public MechanicDragMove DragMove => this.Ensure(ref _dragMove);

        [SerializeField] private MechanicDragCancel _dragCancel;
        public MechanicDragCancel DragCancel => this.Ensure(ref _dragCancel);

        [SerializeField] private MechanicDragScroll _dragScroll;
        public MechanicDragScroll DragScroll => this.Ensure(ref _dragScroll);

        public event Action<DPad> WhenDPadSwipe = delegate { };

        private void Awake()
        {
            this.Ensure(ref _selectable, defaultType: typeof(Target));
            _selectable.WhenHovering += hover =>
            {
                var dPad = hover.DPad;
                if (dPad.IsNone()) return;
                _dragMove.ForceCancel();
                _longPress.ForceCancel();
                _multiPress.ForceCancel();
                WhenDPadSwipe(dPad);
            };

            this.Ensure(ref _longPress);
            Assert.AreEqual(_selectable, _longPress.Target);
            _longPress.WhenPressExecute += longPressEvent =>
            {
                _dragMove.ForceCancel();
                _multiPress.ForceCancel();
            };

            this.Ensure(ref _multiPress);
            Assert.AreEqual(_selectable, _multiPress.Target);
            _multiPress.WhenMultiPress += multiPressEvent =>
            {

            };

            this.Ensure(ref _dragMove);
            Assert.AreEqual(_selectable, _dragMove.Target);
            _dragMove.WhenDragMove += dragMove =>
            {
                if (!dragMove.Phase.IsBegin()) return;
                _longPress.ForceCancel();
                _multiPress.ForceCancel();
            };

            this.Ensure(ref _dragCancel);
            //Assert.AreEqual(_selectable, _dragCancel.Target);
            //_dragCancel.WhenStateChanged += dragCancelState => { };
            _dragCancel.WhenDisplacementLocalChanged += displacementLocal => { };

            this.Ensure(ref _dragScroll);
            Assert.AreEqual(_selectable, _dragScroll.Target);
            _dragScroll.WhenScrollEvent += scrollEvent => { };
        }

        public void Reset()
        {
            _dragMove.ForceCancel();
            _longPress.ForceCancel();
            _multiPress.ForceCancel();
        }
    }
}
