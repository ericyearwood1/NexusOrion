// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using System;
using UnityEngine;
using UnityEngine.Serialization;
using static ARGlasses.Interaction.Mechanic.Peel;
using static ARGlasses.Interaction.Mechanic.Peel.Phase;

namespace ARGlasses.Interaction
{
    public static partial class ExtensionsMechanic
    {
        public static bool IsNone(this Phase phase) => phase == None;
        public static bool IsPre(this Phase phase) => phase == Pre;
        public static bool IsExecute(this Phase phase) => phase == Execute;
        public static bool IsPost(this Phase phase) => phase == Post;
        public static bool IsSuccess(this Phase phase) => phase == Success;
        public static bool IsCancel(this Phase phase) => phase == Cancel;

        public static bool IsMoving(this Phase phase) => phase.IsPre() || phase.IsPost();
        public static bool IsEnded(this Phase phase) => phase.IsSuccess() || phase.IsCancel() || phase.IsNone();
    }

    public static partial class Mechanic
    {
        public static class Peel
        {
            [Serializable]
            public struct Event
            {
                public Phase Phase;
                public IDrag Drag;
                public ISelection Selection;
            }

            public enum Phase
            {
                None,
                Pre,
                Execute,
                Post,
                Success,
                Cancel,
            }
        }
    }


    public class MechanicPeel : MonoBehaviour
    {
        public event Action<Mechanic.Peel.Event> WhenPeel = delegate { };

        [SerializeField] private Selectable _target;
        public Selectable Target => _target;

        [SerializeField] private float _peelDistanceZ = 0.08f;
        public float PeelDistanceZ => _peelDistanceZ;

        [SerializeField] private float _cancelDistanceXY = 0.04f;
        public float CancelDistanceXY => _cancelDistanceXY;

        [SerializeField, ReadOnly] private Mechanic.Peel.Event _event;
        private Mechanic.Peel.Event Event => _event;

        protected void Awake() => this.Ensure(ref _target, defaultType: typeof(Target));
        private void OnEnable() => _target.WhenSelecting += HandleSelecting;
        private void OnDisable() => _target.WhenSelecting -= HandleSelecting;

        private void HandleSelecting(ISelection selection)
        {
            var drag = selection.Drag;
            var dragLocal = drag.DeltaLocal;
            var isCancelled = dragLocal.WithZ(0).magnitude > _cancelDistanceXY;
            var isPeeled = -1 * dragLocal.z > _peelDistanceZ;

            _event.Drag = drag;
            _event.Selection = selection;

            if (selection.Phase.IsBegin()) DragBegin();

            if (selection.Phase.IsUpdate()) Dragging(isCancelled, isPeeled);

            if (selection.Phase.IsEnded()) DragEnd();
        }

        private void DragBegin()
        {
            _event.Phase = Pre;
            WhenPeel(_event);
        }

        private void Dragging(bool cancelled, bool peeled)
        {
            if (_event.Phase.IsEnded()) return;

            if (_event.Phase.IsPost()) WhenPeel(_event);

            if (_event.Phase.IsPre())
            {
                if (cancelled)
                {
                    _event.Phase = Cancel;
                    WhenPeel(_event);
                }
                else if (peeled)
                {
                    _event.Phase = Execute;
                    WhenPeel(_event);
                    _event.Phase = Post;
                }
                else
                {
                    WhenPeel(_event);
                }
            }
        }

        private void DragEnd()
        {
            if (_event.Phase.IsPre())
            {
                _event.Phase = Cancel;
                WhenPeel(_event);
                return;
            }

            if (_event.Phase.IsPost())
            {
                _event.Phase = Success;
                WhenPeel(_event);
            }
        }

        public void ForceSuccess()
        {
            _event.Phase = Success;
            WhenPeel(_event);
        }

        public void ForceCancel()
        {
            _event.Phase = Cancel;
            WhenPeel(_event);
        }
    }
}
