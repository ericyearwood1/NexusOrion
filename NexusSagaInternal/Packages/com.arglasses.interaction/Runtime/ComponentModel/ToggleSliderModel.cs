using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace ARGlasses.Interaction
{
    public class ToggleSliderModel : MonoBehaviour, IToggle
    {
        [FormerlySerializedAs("_slider")] [SerializeField] private SliderModel sliderModel;
        [SerializeField] private bool _press;
        public SliderModel SliderModel => sliderModel;

        public event Action<TargetState> WhenStateChanged = delegate { };
        public event Action<Vector3> WhenDisplacementLocalChanged = delegate { };
        public event Action<bool> WhenValueChanged = delegate { };

        [SerializeField, ReadOnly] private float _clickCancelThreshold = 0.02f;
        [SerializeField, ReadOnly] private bool _isSelectionActive;

        protected void Awake()
        {
            this.Ensure(ref sliderModel);
            sliderModel.SetSteps(1);
        }

        private void Start()
        {
            sliderModel.Target.WhenSelecting += HandleSelecting;
            sliderModel.WhenStateChanged += HandleStateChanged;
            sliderModel.WhenDisplacementLocalChanged += v => WhenDisplacementLocalChanged(v);
            sliderModel.ScrollController.WhenLandmarkIndexChanged += HandleLandmarkIndexChanged;
        }

        public void ForceUpdate()
        {
            sliderModel.ScrollController.ApplyPendingInstantly();
        }

        private void HandleSelecting(ISelection selection)
        {
            _isSelectionActive = _isSelectionActive && selection.Drag.DeltaLocal.magnitude < _clickCancelThreshold;
        }

        // Use the ScrollController as our Model
        private void HandleLandmarkIndexChanged(int _) => WhenValueChanged(Value);

        // Use the ScrollController as our Model
        public bool Value
        {
            get => sliderModel.ScrollController.CurrentLandmarkIndex > 0;
            set
            {
                if (value == Value) return;
                sliderModel.ScrollController.PaginateAbsolute(value ? 1 : 0);
            }
        }

        private void HandleStateChanged(TargetState state)
        {
            if (state.IsPress()) _isSelectionActive = true;

            if (!state.IsPress() && _isSelectionActive)
            {
                _isSelectionActive = false;
                Value = !Value;
            }

            WhenStateChanged(state);
        }

    }
}
