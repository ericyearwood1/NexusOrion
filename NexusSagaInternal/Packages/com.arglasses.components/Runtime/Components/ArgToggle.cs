using ARGlasses.Interaction;
using ITK;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgToggle : ViewController<ToggleViewModel, ToggleSliderModel, ToggleView>
    {
        public static string WhenValueChangedName => nameof(WhenValueChanged);

        public UnityEvent<bool> WhenValueChanged = new UnityEvent<bool>();
        [Group("Slider Events")] public UnityEvent WhenHandleGrabbed = new UnityEvent();
        [Group("Slider Events")] public UnityEvent WhenHandleReleased = new UnityEvent();

        public bool Value
        {
            get => InteractionModel != null ? InteractionModel.Value : ViewModel.Value;
            set
            {
                if (InteractionModel != null) InteractionModel.Value = value;
                ViewModel.Value = value;
            }
        }

        private void Awake()
        {
            InitializeComponents(ensure:true);
            ForceUpdateView();
            InteractionModel.Value = ViewModel.Value;
            InteractionModel.ForceUpdate();

            View.Initialize(InteractionModel);
            InteractionModel.WhenValueChanged += OnValueChanged;

        }

        private void OnValueChanged(bool newVal)
        {
            ViewModel.Value = newVal;
            WhenValueChanged?.Invoke(newVal);
        }

        protected void OnValidate()
        {
            ForceUpdateView();
        }
    }
}
