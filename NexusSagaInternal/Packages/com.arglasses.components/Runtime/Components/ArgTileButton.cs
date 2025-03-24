using ARGlasses.Interaction;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgTileButton : ViewController<TileButtonViewModel, ButtonModel, TileButtonView>
    {
        public UnityEvent WhenClick;

        private void Awake()
        {
            InitializeComponents(ensure: true);
            ForceUpdateView();
            View.Initialize(InteractionModel);
            InteractionModel.WhenClicked += Clicked;
        }

        private void Clicked()
        {
            if (View.Toggleable)
                ViewModel.Value = !ViewModel.Value;
            WhenClick?.Invoke();
        }

        private void OnDestroy()
        {
            InteractionModel.WhenClicked -= Clicked;
        }

        protected void OnValidate()
        {
            ForceUpdateView();
        }

    }
}
