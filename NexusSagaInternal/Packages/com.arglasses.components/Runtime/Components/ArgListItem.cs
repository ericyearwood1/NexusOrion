using ARGlasses.Interaction;
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgListItem : ViewController<ListItemViewModel, ButtonModel, ListItemView>
    {

        public UnityEvent WhenClick;

        public RectTransform RectTransform { get; private set; }

        private void Awake()
        {
            RectTransform = GetComponent<RectTransform>();
        }

        private void Start()
        {
            InitializeComponents(ensure: true);
            ForceUpdateView();
            View.Initialize(InteractionModel);
            InteractionModel.WhenClicked += Clicked;
        }

        private void Clicked()
        {
            if (View.Toggleable)
                ViewModel.Selected = !ViewModel.Selected;
            View.Select(ViewModel.Selected);
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
