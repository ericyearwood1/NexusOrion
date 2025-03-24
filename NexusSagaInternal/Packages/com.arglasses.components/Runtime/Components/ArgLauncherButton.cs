using ARGlasses.Interaction;
using UnityEngine;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgLauncherButton : ViewController<LauncherButtonViewModel, ButtonModel, LauncherButtonView>
    {

        public UnityEvent WhenClick;

        private void Start()
        {
            InitializeComponents(ensure: true);
            ForceUpdateView();
            Invoke(nameof(DelayValidate),.1f);
            InteractionModel.WhenClicked += Clicked;
        }

        void DelayValidate()
        {
            
            if(!View) Debug.Log("NO VIEW");
            if(!InteractionModel) Debug.Log("NO MODEL");
            View.Initialize(InteractionModel);
        }

        private void Clicked()
        {
            WhenClick?.Invoke();
        }

        private void OnDestroy()
        {
            if (InteractionModel != null)
                InteractionModel.WhenClicked -= Clicked;
        }

        protected void OnValidate()
        {
            ForceUpdateView();
        }
    }
}
