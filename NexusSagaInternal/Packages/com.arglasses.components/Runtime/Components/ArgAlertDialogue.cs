using ITK;
using UnityEngine.Events;

namespace ARGlasses.Components
{
    public class ArgAlertDialogue : ViewController<AlertDialogueViewModel, AlertDialogueView>
    {

        [Group("Button Events")] public UnityEvent Button1Clicked;
        [Group("Button Events")] public UnityEvent Button2Clicked;
        [Group("Button Events")] public UnityEvent Button3Clicked;

        protected void OnValidate()
        {
            PopulateDependencies();
        }

        private void Start()
        {
            PopulateDependencies();
            View.Initialize(this);
        }

        public void PopulateDependencies()
        {
            InitializeComponents(ensure: false);
            View = GetComponentInChildren<AlertDialogueView>();
        }

    }
}
